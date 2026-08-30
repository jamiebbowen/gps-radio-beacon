#include "include/launch_detect.h"
#include "include/mpu_config.h"
#include "include/config.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// BNO085 IMU object
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Static variables for launch detection state
static volatile launch_state_t current_state = LAUNCH_STATE_IDLE;
static volatile uint32_t launch_time_seconds = 0;
static volatile uint32_t detection_start_time_ms = 0;
static volatile bool launch_detected = false;
static volatile bool imu_initialized = false;

// Current acceleration values (linear = gravity-removed, body frame)
static float accel_x = 0.0f;
static float accel_y = 0.0f;
static float accel_z = 0.0f;
static float total_accel = 0.0f;

// Latest BNO085 rotation vector (body -> earth). w is the real part.
// If IMU_FUSION_USE_GAME_ROTVEC is set this is the game rotation vector
// (gyro+accel only, no magnetometer).
static volatile float rot_w = 1.0f;
static volatile float rot_x = 0.0f;
static volatile float rot_y = 0.0f;
static volatile float rot_z = 0.0f;
static volatile bool  rot_valid = false;
static volatile uint32_t last_accel_ms = 0;
static volatile uint32_t last_rotvec_ms = 0;

/* GPS-altitude launch fallback state (see launch_detect_gps_fallback_update) */
static bool     gf_baseline_set = false;
static float    gf_baseline_alt = 0.0f;
static uint8_t  gf_climb_samples = 0;

/* Landing detection state (see landing_detect_update) */
static bool     landed = false;
static uint32_t land_window_start_s = 0;
static float    land_window_min = 0.0f;
static float    land_window_max = 0.0f;

/**
 * Initialize BNO085 IMU for launch detection
 */
void launch_detect_init(void) {
    Serial.println("[Launch] Initializing BNO085 IMU...");
    
    // Initialize I2C
    Wire.begin();
    
    // Try to initialize the sensor
    if (!bno08x.begin_I2C(IMU_I2C_ADDR)) {
        Serial.println("[Launch] ✗ Failed to find BNO08x chip");
        imu_initialized = false;
        return;
    }
    
    Serial.println("[Launch] ✓ BNO08x Found!");
    
    // Enable accelerometer reports
    // Using linear acceleration (gravity removed) for better launch detection
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
        Serial.println("[Launch] ✗ Could not enable linear acceleration");
        imu_initialized = false;
        return;
    }
    
    Serial.println("[Launch] ✓ Linear acceleration enabled");

    // Also enable a rotation vector report so the nav/EKF layer can rotate
    // body-frame accel into earth-frame.  GAME_ROTATION_VECTOR omits the
    // magnetometer (good for metal airframes); ROTATION_VECTOR uses full
    // 9-DoF fusion (more accurate heading, but mag-dependent).
#if IMU_FUSION_USE_GAME_ROTVEC
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("[Launch] ⚠ Could not enable GAME rotation vector (fusion will miss orientation)");
    } else {
        Serial.println("[Launch] ✓ Game rotation vector enabled");
    }
#else
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("[Launch] ⚠ Could not enable rotation vector (fusion will miss orientation)");
    } else {
        Serial.println("[Launch] ✓ Rotation vector enabled");
    }
#endif

    // Wait for sensor to settle
    Serial.print("[Launch] Settling for ");
    Serial.print(LAUNCH_SETTLE_TIME);
    Serial.println(" ms...");
    delay(LAUNCH_SETTLE_TIME);
    
    // Initialize state variables
    current_state = LAUNCH_STATE_IDLE;
    launch_time_seconds = 0;
    detection_start_time_ms = 0;
    launch_detected = false;
    gf_baseline_set = false;
    gf_climb_samples = 0;
    landed = false;
    land_window_start_s = 0;
    imu_initialized = true;
    
    Serial.println("[Launch] ✓ Launch detection ready");
}

/**
 * Update launch detection state machine (call regularly from main loop)
 */
void launch_detect_update(uint32_t system_time_seconds, uint32_t ms_counter) {
    if (!imu_initialized) {
        return;  // Can't detect if IMU isn't working
    }
    
    // Read sensor data
    if (bno08x.wasReset()) {
        Serial.println("[Launch] Sensor was reset, re-enabling reports");
        bno08x.enableReport(SH2_LINEAR_ACCELERATION);
#if IMU_FUSION_USE_GAME_ROTVEC
        bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
#else
        bno08x.enableReport(SH2_ROTATION_VECTOR);
#endif
        rot_valid = false;
    }
    
    // Drain all queued sensor events this tick, not just one.  At 100 Hz+ the
    // BNO085 can produce several reports between main-loop iterations, and
    // only consuming one starves the nav/EKF layer of rotation updates.
    while (bno08x.getSensorEvent(&sensorValue)) {

    // Capture rotation vector whenever the sensor emits one.  Both the full
    // and game variants land in the same union field (rotationVector).
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR
        || sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        rot_w = sensorValue.un.rotationVector.real;
        rot_x = sensorValue.un.rotationVector.i;
        rot_y = sensorValue.un.rotationVector.j;
        rot_z = sensorValue.un.rotationVector.k;
        rot_valid = true;
        last_rotvec_ms = millis();
    }

    // Process linear acceleration data (gravity already removed by sensor)
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        last_accel_ms = millis();
        accel_x = sensorValue.un.linearAcceleration.x;
        accel_y = sensorValue.un.linearAcceleration.y;
        accel_z = sensorValue.un.linearAcceleration.z;
        
        // Calculate total acceleration magnitude
        total_accel = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        
        // State machine for launch detection
        switch (current_state) {
            case LAUNCH_STATE_IDLE:
                // Check if acceleration exceeds threshold
                if (total_accel > LAUNCH_ACCEL_THRESHOLD) {
                    Serial.print("[Launch] Acceleration detected: ");
                    Serial.print(total_accel);
                    Serial.println(" m/s²");
                    
                    current_state = LAUNCH_STATE_DETECTING;
                    detection_start_time_ms = millis();
                }
                break;
                
            case LAUNCH_STATE_DETECTING:
                // Check if acceleration is still above threshold
                if (total_accel > LAUNCH_ACCEL_THRESHOLD) {
                    // Check if we've sustained this acceleration long enough
                    uint32_t duration = millis() - detection_start_time_ms;
                    if (duration >= LAUNCH_ACCEL_DURATION) {
                        // Launch confirmed!
                        Serial.println("[Launch] ✓ LAUNCH CONFIRMED!");
                        Serial.print("[Launch] Peak acceleration: ");
                        Serial.print(total_accel);
                        Serial.println(" m/s²");
                        
                        current_state = LAUNCH_STATE_CONFIRMED;
                        launch_time_seconds = system_time_seconds;
                        launch_detected = true;
                    }
                } else {
                    // Acceleration dropped below threshold - false alarm
                    Serial.println("[Launch] False alarm, acceleration dropped");
                    current_state = LAUNCH_STATE_IDLE;
                }
                break;
                
            case LAUNCH_STATE_CONFIRMED:
                // Stay in confirmed state - launch already detected
                break;
        }
    }
    }  // end while(getSensorEvent)
}

/* ---------- IMU accessors for the nav / EKF layer ------------------------ */

void imu_get_linear_accel_body(float *x, float *y, float *z) {
    if (x) *x = accel_x;
    if (y) *y = accel_y;
    if (z) *z = accel_z;
}

void imu_get_quaternion(float *w, float *xq, float *yq, float *zq) {
    if (w)  *w  = rot_w;
    if (xq) *xq = rot_x;
    if (yq) *yq = rot_y;
    if (zq) *zq = rot_z;
}

bool imu_has_quaternion(void) {
    return rot_valid;
}

uint32_t imu_last_accel_ms(void)  { return last_accel_ms; }
uint32_t imu_last_rotvec_ms(void) { return last_rotvec_ms; }

/* ---------- GPS-altitude launch fallback (IMU-independent) --------------- */

/**
 * Feed each valid GPS altitude fix (call at the 1 Hz GPS rate). If the
 * altitude climbs >= LAUNCH_GPS_ALT_CLIMB_M above the pad baseline for
 * LAUNCH_GPS_ALT_CLIMB_SAMPLES consecutive samples, confirm launch exactly
 * as the IMU path would (same state + one-shot edge), so a dead IMU no
 * longer pins the beacon at pad cadence for the whole flight.
 * Returns true on the sample that confirms launch.
 */
bool launch_detect_gps_fallback_update(float alt_m, uint32_t system_time_seconds) {
    if (current_state == LAUNCH_STATE_CONFIRMED) return false;  /* already flying */

    if (!gf_baseline_set) {
        gf_baseline_alt = alt_m;   /* pad altitude baseline */
        gf_baseline_set = true;
        gf_climb_samples = 0;
        return false;
    }

    if (alt_m >= gf_baseline_alt + LAUNCH_GPS_ALT_CLIMB_M) {
        if (++gf_climb_samples >= LAUNCH_GPS_ALT_CLIMB_SAMPLES) {
            Serial.print(F("[Launch] GPS-alt fallback: climb of "));
            Serial.print(alt_m - gf_baseline_alt);
            Serial.println(F(" m sustained - LAUNCH CONFIRMED"));
            current_state = LAUNCH_STATE_CONFIRMED;
            launch_time_seconds = system_time_seconds;
            launch_detected = true;
            return true;
        }
    } else {
        gf_climb_samples = 0;
    }
    return false;
}

/* ---------- Landing detection (post-launch) ------------------------------ */

/**
 * Feed once per second with the current state. Landing = linear accel quiet
 * AND GPS altitude stable, sustained for LAND_QUIET_S, after a confirmed
 * launch. Steady parachute descent passes the accel test but never the
 * altitude-stability test, so it cannot false-trigger. Latches once true.
 */
bool landing_detect_update(float alt_m, bool gps_valid, uint32_t system_time_seconds) {
    if (landed) return true;
    if (current_state != LAUNCH_STATE_CONFIRMED) return false;
    if (!imu_initialized) return false;   /* need the accel-quiet evidence */

    bool quiet = (total_accel < LAND_ACCEL_QUIET_MS2);

    if (!quiet || !gps_valid) {
        land_window_start_s = 0;   /* window broken; restart next second */
        return false;
    }

    if (land_window_start_s == 0) {
        land_window_start_s = system_time_seconds;
        land_window_min = land_window_max = alt_m;
        return false;
    }

    if (alt_m < land_window_min) land_window_min = alt_m;
    if (alt_m > land_window_max) land_window_max = alt_m;

    if ((land_window_max - land_window_min) > LAND_ALT_WINDOW_M) {
        land_window_start_s = 0;   /* still moving vertically */
        return false;
    }

    if ((system_time_seconds - land_window_start_s) >= LAND_QUIET_S) {
        landed = true;
        Serial.println(F("[Launch] LANDING DETECTED (quiet + stable alt)"));
        return true;
    }
    return false;
}

bool launch_detect_has_landed(void) { return landed; }

/**
 * Check if launch has been detected and confirmed (and clear flag)
 */
bool launch_detect_is_launched(void) {
    if (launch_detected) {
        launch_detected = false;  // Clear the flag
        return true;
    }
    return false;
}

/**
 * Get current launch detection state
 */
launch_state_t launch_detect_get_state(void) {
    return current_state;
}

/**
 * Get time since launch was detected (in seconds)
 */
uint32_t launch_detect_get_time_since_launch(uint32_t system_time_seconds) {
    if (launch_time_seconds > 0) {
        return (system_time_seconds - launch_time_seconds);
    }
    return 0;
}

/**
 * Get current total acceleration magnitude (m/s²)
 */
float launch_detect_get_current_accel(void) {
    return total_accel;
}

/**
 * Check if IMU is initialized and working
 */
bool launch_detect_get_imu_status(void) {
    return imu_initialized;
}

/**
 * Get individual acceleration components
 */
void launch_detect_get_accel_xyz(float* x, float* y, float* z) {
    if (x) *x = accel_x;
    if (y) *y = accel_y;
    if (z) *z = accel_z;
}
