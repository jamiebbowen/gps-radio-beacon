#include "include/launch_detect.h"
#include "include/mpu_config.h"
#include "include/config.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// BNO085 IMU object
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

/* Every IMU report, in order. The hub batches reports that fall due
 * together into one SHTP transfer, and Adafruit_BNO08x::getSensorEvent()
 * decodes each into a single slot and returns only the LAST one - so with
 * four reports enabled at the same rate, linear acceleration batched ahead
 * of a rotation vector never reached launch detection. Our own sensor
 * callback (installed after every begin_I2C, which reinstalls the
 * library's) queues them all instead. */
#define IMU_EVQ_LEN                 16
#define IMU_MAX_TRANSFERS_PER_TICK  32   /* bounded drain per update */
static sh2_SensorValue_t imu_evq[IMU_EVQ_LEN];
static uint8_t  imu_evq_head = 0, imu_evq_count = 0;
static uint32_t imu_ev_delivered = 0;    /* reports handed over, ever */
static uint32_t imu_ev_dropped = 0;      /* queue full: oldest dropped */

static void imu_event_cb(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t v;
    if (sh2_decodeSensorEvent(&v, event) != SH2_OK) return;
    if (imu_evq_count == IMU_EVQ_LEN) {
        imu_evq_head = (uint8_t)((imu_evq_head + 1) % IMU_EVQ_LEN);
        imu_evq_count--;
        if (imu_ev_dropped++ == 0) {
            Serial.println(F("[Launch] IMU report queue overflow - dropping oldest"));
        }
    }
    imu_evq[(imu_evq_head + imu_evq_count) % IMU_EVQ_LEN] = v;
    imu_evq_count++;
    imu_ev_delivered++;
}

/* Next queued report, servicing the hub one transfer at a time while the
 * queue is empty. False once a transfer brings nothing new or the per-tick
 * budget is spent. */
static bool imu_next_event(sh2_SensorValue_t *out, int *transfers)
{
    while (imu_evq_count == 0) {
        if (*transfers >= IMU_MAX_TRANSFERS_PER_TICK) return false;
        (*transfers)++;
        uint32_t before = imu_ev_delivered;
        sh2_service();
        if (imu_ev_delivered == before) return false;
    }
    *out = imu_evq[imu_evq_head];
    imu_evq_head = (uint8_t)((imu_evq_head + 1) % IMU_EVQ_LEN);
    imu_evq_count--;
    return true;
}

// Static variables for launch detection state
static volatile launch_state_t current_state = LAUNCH_STATE_IDLE;
static volatile uint32_t launch_time_seconds = 0;
/* Separate validity flag: a launch confirmed at system time 0 (the
 * reboot-resume path runs before loop() advances the clock) is real, so
 * 0 cannot double as "never launched". */
static bool launch_time_valid = false;
static volatile uint32_t detection_start_time_ms = 0;
static volatile bool launch_detected = false;
static volatile bool imu_initialized = false;

// Current acceleration values (linear = gravity-removed, body frame)
static float accel_x = 0.0f;
static float accel_y = 0.0f;
static float accel_z = 0.0f;
static float total_accel = 0.0f;
static float peak_accel = 0.0f;   /* max |linear accel| since the last take */

/* Inertial-trace bookkeeping. During the flight window every degree of
 * rotation and burst of thrust matters for post-shred forensics; the radio
 * keeps up with a ~1 Hz IMU packet while armed. Cached here; header reads. */
static float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
static uint32_t last_gyro_ms = 0;

// Latest BNO085 rotation vector (body -> earth). w is the real part.
// If IMU_FUSION_USE_GAME_ROTVEC is set this is the game rotation vector
// (gyro+accel only, no magnetometer).
static volatile float rot_w = 1.0f;
static volatile float rot_x = 0.0f;
static volatile float rot_y = 0.0f;
static volatile float rot_z = 0.0f;
static volatile bool  rot_valid = false;
static volatile uint32_t last_accel_ms = 0;

/* GPS-altitude launch fallback state (see launch_detect_gps_fallback_update) */
static bool     gf_baseline_set = false;
static bool     gf_armed = false;
static float    gf_baseline_alt = 0.0f;
static uint8_t  gf_fix_samples = 0;
static uint32_t gf_first_fix_s = 0;
static uint32_t gf_last_fix_s = 0;
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
    total_accel = 0.0f;
    peak_accel = 0.0f;
    
    // Initialize I2C. Try 400 kHz first (BNO085 supports fast mode); the
    // long-wire risk is NAKs / corrupted reads, so probe with a chip-ID
    // read. Fall back to the always-good 100 kHz on any proven failure.
    Wire.begin();
    Wire.setClock(400000UL);
    bool fast_ok = bno08x.begin_I2C(IMU_I2C_ADDR);
    if (fast_ok) {
        /* Verify with a benign re-read of a fixed register value. The
         * BNO08x begin already round-trips several transfers; that plus
         * the first real enableReport is the probe. */
    } else {
        Serial.println("[Launch] BNO08x: 400 kHz failed, retrying 100 kHz");
        Wire.end();
        Wire.begin();
        Wire.setClock(100000UL);
        fast_ok = bno08x.begin_I2C(IMU_I2C_ADDR);
    }

    if (!fast_ok) {
        Serial.println("[Launch] ✗ Failed to find BNO08x chip");
        imu_initialized = false;
        return;
    }
    Serial.println("[Launch] ✓ BNO08x Found!");
    imu_evq_head = imu_evq_count = 0;
    imu_ev_dropped = 0;
    sh2_setSensorCallback(imu_event_cb, NULL);   /* every report, not the last */
    
    // Enable accelerometer reports
    // Using linear acceleration (gravity removed) for better launch detection
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
        Serial.println("[Launch] ✗ Could not enable linear acceleration");
        imu_initialized = false;
        return;
    }

    Serial.println("[Launch] ✓ Linear acceleration enabled");

    // Calibrated gyroscope alongside: rotation rates are the only measurable
    // witness to "which axis went first" in a mid-flight structural failure.
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("[Launch] ⚠ Gyro report unavailable - inertial packet without it");
    } else {
        Serial.println("[Launch] ✓ Gyroscope enabled");
    }

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
    launch_time_valid = false;
    detection_start_time_ms = 0;
    launch_detected = false;
    gf_baseline_set = false;
    gf_armed = false;
    gf_fix_samples = 0;
    gf_climb_samples = 0;
    landed = false;
    land_window_start_s = 0;
    imu_initialized = true;
    
    Serial.println("[Launch] ✓ Launch detection ready");
}

/**
 * Update launch detection state machine (call regularly from main loop)
 */
void launch_detect_update(uint32_t system_time_seconds) {
    if (!imu_initialized) {
        return;  // Can't detect if IMU isn't working
    }
    
    // Read sensor data
    if (bno08x.wasReset()) {
        Serial.println("[Launch] Sensor was reset, re-enabling reports");
        bno08x.enableReport(SH2_LINEAR_ACCELERATION);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
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
    int imu_transfers = 0;
    while (imu_next_event(&sensorValue, &imu_transfers)) {

    // Capture rotation vector whenever the sensor emits one.  Both the full
    // and game variants land in the same union field (rotationVector).
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR
        || sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        rot_w = sensorValue.un.rotationVector.real;
        rot_x = sensorValue.un.rotationVector.i;
        rot_y = sensorValue.un.rotationVector.j;
        rot_z = sensorValue.un.rotationVector.k;
        rot_valid = true;
    }

    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        gyro_x = sensorValue.un.gyroscope.x;
        gyro_y = sensorValue.un.gyroscope.y;
        gyro_z = sensorValue.un.gyroscope.z;
        last_gyro_ms = millis();
    }

    // Process linear acceleration data (gravity already removed by sensor)
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        last_accel_ms = millis();
        accel_x = sensorValue.un.linearAcceleration.x;
        accel_y = sensorValue.un.linearAcceleration.y;
        accel_z = sensorValue.un.linearAcceleration.z;
        
        // Calculate total acceleration magnitude
        total_accel = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        if (total_accel > peak_accel) peak_accel = total_accel;
        
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
                        launch_time_valid = true;
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
    }  // end while(imu_next_event)
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

/* ---------- GPS-altitude launch fallback (IMU-independent) --------------- */

/**
 * Feed each valid GPS altitude fix (call at the 1 Hz GPS rate). If the
 * altitude climbs >= LAUNCH_GPS_ALT_CLIMB_M above the pad baseline for
 * LAUNCH_GPS_ALT_CLIMB_SAMPLES consecutive samples, confirm launch exactly
 * as the IMU path would (same state + one-shot edge), so a dead IMU no
 * longer pins the beacon at pad cadence for the whole flight.
 * Returns true on the sample that confirms launch.
 *
 * Baseline = running maximum of every valid fix the pad ever reports.
 * The first fix used to freeze the baseline instantly: GPS cold-start
 * fixes are often tens of meters LOW for several seconds, so when the
 * error decayed the recovery "climb" exceeded +50 m for 3+ samples and
 * confirmed a launch that never happened. Two guards now prevent that:
 *  - SETTLE: the climb test stays disarmed until the fix stream has run
 *    LAUNCH_GPS_BASELINE_SETTLE_S seconds with enough samples; during
 *    settle the baseline just tracks the max, absorbing the correction.
 *  - RATCHET: after arming, a non-triggering sample still raises the
 *    baseline, so slow pad drift can never accumulate into a fake climb
 *    (a real launch climbs far faster than drift can lift the max).
 * A fix outage longer than the settle window re-enters settle - a
 * re-acquired fix gets the same cold-start transient treatment.
 */
bool launch_detect_gps_fallback_update(float alt_m, uint32_t system_time_seconds) {
    if (current_state == LAUNCH_STATE_CONFIRMED) return false;  /* already flying */
    /* A corrupt NMEA altitude parses via atof() into NaN/Inf; one bad
     * sample would poison gf_baseline_alt permanently (every comparison
     * against NaN fails, so the fallback could never fire). */
    if (!isfinite(alt_m)) return false;

    if (!gf_baseline_set) {
        gf_baseline_alt = alt_m;   /* first seed for the max-tracked baseline */
        gf_baseline_set = true;
        gf_first_fix_s = system_time_seconds;
        gf_last_fix_s  = system_time_seconds;
        gf_fix_samples = 1;
        gf_climb_samples = 0;
        return false;
    }

    /* Long outage: the re-acquired stream is a fresh cold start. Stay
     * conservative - keep the max baseline but disarm the climb test for
     * another settle window. */
    if (gf_armed &&
        (system_time_seconds - gf_last_fix_s) > LAUNCH_GPS_BASELINE_SETTLE_S) {
        gf_armed = false;
        gf_first_fix_s = system_time_seconds;
        gf_fix_samples = 0;
    }
    gf_last_fix_s = system_time_seconds;
    gf_fix_samples++;

    if (!gf_armed) {
        if (alt_m > gf_baseline_alt) gf_baseline_alt = alt_m;   /* max track */
        if ((system_time_seconds - gf_first_fix_s) >= LAUNCH_GPS_BASELINE_SETTLE_S &&
            gf_fix_samples >= LAUNCH_GPS_BASELINE_MIN_FIXES) {
            gf_armed = true;
        }
        return false;
    }

    if (alt_m >= gf_baseline_alt + LAUNCH_GPS_ALT_CLIMB_M) {
        if (++gf_climb_samples >= LAUNCH_GPS_ALT_CLIMB_SAMPLES) {
            Serial.print(F("[Launch] GPS-alt fallback: climb of "));
            Serial.print(alt_m - gf_baseline_alt);
            Serial.println(F(" m sustained - LAUNCH CONFIRMED"));
            current_state = LAUNCH_STATE_CONFIRMED;
            launch_time_seconds = system_time_seconds;
            launch_time_valid = true;
            launch_detected = true;
            return true;
        }
    } else {
        gf_climb_samples = 0;
        /* Non-triggering max still raises the baseline: slow pad drift
         * (thermal, multipath wander) is absorbed instead of accumulating
         * toward the +50 m threshold. */
        if (alt_m > gf_baseline_alt) gf_baseline_alt = alt_m;
    }
    return false;
}

static uint8_t  land_outlier_streak = 0;   /* consecutive out-of-window samples */

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
    /* A corrupt NMEA altitude parses via atof() into NaN/Inf. NaN seeds
     * land_window_min/max and (NaN-NaN) > window is always false, so
     * every later sample would read "stable" - a false LANDING could
     * latch mid-flight. Treat as a no-fix sample (breaks the window). */
    if (gps_valid && !isfinite(alt_m)) gps_valid = false;

    bool quiet = (total_accel < LAND_ACCEL_QUIET_MS2);

    if (!quiet || !gps_valid) {
        land_window_start_s = 0;   /* window broken; restart next second */
        return false;
    }

    if (land_window_start_s == 0) {
        land_window_start_s = system_time_seconds;
        land_window_min = land_window_max = alt_m;
        land_outlier_streak = 0;
        return false;
    }

    /* Multipath guard: rockets land in trees and gullies where resting GPS
     * altitude throws isolated 20-30 m spikes. A single outlier must not
     * flush a nearly-complete 60 s window and cost the battery-save
     * transition (the beacon stays at recovery power overnight and dies
     * before the walk). One outlier: tolerated (not folded into min/max,
     * window keeps running). Two IN A ROW: the spread is real motion -
     * the rocket is still sliding - reset as before. */
    float cand_min = (alt_m < land_window_min) ? alt_m : land_window_min;
    float cand_max = (alt_m > land_window_max) ? alt_m : land_window_max;
    if ((cand_max - cand_min) > LAND_ALT_WINDOW_M) {
        if (++land_outlier_streak >= 2) {
            land_window_start_s = 0;   /* genuinely still moving vertically */
            land_outlier_streak = 0;
        }
        return false;
    }
    land_outlier_streak = 0;
    land_window_min = cand_min;
    land_window_max = cand_max;

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

void launch_detect_force_confirmed(uint32_t system_time_seconds) {
    /* CONFIRMED with NO edge: is_launched() must not fire, or firmware.ino
     * would re-run the launch block and erase the resumed flight log.
     * launch_time_seconds anchors t_since_launch at the resume moment -
     * the true T0 is lost with the reboot, but event pacing only needs a
     * consistent reference. */
    current_state = LAUNCH_STATE_CONFIRMED;
    launch_time_seconds = system_time_seconds;
    launch_time_valid = true;
    launch_detected = false;
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
    if (launch_time_valid) {
        return (system_time_seconds - launch_time_seconds);
    }
    return 0;
}

/**
 * Get current total acceleration magnitude (m/s²)
 */
/* Peak |linear accel| (m/s^2) since the previous call - every report
 * counts, not just the one current when the IMU trace packet is built.
 * The next window starts at the current magnitude. */
float launch_detect_take_peak_accel(void) {
    float p = (total_accel > peak_accel) ? total_accel : peak_accel;
    peak_accel = total_accel;
    return p;
}

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
void launch_detect_get_gyro_rads(float* x, float* y, float* z) {
    if (x) *x = gyro_x;
    if (y) *y = gyro_y;
    if (z) *z = gyro_z;
}


void launch_detect_get_accel_xyz(float* x, float* y, float* z) {
    if (x) *x = accel_x;
    if (y) *y = accel_y;
    if (z) *z = accel_z;
}
