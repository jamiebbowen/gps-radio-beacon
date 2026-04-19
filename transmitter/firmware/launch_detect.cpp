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

// Current acceleration values
static float accel_x = 0.0f;
static float accel_y = 0.0f;
static float accel_z = 0.0f;
static float total_accel = 0.0f;

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
    }
    
    // Get new sensor event
    if (!bno08x.getSensorEvent(&sensorValue)) {
        return;  // No new data
    }
    
    // Process linear acceleration data (gravity already removed by sensor)
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
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
}

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
