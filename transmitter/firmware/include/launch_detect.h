#ifndef LAUNCH_DETECT_H
#define LAUNCH_DETECT_H

#include <stdint.h>
#include <stdbool.h>
#include "mpu_config.h"

// Launch detection using BNO085 IMU (acceleration-based)

// Launch detection states
typedef enum {
    LAUNCH_STATE_IDLE = 0,          // On pad, waiting for launch
    LAUNCH_STATE_DETECTING = 1,     // Acceleration above threshold
    LAUNCH_STATE_CONFIRMED = 2,     // Launch confirmed (sustained acceleration)
} launch_state_t;

// Function prototypes
void launch_detect_init(void);
void launch_detect_update(uint32_t system_time_seconds, uint32_t ms_counter);
bool launch_detect_is_launched(void);
launch_state_t launch_detect_get_state(void);
uint32_t launch_detect_get_time_since_launch(uint32_t system_time_seconds);

// Diagnostic functions
float launch_detect_get_current_accel(void);      // Get current total acceleration (m/s²)
bool launch_detect_get_imu_status(void);          // Check if IMU is working
void launch_detect_get_accel_xyz(float* x, float* y, float* z);  // Get 3-axis acceleration

#endif // LAUNCH_DETECT_H
