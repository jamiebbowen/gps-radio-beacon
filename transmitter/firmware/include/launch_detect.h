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

/**
 * ONE-SHOT edge trigger: returns true exactly once after launch is confirmed,
 * then clears the internal flag. Owned by the beacon state machine in
 * firmware.ino - do NOT call from anywhere else. For a level query
 * ("has a launch happened?") use:
 *   launch_detect_get_state() == LAUNCH_STATE_CONFIRMED
 */
bool launch_detect_is_launched(void);

/** Level query: current launch detection state (never auto-clears). */
launch_state_t launch_detect_get_state(void);

/* GPS-altitude launch fallback: feed each valid 1 Hz GPS altitude fix.
 * Confirms launch (same state/edge as the IMU path) on a sustained climb
 * above the pad baseline - covers a dead IMU. Returns true on confirmation. */
bool launch_detect_gps_fallback_update(float alt_m, uint32_t system_time_seconds);

/* Landing detection: feed once per second. Latches true when post-launch
 * linear accel is quiet AND GPS altitude stays stable for LAND_QUIET_S. */
bool landing_detect_update(float alt_m, bool gps_valid, uint32_t system_time_seconds);
bool launch_detect_has_landed(void);

/** Seconds since launch confirmation, or 0 if not launched yet. */
uint32_t launch_detect_get_time_since_launch(uint32_t system_time_seconds);

// Diagnostic functions
float launch_detect_get_current_accel(void);      // Get current total acceleration (m/s²)
bool launch_detect_get_imu_status(void);          // Check if IMU is working
void launch_detect_get_accel_xyz(float* x, float* y, float* z);  // Get 3-axis acceleration

/* ---------- IMU accessors (populated by launch_detect_update) ------------ */
/* Body-frame linear acceleration (gravity removed by BNO085 fusion), m/s^2 */
void     imu_get_linear_accel_body(float *x, float *y, float *z);
/* Body -> earth rotation quaternion (w, x, y, z).  Identity if not yet valid */
void     imu_get_quaternion(float *w, float *x, float *y, float *z);
bool     imu_has_quaternion(void);
uint32_t imu_last_accel_ms(void);
uint32_t imu_last_rotvec_ms(void);

#endif // LAUNCH_DETECT_H
