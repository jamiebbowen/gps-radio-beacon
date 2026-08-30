#ifndef CONFIG_H
#define CONFIG_H

/**
 * Configuration Header for GPS Radio Beacon
 * =========================================
 * 
 * This file contains compile-time configuration options for the GPS radio beacon.
 * Toggle TESTING_MODE to switch between production and testing configurations.
 */

#define LAUNCH_DEBOUNCE_TIME_MS         200   // 200ms debounce time for launch detection

// Set to 1 for testing mode, 0 for production mode
#define TESTING_MODE 0

/* Minimum seconds between no-fix heartbeat packets. A heartbeat is sent when
 * a beacon TX was requested but no transmittable GPS fix exists, keeping the
 * receiver's channel scan and operator informed that the beacon is alive.
 * Matches the production pad interval so the RX scan dwell (6.5 s) still
 * exceeds the slowest pad-state packet spacing. */
#define HEARTBEAT_INTERVAL_SEC 5

/**
 * Bench-test switch: when 1, the beacon boots directly into BEACON_STATE_LAUNCH
 * so the TX continuously streams packets as if a launch had been detected.
 * Useful for exercising the receiver (e.g. compass cal persistence, heading
 * valid indicator, LoRa link) without a physical launch event or GPS fix.
 *
 * MUST be 0 for flight.
 */
#define BENCH_TEST_FORCE_LAUNCH         0

/* ---------------------------------------------------------------------------
 * IMU / GPS sensor fusion (6-state EKF: pos + vel in local NED frame)
 * -------------------------------------------------------------------------*/

/* Master switch. When 1, the beacon transmits PACKET_TYPE_FUSED packets in
 * addition to PACKET_TYPE_GPS. When 0, the EKF still runs (so residuals can
 * be logged to Serial for tuning) but no fused packets go over the air. */
#define IMU_FUSION_ENABLED              1

/* When 1, print GPS measurement innovation and fused state to Serial after
 * every EKF update. Useful for σ tuning from recorded bench or flight data. */
#define IMU_FUSION_LOG_RESIDUALS        1

/* Use the BNO085 Game Rotation Vector (gyro + accel only, no magnetometer)
 * instead of the full Rotation Vector. Set to 1 if the airframe has ferrous
 * parts or motors that bias the magnetometer. Trade-off: heading slowly
 * drifts over tens of minutes, but horizontal position integration becomes
 * immune to magnetic disturbances. */
#define IMU_FUSION_USE_GAME_ROTVEC      0

/* Seconds without a fresh GPS fix before we flag the fused output as
 * "dead reckoning" (FUSED_FLAG_DEAD_RECKONING). Past this horizon integration
 * drift dominates and consumers should treat the position as a soft hint. */
#define NAV_DR_TIMEOUT_S                3.0f

/* EKF tuning. These default values are conservative; tune from logs. */
#define EKF_SIGMA_ACCEL_MS2             0.5f   /* process noise (m/s^2)  */
#define EKF_SIGMA_GPS_HORIZ_M           3.0f   /* GPS horiz meas noise   */
#define EKF_SIGMA_GPS_VERT_M            6.0f   /* GPS vertical noise     */
#define EKF_INIT_POS_VAR_M2             25.0f  /* initial P diag (pos)   */
#define EKF_INIT_VEL_VAR_MS2            4.0f   /* initial P diag (vel)   */

/* GPS innovation gate (nav_update_from_gps). A fix whose normalized
 * innovation (chi-square-ish, 3 dof) exceeds NAV_NIS_REJECT (~4 sigma) AND
 * is more than NAV_GATE_MIN_M from the filter's own estimate is treated as
 * a multipath/stale-position glitch and rejected. Armed only while the IMU
 * is healthy (predict tracks real dynamics); with a dead IMU the filter
 * coasts and honest boost-phase motion produces big innovations, so the
 * gate fails open and GPS flows ungated. */
#define NAV_NIS_REJECT                  16.0f  /* 3-dof chi2 99.9% ~= 16.3 */
#define NAV_GATE_MIN_M                  20.0f  /* never reject sub-20 m steps */

/* The tangent-plane anchor requires two consecutive fixes within this
 * radius of each other: a lone glitch fix can't anchor the EKF wrong. */
#define NAV_ANCHOR_CONFIRM_M            30.0f

/* Fused-packet transmit cadence in LAUNCH state. Set to 10 Hz so we get
 * smoothed interpolation between 1 Hz GPS fixes. Ignored when state machine
 * isn't in LAUNCH / POST_LAUNCH (uses state-machine cadence there). */
#define FUSED_TX_INTERVAL_MS            100

#if TESTING_MODE
    // Testing Configuration - Fast intervals for development/testing
    #define PRE_LAUNCH_INTERVAL_SEC         30    // 30 seconds between transmissions in pre-launch
    #define POST_LAUNCH_DURATION_SEC        10    // 10 seconds in LAUNCH state before POST_LAUNCH
    #define POST_LAUNCH_RECOVERY_DURATION_SEC 1200  // 20 minutes duration in post-launch state
    #define BATTERY_SAVE_INTERVAL_SEC       30    // 30 seconds between transmissions in battery save
    #define CALLSIGN_TRANSMIT_INTERVAL_SEC  30    // 30 seconds between callsign transmissions
    
    // Debug features enabled in testing mode
    #define DEBUG_OUTPUT_ENABLED        1     // Enable debug output
    #define INCLUDE_CARRIAGE_RETURNS    1     // Include \r\n for terminal debugging
    
#else
    // Production Configuration - Conservative intervals for flight
    #define PRE_LAUNCH_INTERVAL_SEC         5     // 5 seconds between transmissions in pre-launch
    #define POST_LAUNCH_DURATION_SEC        1     // 1 second in LAUNCH state before POST_LAUNCH
    #define POST_LAUNCH_RECOVERY_DURATION_SEC 600 // 10 minutes duration in post-launch state
    #define BATTERY_SAVE_INTERVAL_SEC       60    // 60 seconds between transmissions in battery save
    #define CALLSIGN_TRANSMIT_INTERVAL_SEC  300   // 5 minutes between callsign transmissions
    
    // Debug features disabled in production mode
    #define DEBUG_OUTPUT_ENABLED        0     // Disable debug output
    #define INCLUDE_CARRIAGE_RETURNS    0     // No \r\n in production (RF efficiency)
    
#endif

#endif // CONFIG_H
