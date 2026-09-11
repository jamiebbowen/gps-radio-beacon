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
 * 5 s while range-testing (dense = fast scan lock + marginal-link evidence).
 * For flight-prep benching, consider 30 s so pad bursts leave the GPS
 * front ends long clean windows (see git history / reliability.md). */
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

/* Milliseconds of BNO085 silence before the fused output is flagged
 * FUSED_FLAG_SENSOR_DEGRADED ("dead or in retry"). The BNO085 streams at
 * 100 Hz, so even the worst legitimate gap is tens of ms; 2 s of silence
 * means a wedged I2C bus, a chip reset being recovered via wasReset(), or
 * the chip never answered at boot. This is deliberately 4x the 500 ms
 * imu_healthy staleness window: healthy/degraded answers different
 * questions ("data stale right now" vs "the sensor is gone"). */
#define NAV_SENSOR_DEAD_MS              2000u

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

/* EKF rescue policy. The innovation gate is only trustworthy against a
 * HEALTHY filter: after NAV_RESCUE_NOFIX_MS without an accepted fix the
 * filter itself is the suspect (its coast state can be pure fiction and
 * every honest fix looks like a glitch). Past that, the gate fails open
 * and fixes must pass a motion-consistency pair check (two fixes within
 * NAV_RESCUE_MAX_STEP_M and 5 s of each other). If the confirmed fix is
 * more than NAV_REANCHOR_M from the tangent origin, the anchor itself is
 * re-based and the filter hard-reset there. (2026-09-11 field log: a
 * ~5 min GPS hole left 40 m/s of phantom velocity; the gate then rejected
 * good fixes forever and the fused stream trailed off 22 km away.) */
#define NAV_RESCUE_NOFIX_MS     15000UL /* fail the gate open past this   */
#define NAV_REANCHOR_M          2000.0f /* rescue: re-base anchor past    */
#define NAV_RESCUE_MAX_STEP_M   300.0f  /* pair-consistency radius        */

/* The tangent-plane anchor requires two consecutive fixes within this
 * radius of each other: a lone glitch fix can't anchor the EKF wrong. */
#define NAV_ANCHOR_CONFIRM_M            30.0f

/* Post-launch recovery-mode pacing between raw GPS packets (seconds).
 * Fused packets interleave at FUSED_TX_INTERVAL_MS, so position updates
 * land roughly every 0.6 s anyway - the raw stream here is the audit
 * copy. Free-running BOTH streams at SF10 would key the M33S PA at ~100%
 * duty for the entire POST_LAUNCH recovery window: thermal stress the
 * module class isn't rated for, plus needless current. */
#define POST_LAUNCH_PACKET_INTERVAL_SEC 2

/* Fused-packet transmit cadence in LAUNCH state. Must exceed the fused
 * packet's air time (SF10/BW62.5k/CR4-6/8-sym-preamble, 19 bytes ≈
 * 0.83 s); 1200 ms gives ~0.8 fused updates/s in flight with comfortable
 * margin. Faster is impossible without airtime exceeding the interval,
 * which just converts the "rate" into back-to-back blocking transmits. */
#define FUSED_TX_INTERVAL_MS            1200

/* Fused-packet cadence in every non-flight state (pad idle, post-landing
 * battery-save). Was 1 Hz: at full PA drive that ~170 ms-every-second
 * burst pattern desenses GPS front ends nearby (the beacon's own included)
 * - field logs showed the beacon GPS needing three watchdog recovery rounds
 * before its first fix. Every 10 s cuts pad duty from ~17% to under 2%
 * while still refreshing the operator's display healthily. */
#define FUSED_TX_INTERVAL_IDLE_MS       5000   /* range-testing dense mode */

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
    #define PRE_LAUNCH_INTERVAL_SEC         5     // 5 s between raw GPS packets on the pad
                                                  // (dense for range testing; bench-quiet flight
                                                  //  prep variant: 30 s, see git history)
    #define POST_LAUNCH_DURATION_SEC        1     // 1 second in LAUNCH state before POST_LAUNCH
    #define POST_LAUNCH_RECOVERY_DURATION_SEC 600 // 10 minutes duration in post-launch state
    #define BATTERY_SAVE_INTERVAL_SEC       60    // 60 seconds between transmissions in battery save
    #define CALLSIGN_TRANSMIT_INTERVAL_SEC  300   // 5 minutes between callsign transmissions
    
    // Debug features disabled in production mode
    #define DEBUG_OUTPUT_ENABLED        0     // Disable debug output
    #define INCLUDE_CARRIAGE_RETURNS    0     // No \r\n in production (RF efficiency)
    
#endif

#endif // CONFIG_H
