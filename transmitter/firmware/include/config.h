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

/* Minimum seconds between heartbeat packets when a beacon TX was requested
 * but no transmittable GPS fix exists (flight / battery-save states). On the
 * pad the heartbeat IS the beacon interval instead - see the TURN_ON /
 * PRELAUNCH cadence constants below. */
#define HEARTBEAT_INTERVAL_SEC 5

/* ---------------------------------------------------------------------------
 * Beep-before-silence pad policy (turn-on grace + quiet pad)
 * -------------------------------------------------------------------------*/

/* Turn-on grace: the operator is standing at the rail asking "did it come
 * up? did the GPS start? is it on the right channel?" Heartbeat every 5 s
 * for the first minute answers all three while packets are still cheap;
 * then the pad goes quiet. */
#define TURN_ON_DURATION_SEC            60
#define TURN_ON_HEARTBEAT_INTERVAL_SEC  5

/* Quiet pad: one heartbeat a minute proves liveness without PA noise in the
 * GPS front end's face (field logs: pad TX bursts cost the beacon GPS three
 * watchdog rounds before first fix). On-demand freshness between heartbeats
 * is the two-way channel's job (PING). */
#define PRELAUNCH_HEARTBEAT_INTERVAL_SEC 60

/* Full GPS position on the pad: once on the first good fix (the logged pad
 * coordinate everyone argues from later), then a sparse audit copy. */
#define PRELAUNCH_GPS_AUDIT_INTERVAL_SEC 600

/* ---------------------------------------------------------------------------
 * Flight-event + anomaly detection (flight_events.cpp). Pure kinematics:
 * fused v_d / gyro magnitude / sensor health, fed per loop pass. Thresholds
 * pinned by transmitter/tests/test_flight_events.cpp.
 * -------------------------------------------------------------------------*/

/* Apogee: sustained descent after the launch edge. The 2 s floor rejects
 * detection glitches off the launch transient; 2 m/s for 0.5 s rejects the
 * v_d noise band of a healthy anchor without delaying the real call. */
#define APOGEE_MIN_T_SINCE_LAUNCH_S   2U
#define APOGEE_VD_MIN_MS              2.0f
#define APOGEE_HOLD_MS                500U

/* Drogue: descent rate stabilizes inside the drogue band. Dead zone
 * 8..10 m/s vs the main band below is deliberate - an ambiguous rate
 * declares nothing until it resolves. */
#define DROGUE_BAND_MIN_MS            10.0f
#define DROGUE_BAND_MAX_MS            30.0f
#define DROGUE_HOLD_MS                1000U

/* Main: descent rate steps down to <= max(MAIN_BAND_MAX, 50% of the drogue
 * baseline rate). MAIN_MIN_BASELINE keeps a lazy 9 m/s all-the-way-down hop
 * from inventing a deployment. */
#define MAIN_BAND_MAX_MS              8.0f
#define MAIN_STEP_RATIO               0.5f
#define MAIN_MIN_BASELINE_MS          10.0f
#define MAIN_HOLD_MS                  1000U

/* Anomalies (level conditions with holds; entered as FLIGHT_EVENT codes,
 * re-announced while active, cleared after ANOM_CLEAR_MS of all-well). */
#define ANOM_BALLISTIC_VD_MS          35.0f  /* past the drogue band, post-grace */
#define ANOM_BALLISTIC_GRACE_S        5U     /* drogue may need seconds to open  */
#define ANOM_BALLISTIC_HOLD_MS        1000U
#define ANOM_TUMBLE_RADS              17.5f  /* ~1000 deg/s: not a flying rocket */
#define ANOM_TUMBLE_HOLD_MS           300U
#define ANOM_SENSOR_HOLD_MS           1000U  /* BNO085 degraded this long in air */
#define ANOM_GPS_OUTAGE_MS            15000UL /* = NAV_RESCUE_NOFIX_MS horizon  */
#define ANOM_CLEAR_MS                 5000U  /* all-clear hysteresis             */

/* While an anomaly is active the beacon bleeds everything: the inertial
 * trace tightens to ANOM_BEACON_INTERVAL_S (from the 2 s recovery pacing)
 * and the anomaly event re-announces at ANOM_EVENT_REPEAT_MS. 1 s spacing
 * with 1.12 s packet airtimes keys the PA near 100% until the anomaly
 * clears - accepted deliberately: ballistic anomalies last seconds, and
 * sensor-loss anomalies are exactly the flights where the trace is the
 * only narrative. The POST_LAUNCH recovery window / landing latch bound
 * the worst-case duration. */
#define ANOM_BEACON_INTERVAL_S        1U
#define ANOM_EVENT_REPEAT_MS          10000UL

/* Certified events are one-shot edges - sent FLIGHT_EVENT_REPEATS times
 * this far apart, so a single RF-null moment can't erase the record. */
#define FLIGHT_EVENT_REPEATS          3U
#define FLIGHT_EVENT_SPACING_MS       1200U

/* Maxima recap cadence while airborne (plus one copy at landing). The
 * recurring stream makes EVERY heard packet a cumulative flight record -
 * the "how high did it go" answer survives a shred at apogee, when no
 * APOGEE event would ever fire. */
#define MAXIMA_TX_INTERVAL_MS         5000U

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
 * packet's air time. SF10/BW62.5k/CR4-8/8-sym-preamble with LDRO: the V3
 * 19-byte packet is 60.25 symbols = ~0.99 s on-air (V3 dropped age_ds to
 * get back under the step the V2 rocket_id byte had crossed: 20 B cost
 * 1.12 s). The 1500 ms interval keeps the PA at ~66% duty with a ~510 ms
 * radio-quiet gap per cycle. test_flight_cadence pins this against the
 * computed airtime; drift is a red test. */
#define FUSED_TX_INTERVAL_MS            1500

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
