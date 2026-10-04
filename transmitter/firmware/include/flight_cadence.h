#ifndef FLIGHT_CADENCE_H
#define FLIGHT_CADENCE_H

#include <stdint.h>

/* Beacon cadence policy, extracted from firmware.ino so the pacing rules
 * the recovery depends on (pad rate, flight rate, battery-save, FCC
 * callsign ID interval, phase-exit durations) are host-testable. All
 * constants live in config.h; these functions are the ONLY place the
 * state -> timing mapping is defined. */
typedef enum {
    BEACON_STATE_TURN_ON = 0,      /* boot grace: heartbeat 5 s x 60 s     */
    BEACON_STATE_PRE_LAUNCH,       /* quiet pad: heartbeat 60 s, GPS rare  */
    BEACON_STATE_LAUNCH,           /* flight: continuous raw + fast fused  */
    BEACON_STATE_POST_LAUNCH,      /* recovery window: paced raw + fused   */
    BEACON_STATE_BATTERY_SAVE      /* landed / late: sparse everything     */
} beacon_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Seconds between beacon ticks in `state` (LAUNCH: 0 = continuous). In the
 * pad states a tick is a heartbeat; in BATTERY_SAVE it is GPS-or-heartbeat;
 * in flight it carries the inertial trace. */
uint32_t flight_cadence_beacon_interval_s(beacon_state_t state);

/* Milliseconds between fused packets in `state`. Dense in flight phases;
 * 0 = the fused stream is OFF (pad states: no operator value in a
 * stationary fused packet, and the PA quiet protects the GPS front end). */
uint32_t flight_cadence_fused_interval_ms(beacon_state_t state);

/* Seconds between heartbeat packets in `state` (also the floor for the
 * no-fix heartbeat rate limiter in flight/ground states). */
uint32_t flight_cadence_heartbeat_interval_s(beacon_state_t state);

/* Seconds between full GPS audit packets in pad states (0 = no audit
 * stream; the first-good-fix one-shot is handled by the loop). */
uint32_t flight_cadence_gps_audit_interval_s(beacon_state_t state);

/* Seconds between callsign IDs (FCC 97.119 wants <= 600 s). */
uint32_t flight_cadence_callsign_interval_s(void);

/* Phase exits: raw seconds-in-phase compared against config.h durations. */
int flight_cadence_should_leave_turn_on(uint32_t uptime_s);
int flight_cadence_should_leave_launch(uint32_t time_since_launch_s);
int flight_cadence_should_leave_post_launch(uint32_t time_in_post_launch_s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FLIGHT_CADENCE_H */
