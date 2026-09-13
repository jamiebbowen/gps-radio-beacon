#ifndef FLIGHT_CADENCE_H
#define FLIGHT_CADENCE_H

#include <stdint.h>

/* Beacon cadence policy, extracted from firmware.ino so the pacing rules
 * the recovery depends on (pad rate, flight rate, battery-save, FCC
 * callsign ID interval, phase-exit durations) are host-testable. All
 * constants live in config.h; these functions are the ONLY place the
 * state -> timing mapping is defined. */
typedef enum {
    BEACON_STATE_PRE_LAUNCH = 0,   /* pad: sparse raw GPS packets          */
    BEACON_STATE_LAUNCH,           /* flight: continuous raw + fast fused  */
    BEACON_STATE_POST_LAUNCH,      /* recovery window: paced raw + fused   */
    BEACON_STATE_BATTERY_SAVE      /* landed / late: sparse everything     */
} beacon_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Seconds between raw GPS beacons in `state` (LAUNCH: 0 = continuous). */
uint32_t flight_cadence_beacon_interval_s(beacon_state_t state);

/* Milliseconds between fused packets in `state`. Dense in flight phases;
 * sparse at pad / battery-save to spare the PA (thermal) and to keep the
 * beacon's own GPS out of its desense shadow. */
uint32_t flight_cadence_fused_interval_ms(beacon_state_t state);

/* Seconds between callsign IDs (FCC 97.119 wants <= 600 s). */
uint32_t flight_cadence_callsign_interval_s(void);

/* Phase exits: raw seconds-in-phase compared against config.h durations. */
int flight_cadence_should_leave_launch(uint32_t time_since_launch_s);
int flight_cadence_should_leave_post_launch(uint32_t time_in_post_launch_s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FLIGHT_CADENCE_H */
