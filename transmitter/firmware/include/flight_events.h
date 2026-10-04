#ifndef FLIGHT_EVENTS_H
#define FLIGHT_EVENTS_H

/* Airborne life-cycle events (apogee / drogue / main) and the flight
 * anomaly level-set, derived purely from observables the beacon already
 * has: fused NED velocity (v_d positive = descending), gyro magnitude,
 * IMU health, GPS fix age and time-since-launch.
 *
 * Fed once per loop pass while airborne. Life-cycle detections are
 * one-shot edges returned from flight_events_feed() (the caller queues
 * them on the radio with redundancy). Anomalies are LEVEL conditions with
 * per-cause holds and an all-clear hysteresis; the entry edge is returned
 * as an event and flight_events_anomaly_* drives the cadence override and
 * the in-anomaly re-announce while it stays active.
 *
 * Pure logic, no Arduino deps: thresholds live in config.h and the whole
 * state machine is pinned by transmitter/tests/test_flight_events.cpp.
 * Wire codes live in packet_format.h (FLIGHT_EVENT_*). */

#include <stdint.h>
#include "packet_format.h"   /* FLIGHT_EVENT_* wire codes */
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  airborne;         /* launch confirmed && landing not latched  */
    float    v_d_ms;           /* fused descent rate m/s (positive = down) */
    float    alt_m;            /* fused altitude m MSL                     */
    uint8_t  nav_valid;        /* 0 = EKF un-anchored: v_d/alt are fiction */
    float    gyro_mag_rads;    /* |gyro| rad/s                             */
    uint8_t  imu_degraded;     /* BNO085 dead / in retry (level)           */
    uint32_t gps_fix_age_ms;   /* ms since last accepted GPS fix           */
    uint32_t t_since_launch_s; /* seconds since launch confirmation        */
} flight_events_input_t;

void flight_events_init(void);

/* Advance the detector. Returns a FLIGHT_EVENT_* wire code when an edge
 * fires this pass (0 = nothing), and writes the event value via
 * value_out: altitude (whole meters) for APOGEE, descent rate (cm/s) for
 * the deployment/anomaly codes, 0 otherwise. At most one event per pass;
 * simultaneous edges pend and drain in priority order (anomalies first). */
uint8_t flight_events_feed(const flight_events_input_t *in,
                           uint32_t now_ms, int16_t *value_out);

/* Anomaly level-set. While active, the main loop tightens the inertial
 * trace cadence (ANOM_BEACON_INTERVAL_S) and re-announces the current
 * highest-priority code every ANOM_EVENT_REPEAT_MS. Clears only after
 * ANOM_CLEAR_MS with no condition held (or at landing: airborne 0 makes
 * every condition false). */
uint8_t flight_events_anomaly_active(void);
uint8_t flight_events_anomaly_code(void);   /* wire code, 0 when inactive */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FLIGHT_EVENTS_H */
