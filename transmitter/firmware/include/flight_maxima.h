#ifndef FLIGHT_MAXIMA_H
#define FLIGHT_MAXIMA_H

/* Running flight maxima ("black box on the air"): while airborne, tracks
 * peak altitude/speed/accel/rotation so the periodic MAXIMA packets and
 * the final landing-latch copy always carry the flight's envelope UP TO
 * THE CURRENT SECOND - a beacon destroyed mid-flight still leaves its
 * story on the ground log (L0016's unanswered "how high did it go").
 *
 * Pure logic, host-tested by test_flight_maxima.cpp. Feeding is cheap;
 * call once per loop pass alongside the flight_events feed. */

#include <stdint.h>
#include "packet_format.h"   /* MaximaPacket_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  airborne;       /* launch confirmed && landing not latched    */
    uint8_t  nav_valid;      /* alt/speed inputs trustworthy               */
    float    alt_m;          /* fused altitude m MSL                       */
    float    v_n_ms, v_e_ms, v_d_ms;  /* fused NED velocity m/s            */
    float    accel_ms2;      /* |linear accel| magnitude, m/s^2            */
    float    gyro_mag_rads;  /* |gyro| rad/s                               */
    uint32_t uptime_s;
} flight_maxima_input_t;

void flight_maxima_init(void);
void flight_maxima_feed(const flight_maxima_input_t *in);

/* Fill the payload fields of a MAXIMA wire packet (packet_type/rocket_id
 * left to the caller). Counts/positions clamp to the wire ranges. */
void flight_maxima_get(MaximaPacket_t *out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FLIGHT_MAXIMA_H */
