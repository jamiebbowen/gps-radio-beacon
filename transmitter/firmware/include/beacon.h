#ifndef BEACON_H
#define BEACON_H

#include <stdint.h>
#include <stdbool.h>
#include "gps.h"

// Beacon transmission functions
uint8_t beacon_transmit_gps_data(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast_flag);
uint8_t beacon_transmit_gps_data_binary(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast_flag);
void beacon_transmit_callsign(uint8_t transmit_fast);

/* Transmit a fused (EKF) position+velocity packet.  Returns 1 on success,
 * 0 if the nav layer isn't anchored yet or the radio TX failed. */
uint8_t beacon_transmit_fused_data(uint32_t system_time_seconds, uint8_t transmit_fast_flag);

/* Transmit a heartbeat packet, rate-limited to one per `min_interval_s`
 * (state cadence comes from flight_cadence_heartbeat_interval_s). On the
 * pad states the heartbeat IS the beacon tick; in flight/ground states call
 * it when a beacon TX was requested but the GPS data was rejected.
 * Returns 1 if a heartbeat was transmitted, 0 if rate-limited or TX failed. */
uint8_t beacon_transmit_heartbeat(const GPSCoordinates_t* coords, uint32_t system_time_seconds,
                                  uint32_t min_interval_s, uint8_t transmit_fast_flag);
uint8_t beacon_transmit_launch_t0(uint32_t system_time_seconds, uint8_t transmit_fast_flag);
uint8_t beacon_transmit_imu_trace(uint8_t transmit_fast_flag);
void beacon_poll_commands(uint8_t in_flight);

/* Certified flight events (PACKET_TYPE_FLIGHT_EVENT). Queue an edge with
 * redundancy; the loop calls beacon_service_flight_events() every pass and
 * copies go out FLIGHT_EVENT_SPACING_MS apart until `repeats` are sent.
 * beacon_queue_flight_event returns 0 if the queue was full (counted via
 * beacon_flight_event_dropped()). */
void    beacon_queue_flight_event(uint8_t code, int16_t value, uint8_t repeats);
uint8_t beacon_service_flight_events(uint32_t now_ms, uint32_t now_s, uint8_t transmit_fast_flag);
uint32_t beacon_flight_event_dropped(void);
/* Single immediate TX; used by the queue service. Blocks ~0.7 s (8 B at
 * SF10/BW62.5k) like every packet here. */
uint8_t beacon_transmit_flight_event(uint8_t code, int16_t value,
                                     uint32_t system_time_seconds, uint8_t transmit_fast_flag);

/* Boot identity one-shot: firmware hash (+ dirty mark) so the ground log
 * names exactly which beacon firmware flew. Call once from setup. */
uint8_t beacon_transmit_hello(uint32_t system_time_seconds, uint8_t transmit_fast_flag);

/* Running maxima recap: call at MAXIMA_TX_INTERVAL_MS while airborne and
 * once at the landing latch. Values come from flight_maxima. */
uint8_t beacon_transmit_maxima(uint32_t system_time_seconds, uint8_t transmit_fast_flag);

#endif // BEACON_H
