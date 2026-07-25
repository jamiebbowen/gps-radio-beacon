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

#endif // BEACON_H
