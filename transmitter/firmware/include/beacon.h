#ifndef BEACON_H
#define BEACON_H

#include <stdint.h>
#include <stdbool.h>
#include "gps.h"

// Beacon transmission functions
uint8_t beacon_transmit_gps_data(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast_flag);
uint8_t beacon_transmit_gps_data_binary(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast_flag);
void beacon_transmit_callsign(uint8_t transmit_fast);

#endif // BEACON_H
