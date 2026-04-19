/**
 * @file globals.h
 * @brief Global variables for the GPS Radio Beacon Receiver
 */

#ifndef __GLOBALS_H
#define __GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gps.h"
#include "compass.h"
#include <stdint.h>

/* Global GPS data */
extern GPS_Data local_gps_data;
extern GPS_Data remote_gps_data;
extern uint8_t has_valid_local_gps;
extern uint8_t has_valid_remote_gps;

/* Global Compass data */
extern Compass_Data compass_data;

/* Global RF data */
extern uint32_t last_ping_time;

/* Global navigation data */
extern float distance_to_tx;
extern float direction_to_tx;
extern float relative_direction;

#ifdef __cplusplus
}
#endif

#endif /* __GLOBALS_H */
