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

/* 0 = LoRa radio init failed (boot + periodic retries in main). Display
 * modes use this to show "RF DEAD" instead of the misleading "No signal". */
extern uint8_t rf_initialized;
extern uint8_t has_valid_local_gps;

/* Global Compass data */
extern Compass_Data compass_data;

/* Global navigation data */
extern float distance_to_tx;
extern float direction_to_tx;
extern float relative_direction;

#ifdef __cplusplus
}
#endif

#endif /* __GLOBALS_H */
