/**
 ******************************************************************************
 * @file           : gps_mode.h
 * @brief          : GPS display mode functions
 ******************************************************************************
 */

#ifndef GPS_MODE_H
#define GPS_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gps.h"

/**
 * @brief Display GPS data on screen
 * @param gps_data Pointer to current GPS data structure
 * @param last_good_gps Pointer to last known good GPS data
 * @param has_last_good Flag indicating if last good GPS data is available
 * @retval None
 */
void DisplayMode_GPS(GPS_Data *gps_data, GPS_Data *last_good_gps, uint8_t has_last_good);

#ifdef __cplusplus
}
#endif

#endif /* GPS_MODE_H */
