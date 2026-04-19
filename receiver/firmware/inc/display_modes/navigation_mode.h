/**
 ******************************************************************************
 * @file           : navigation_mode.h
 * @brief          : Navigation display mode functions
 ******************************************************************************
 */

#ifndef NAVIGATION_MODE_H
#define NAVIGATION_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gps.h"

/**
 * @brief Display navigation data on screen
 * @param has_valid_local_gps Flag indicating valid local GPS
 * @param has_valid_remote_gps Flag indicating valid remote GPS
 * @param local_gps_data Pointer to local GPS data structure
 * @param remote_gps_data Pointer to remote GPS data structure
 * @param compass_heading Current compass heading
 * @param last_rf_packet_time Time of last RF packet received
 * @param rf_packet_count Count of RF packets received
 * @param first_rf_packet_time Time of first RF packet received
 * @retval None
 */
void DisplayMode_Navigation(uint8_t has_valid_local_gps, uint8_t has_valid_remote_gps,
                           GPS_Data *local_gps_data, GPS_Data *remote_gps_data,
                           float compass_heading, uint32_t last_rf_packet_time, uint32_t rf_packet_count,
                           uint32_t first_rf_packet_time);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_MODE_H */
