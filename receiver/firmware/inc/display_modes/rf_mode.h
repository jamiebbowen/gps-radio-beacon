/**
 ******************************************************************************
 * @file           : rf_mode.h
 * @brief          : RF display mode functions
 ******************************************************************************
 */

#ifndef RF_MODE_H
#define RF_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gps.h"

/**
 * @brief Display RF data on screen
 * @param remote_gps_data Pointer to current remote GPS data structure
 * @param has_valid_remote_gps Pointer to flag indicating valid remote GPS
 * @param last_rf_packet_time Pointer to last RF packet time
 * @param rf_packet_count Pointer to RF packet count
 * @param last_good_remote_gps Pointer to last known good remote GPS data
 * @param has_last_good_remote Flag indicating if last good remote GPS data is available
 * @retval None
 */
void DisplayMode_RF(GPS_Data *remote_gps_data, uint8_t *has_valid_remote_gps, 
                   uint32_t *last_rf_packet_time, uint32_t *rf_packet_count,
                   GPS_Data *last_good_remote_gps, uint8_t has_last_good_remote);

#ifdef __cplusplus
}
#endif

#endif /* RF_MODE_H */
