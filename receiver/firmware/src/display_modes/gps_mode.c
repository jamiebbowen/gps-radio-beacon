/**
 ******************************************************************************
 * @file           : gps_mode.c
 * @brief          : GPS display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "display_modes/gps_mode.h"
#include "display.h"
#include "gps.h"

/**
 * @brief Display GPS data on screen
 * @param gps_data Pointer to current GPS data structure
 * @param last_good_gps Pointer to last known good GPS data
 * @param has_last_good Flag indicating if last good GPS data is available
 * @retval None
 */
void DisplayMode_GPS(GPS_Data *gps_data, GPS_Data *last_good_gps, uint8_t has_last_good)
{
  char buffer[32];
  GPS_Data *display_gps = gps_data;
  uint8_t using_last_good = 0;
  
  /* Use last known good GPS data if current fix is invalid but we have good data */
  if (!gps_data->fix && has_last_good) {
    display_gps = last_good_gps;
    using_last_good = 1;
  }
    
  /* Display fix status and satellite count */
  if (using_last_good) {
    sprintf(buffer, "Fix: L (was Y)");
  } else {
    sprintf(buffer, "Fix: %s Sat: %d", gps_data->fix ? "Y" : "N", gps_data->satellites);
  }
  Display_DrawTextRowCol(0, 0, buffer);
  
  /* Display latitude and longitude - use integer formatting to avoid float issues */
  int32_t lat_int = (int32_t)(display_gps->latitude * 100000);
  int32_t lat_dec = labs(lat_int % 100000);
  int32_t lat_whole = lat_int / 100000;
  sprintf(buffer, "Lat: %ld.%05ld", lat_whole, lat_dec);
  Display_DrawTextRowCol(1, 0, buffer);
  
  int32_t lon_int = (int32_t)(display_gps->longitude * 100000);
  int32_t lon_dec = labs(lon_int % 100000);
  int32_t lon_whole = lon_int / 100000;
  sprintf(buffer, "Lon: %ld.%05ld", lon_whole, lon_dec);
  Display_DrawTextRowCol(2, 0, buffer);
  
  /* Display altitude - use integer formatting to avoid float issues */
  int32_t alt_int = (int32_t)(display_gps->altitude * 10);
  int32_t alt_dec = labs(alt_int % 10);
  int32_t alt_whole = alt_int / 10;
  sprintf(buffer, "Alt: %ld.%01ldm", alt_whole, alt_dec);
  Display_DrawTextRowCol(3, 0, buffer);
  
  /* Display time */
  sprintf(buffer, "Time: %02d:%02d:%02d", display_gps->hour, display_gps->minute, display_gps->second);
  Display_DrawTextRowCol(4, 0, buffer);
  
  /* Display date */
  if (display_gps->day > 0 && display_gps->month > 0 && display_gps->year >= 2000) {
    sprintf(buffer, "Date: %02d/%02d/%02d", display_gps->day, display_gps->month, display_gps->year % 100);
    Display_DrawTextRowCol(5, 0, buffer);
  } else {
    /* Date not available (GPGGA doesn't contain date, only GPRMC does) */
    Display_DrawTextRowCol(5, 0, "Date: No Date");
  }
}
