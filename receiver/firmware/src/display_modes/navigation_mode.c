/**
 ******************************************************************************
 * @file           : navigation_mode.c
 * @brief          : Navigation display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>  /* For abs and labs functions */
#include <math.h>
#include "stm32f4xx_hal.h"
#include "display_modes/navigation_mode.h"
#include "display.h"
#include "gps.h"
#include "math_utils.h"
#include "compass.h"

extern Compass_Data compass_data;

/* Expected beacon transmission period (must match transmitter). */
#define NAV_BEACON_INTERVAL_MS   5000U
/* After this long without a packet, start counting it as missed.       *
 * Set to 1.5× the interval so a single late packet is not penalised.  */
#define NAV_PACKET_STALE_MS      7500U
/* Width of the rolling packet-loss percentage window. */
#define NAV_LOSS_WINDOW_MS      60000U

/**
 * @brief Display navigation data screen
 * @param has_valid_local_gps Flag indicating if local GPS data is valid
 * @param has_valid_remote_gps Flag indicating if remote GPS data is valid
 * @param local_gps_data Local GPS data structure
 * @param remote_gps_data Remote GPS data structure
 * @param compass_heading Current compass heading
 * @param last_rf_packet_time Time of last RF packet received
 * @param rf_packet_count Count of RF packets received
 * @param rf_missed_packet_count Cumulative count of missed packets
 * @retval None
 */
void DisplayMode_Navigation(uint8_t has_valid_local_gps, uint8_t has_valid_remote_gps,
                           GPS_Data *local_gps_data, GPS_Data *remote_gps_data,
                           float compass_heading, uint32_t last_rf_packet_time, uint32_t rf_packet_count,
                           uint32_t rf_missed_packet_count)
{
  char buffer[32];
  float distance_to_tx = -1.0f;
  float direction_to_tx = -1.0f;
  float relative_direction = -1.0f;
  
  /* Static variables to store last known good values */
  static float last_good_distance = -1.0f;
  static float last_good_direction = -1.0f;
  static float last_good_relative = -1.0f;
  static uint32_t last_good_update_time = 0;
  
  /* Calculate distance and direction if we have valid local GPS and remote position (live or SD-loaded) */
  if (has_valid_local_gps && (last_rf_packet_time > 0 || has_valid_remote_gps)) {
    /* Validate coordinates are within reasonable ranges and non-zero */
    if (local_gps_data->latitude != 0.0f && local_gps_data->longitude != 0.0f && 
        remote_gps_data->latitude != 0.0f && remote_gps_data->longitude != 0.0f &&
        local_gps_data->latitude >= -90.0f && local_gps_data->latitude <= 90.0f &&
        local_gps_data->longitude >= -180.0f && local_gps_data->longitude <= 180.0f &&
        remote_gps_data->latitude >= -90.0f && remote_gps_data->latitude <= 90.0f &&
        remote_gps_data->longitude >= -180.0f && remote_gps_data->longitude <= 180.0f) {
      /* Calculate distance between local and remote GPS coordinates */
      /* Always use the last known remote GPS position, even if it's stale */
      distance_to_tx = calculate_distance(local_gps_data->latitude, local_gps_data->longitude,
                                        remote_gps_data->latitude, remote_gps_data->longitude) / 1000.0f; /* Convert to km */
      
      /* Calculate bearing from local to remote GPS coordinates */
      direction_to_tx = calculate_bearing(local_gps_data->latitude, local_gps_data->longitude,
                                        remote_gps_data->latitude, remote_gps_data->longitude);
      
      /* Calculate relative direction based on current compass heading */
      relative_direction = direction_to_tx - compass_heading;
      
      /* Normalize relative direction to 0-359.9 degrees */
      while (relative_direction < 0) relative_direction += 360.0f;
      while (relative_direction >= 360.0f) relative_direction -= 360.0f;
      
      /* Always cache: reaching here means GPS coords were valid and both
       * calculate_distance / calculate_bearing returned meaningful values.
       * Note: direction_to_tx can legitimately be 0.0 (beacon due North)
       * so a >= 0 guard would incorrectly skip that case. */
      last_good_distance    = distance_to_tx;
      last_good_direction   = direction_to_tx;
      last_good_relative    = relative_direction;
      last_good_update_time = HAL_GetTick();
    } else {
      /* Set invalid values if coordinates are zero */
      distance_to_tx = -1.0f;
      direction_to_tx = -1.0f;
      relative_direction = -1.0f;
    }
  }
  
  /* If current values are invalid but we have last known good values, use them */
  if (distance_to_tx < 0.0f && last_good_distance >= 0.0f) {
    distance_to_tx = last_good_distance;
  }
  
  if (relative_direction < 0.0f && last_good_relative >= 0.0f) {
    relative_direction = last_good_relative;
  }
  
  /* 60-second rolling packet loss window */
  static uint32_t win_start_time = 0;
  static uint32_t win_start_received = 0;
  static uint8_t  loss_pct = 0;         /* loss% for the last completed window */
  static uint8_t  loss_valid = 0;       /* 1 once first window completes */

  if (last_rf_packet_time > 0) {
    uint32_t now = HAL_GetTick();
    if (win_start_time == 0) {
      win_start_time     = now;
      win_start_received = rf_packet_count;
    }
    uint32_t win_ms = now - win_start_time;
    if (win_ms >= NAV_LOSS_WINDOW_MS) {
      uint32_t expected = win_ms / NAV_BEACON_INTERVAL_MS;
      uint32_t received = rf_packet_count - win_start_received;
      if (expected > 0) {
        loss_pct = (expected > received)
                   ? (uint8_t)(((expected - received) * 100) / expected)
                   : 0;
      }
      loss_valid         = 1;
      win_start_time     = now;
      win_start_received = rf_packet_count;
    }
  }

  /* Display navigation data - optimized for 128x64 screen */
  /* Show navigation data if we have remote position (live RF or SD-loaded beacon) */
  if (last_rf_packet_time > 0 || has_valid_remote_gps) {

    /* Row 0: Distance */
    if (distance_to_tx >= 0.0f) {
      if (distance_to_tx < 1.0f) {
        int meters = (int)(distance_to_tx * 1000.0f);
        if (meters <= 0) meters = 1;
        sprintf(buffer, "Dist: %dm", meters);
      } else {
        int km_whole = (int)distance_to_tx;
        int km_frac  = (int)((distance_to_tx - km_whole) * 100.0f + 0.5f);
        sprintf(buffer, "Dist: %d.%02dkm", km_whole, km_frac);
      }
    } else {
      sprintf(buffer, "Dist: --");
    }
    Display_DrawTextRowCol(0, 0, buffer);

    /* Row 1: Combined local + beacon fix status */
    const char *b_fix = "B:--";
    if (last_rf_packet_time > 0) {
      if      (remote_gps_data->fix == 0) b_fix = "B:NoFix";
      else if (remote_gps_data->fix == 1) b_fix = "B:2D";
      else                                b_fix = "B:3D";
    }
    sprintf(buffer, "%s %s", has_valid_local_gps ? "L:Fix" : "L:--", b_fix);
    Display_DrawTextRowCol(1, 0, buffer);

    /* Row 2: Beacon satellite count + stale flag */
    if (last_rf_packet_time > 0) {
      if (!has_valid_remote_gps) {
        sprintf(buffer, "Sats:%d STALE", remote_gps_data->satellites);
      } else {
        sprintf(buffer, "Sats: %d", remote_gps_data->satellites);
      }
    } else {
      sprintf(buffer, "Sats: SD");
    }
    Display_DrawTextRowCol(2, 0, buffer);

    /* Row 3: Time since last packet */
    uint32_t time_since_last_packet = (last_rf_packet_time > 0)
                                      ? (HAL_GetTick() - last_rf_packet_time) : 0;
    if (last_rf_packet_time > 0) {
      sprintf(buffer, "Last: %lus", time_since_last_packet / 1000);
    } else {
      sprintf(buffer, "Last: SD");
    }
    Display_DrawTextRowCol(3, 0, buffer);

    /* Row 4: Packet count (received / expected) */
    if (last_rf_packet_time > 0 && rf_packet_count > 0) {
      uint32_t pending = 0;
      if (time_since_last_packet >= NAV_PACKET_STALE_MS) {
        pending = (time_since_last_packet + NAV_BEACON_INTERVAL_MS / 2) / NAV_BEACON_INTERVAL_MS - 1;
      }
      uint32_t total_expected = rf_packet_count + rf_missed_packet_count + pending;
      sprintf(buffer, "Pkts:%lu/%lu", rf_packet_count, total_expected);
    } else {
      sprintf(buffer, "Pkts: 0");
    }
    Display_DrawTextRowCol(4, 0, buffer);

    /* Row 5: 60-second rolling packet loss percentage */
    if (last_rf_packet_time > 0) {
      if (loss_valid) {
        sprintf(buffer, "Loss: %d%%", loss_pct);
      } else {
        /* Running estimate for current incomplete window */
        uint32_t win_ms = HAL_GetTick() - win_start_time;
        uint32_t expected = win_ms / NAV_BEACON_INTERVAL_MS;
        uint32_t received = rf_packet_count - win_start_received;
        uint8_t curr_loss = (expected > 0 && expected > received)
                            ? (uint8_t)(((expected - received) * 100) / expected) : 0;
        sprintf(buffer, "Loss: %d%%~", curr_loss);
      }
    } else {
      sprintf(buffer, "Loss: --");
    }
    Display_DrawTextRowCol(5, 0, buffer);

    /* Right side: direction indicator.
     * Only draw arrow when the compass heading is valid (mag_cal >= 1).
     * Draw a plain circle with "CAL" label when magnetometer is uncalibrated
     * so the user knows to wave the device in a figure-8 pattern. */
    if (relative_direction >= 0.0f && relative_direction < 360.0f &&
        compass_data.heading_valid) {
      Display_DrawDirectionIndicator(96, 32, relative_direction);
    } else if (!compass_data.heading_valid) {
      Display_DrawCircle(96, 32, 15, 1);
      Display_DrawText(89, 28, "CAL");
    } else {
      Display_DrawCircle(96, 32, 15, 1);
    }

  } else {
    if (!has_valid_local_gps) {
      Display_DrawTextRowCol(4, 0, "No GPS Fix");
    } else {
      Display_DrawTextRowCol(5, 0, "No RF Data");
    }
  }
}
