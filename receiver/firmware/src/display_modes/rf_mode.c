/**
 ******************************************************************************
 * @file           : rf_mode.c
 * @brief          : RF display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "display_modes/rf_mode.h"
#include "display.h"
#include "gps.h"
#include "rf_receiver.h"
#include "rf_parser.h"

/**
 * @brief Display RF data screen
 * @param remote_gps_data Pointer to remote GPS data structure
 * @param has_valid_remote_gps Pointer to remote GPS validity flag
 * @param last_rf_packet_time Pointer to last RF packet time
 * @param rf_packet_count Pointer to RF packet counter
 * @param last_good_remote_gps Pointer to last good remote GPS data structure
 * @param has_last_good_remote Flag indicating if last good remote GPS data is valid
 * @retval None
 */
void DisplayMode_RF(GPS_Data *remote_gps_data, uint8_t *has_valid_remote_gps, 
                    uint32_t *last_rf_packet_time, uint32_t *rf_packet_count,
                    GPS_Data *last_good_remote_gps, uint8_t has_last_good_remote)
{
  char buffer[32];
  char line1[16] = {0};
  char line2[16] = {0};
  char ascii_buffer[64] = {0};
  uint32_t parse_attempts = 0;
  uint32_t parse_successes = 0;
  uint32_t parse_failures = 0;
  uint32_t null_packet = 0;
  uint32_t insufficient_fields = 0;
  uint32_t checksum_invalid = 0;
  uint32_t marker_not_found = 0;
  uint32_t callsign_too_short = 0;
  
  /* Get parser diagnostics */
  RF_Parser_GetDiagnostics(&parse_attempts, &parse_successes, &parse_failures);
  RF_Parser_GetDetailedFailures(&null_packet, &insufficient_fields, &checksum_invalid, 
                                &marker_not_found, &callsign_too_short);
  
  /* Get LoRa packet count and IRQ diagnostics */
  uint32_t lora_packets = RF_Receiver_GetLoRaPacketCount();
  uint32_t irq_checks = 0;
  uint16_t last_irq = 0;
  uint8_t device_mode = 0;
  uint8_t spi_test = 0xFF;
  uint8_t busy_state = 0;
  RF_Receiver_GetIRQDiagnostics(&irq_checks, &last_irq, &device_mode, &spi_test, &busy_state);
  
  if (*last_rf_packet_time > 0) {
    uint32_t time_since_last = HAL_GetTick() - *last_rf_packet_time;
    sprintf(buffer, "Last: %lu ms", time_since_last);
    Display_DrawTextRowCol(0, 0, buffer);
  } else {
    /* Mode: 0=Unused, 2=STDBY, 3=STDBY_XOSC, 4=FS, 5=RX, 6=TX */
    /* SPI: 0=Fail, 1=Pass, F=Not tested; BUSY: 0=Ready, 1=Stuck */
    /* Show IRQ status in hex to debug what IRQs we're getting */
    sprintf(buffer, "L:%lu M:%u IRQ:%03X", lora_packets, device_mode, last_irq);
    Display_DrawTextRowCol(0, 0, buffer);
  }
  
  /* Display parser diagnostics */
  sprintf(buffer, "P: %lu/%lu M%lu C%lu F%lu", parse_successes, parse_attempts, marker_not_found, checksum_invalid, insufficient_fields);
  Display_DrawTextRowCol(1, 0, buffer);
  
  /* Determine which GPS data to display */
  GPS_Data *display_gps = remote_gps_data;
  uint8_t using_last_good = 0;
  
  if (!(*has_valid_remote_gps) && has_last_good_remote) {
    display_gps = last_good_remote_gps;
    using_last_good = 1;
  }
  
  /* Display remote GPS data if valid or using fallback */
  if (*has_valid_remote_gps || using_last_good) {
    /* Display remote GPS coordinates - use integer formatting to avoid float issues */
    int32_t lat_int = (int32_t)(display_gps->latitude * 100000);
    int32_t lat_dec = labs(lat_int % 100000);
    int32_t lat_whole = lat_int / 100000;
    if (using_last_good) {
      sprintf(buffer, "Lat: %ld.%05ld (L)", lat_whole, lat_dec);
    } else {
      sprintf(buffer, "Lat: %ld.%05ld", lat_whole, lat_dec);
    }
    Display_DrawTextRowCol(2, 0, buffer);
    
    int32_t lon_int = (int32_t)(display_gps->longitude * 100000);
    int32_t lon_dec = labs(lon_int % 100000);
    int32_t lon_whole = lon_int / 100000;
    if (using_last_good) {
      sprintf(buffer, "Lon: %ld.%05ld (L)", lon_whole, lon_dec);
    } else {
      sprintf(buffer, "Lon: %ld.%05ld", lon_whole, lon_dec);
    }
    Display_DrawTextRowCol(3, 0, buffer);
    
    /* Display remote GPS altitude - use integer formatting to avoid float issues */
    int32_t alt_int = (int32_t)(display_gps->altitude * 10);
    int32_t alt_dec = labs(alt_int % 10);
    int32_t alt_whole = alt_int / 10;
    if (using_last_good) {
      sprintf(buffer, "Alt: %ld.%01ldm (LAST)", alt_whole, alt_dec);
    } else {
      sprintf(buffer, "Alt: %ld.%01ldm", alt_whole, alt_dec);
    }
    Display_DrawTextRowCol(4, 0, buffer);
    
    /* Display remote GPS fix status and satellite count */
    if (using_last_good) {
      sprintf(buffer, "Fix: L (was Y) Sat: %d", display_gps->satellites);
    } else {
      sprintf(buffer, "Fix: %s Sat: %d", display_gps->fix ? "Y" : "N", display_gps->satellites);
    }
    Display_DrawTextRowCol(5, 0, buffer);
  } else if (*last_rf_packet_time > 0) {
    Display_DrawTextRowCol(2, 0, "No valid GPS data");
    Display_DrawTextRowCol(3, 0, "in received packet");
  }
  
  /* Display last packet data in ASCII */
  if (RF_Receiver_GetLastPacketASCII(ascii_buffer, sizeof(ascii_buffer)) > 0) {   
    /* Split the ASCII buffer into two lines for display */
    if (strlen(ascii_buffer) > 0) {
      strncpy(line1, ascii_buffer, 15);
      Display_DrawTextRowCol(6, 0, line1);
      
      if (strlen(ascii_buffer) > 15) {
        strncpy(line2, &ascii_buffer[15], 15);
        Display_DrawTextRowCol(7, 0, line2);
      }
    }
  } else if (*last_rf_packet_time == 0) {
    Display_DrawTextRowCol(6, 0, "No RF data received");
  }
  
  /* Display raw packet data for debugging */
  char raw_buffer[64];
  if (RF_Parser_GetLastRawPacket(raw_buffer, sizeof(raw_buffer)) > 0) {    
    /* Split the raw buffer into two lines for display */
    if (strlen(raw_buffer) > 0) {
      memset(line1, 0, sizeof(line1));
      memset(line2, 0, sizeof(line2));
      
      strncpy(line1, raw_buffer, 15);
      Display_DrawTextRowCol(8, 0, line1);
      
      if (strlen(raw_buffer) > 15) {
        strncpy(line2, &raw_buffer[15], 15);
        Display_DrawTextRowCol(9, 0, line2);
      }
    }
  }
}
