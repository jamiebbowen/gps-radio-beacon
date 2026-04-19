/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "gps.h"
#include "compass.h"
#include "globals.h"
#include "display.h"
#include "rf_receiver.h"
#include "display_modes.h"
#include "math_utils.h"
#include "button.h"
#include "sd_card.h"
#include "sd_diskio.h"
#include "display_modes/sd_card_mode.h"

/* Constants */
#define DEG_TO_RAD 0.017453292519943295f  /* PI/180 */
#define RAD_TO_DEG 57.29577951308232f    /* 180/PI */
#define EARTH_RADIUS_KM 6371.0f         /* Earth radius in km */
#define BUTTON_DEBOUNCE_MS 200

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Display mode enum */
typedef enum {
  DISPLAY_MODE_NAVIGATION = 0,
  DISPLAY_MODE_GPS = 1,
  DISPLAY_MODE_RF = 2,
  DISPLAY_MODE_COMPASS_VISUAL = 3,
  DISPLAY_MODE_COMPASS_HEADING = 4,
  DISPLAY_MODE_SD_CARD = 5
} DisplayMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1; /* Global SPI handle for SD card */

/* Private variables */
static float compass_heading = 0.0f;

/* Global variables defined in globals.h */
GPS_Data local_gps_data = {0};        /* Initialize to zero to prevent garbage in date fields */
GPS_Data remote_gps_data = {0};       /* Initialize to zero */
GPS_Data last_good_local_gps = {0};   /* Store last known good local GPS data */
GPS_Data last_good_remote_gps = {0};  /* Store last known good remote GPS data */
Compass_Data compass_data;

/* Display mode control */
static DisplayMode_t current_display_mode = DISPLAY_MODE_NAVIGATION;
static uint32_t mode_change_time = 0;

/* Flags and counters */
uint8_t has_valid_local_gps = 0;
uint8_t has_valid_remote_gps = 0;
uint8_t has_last_good_local_gps = 0;  /* Flag for last known good local GPS */
uint8_t has_last_good_remote_gps = 0; /* Flag for last known good remote GPS */
uint8_t rf_initialized = 0;
uint32_t last_rf_packet_time = 0;
uint32_t rf_packet_count = 0;
uint32_t first_rf_packet_time = 0;  /* Time when first packet was received */
uint32_t rf_missed_packet_count = 0;  /* Real-time count of missed packets */
uint32_t rf_next_expected_deadline = 0;  /* When we expect the next packet (in ms) */
uint32_t last_good_gps_time = 0;  /* Time when we last had good GPS data */

/* Navigation data */
float distance_to_tx = -1.0f;
float direction_to_tx = -1.0f;
float relative_direction = -1.0f;
float last_known_good_distance = -1.0f; /* Store last known good distance */

/* RF receiver status */
uint32_t last_ping_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void Error_Handler(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Forward declarations of global variables used in display functions */
/* No need for extern declarations since these are already defined in this file */

/**
 * @brief Display GPS data screen
 * @param gps_data Pointer to GPS data structure
 * @retval None
 */
/* Display mode functions moved to display_modes.c */

/**
 * @brief Display compass data screen with enhanced diagnostics
 * @param compass_data Pointer to compass data structure
 * @param compass_heading Pointer to compass heading variable
 * @retval None
 */
/* Display mode functions moved to display_modes.c */

/**
 * @brief Display RF data screen
 * @param remote_gps_data Pointer to remote GPS data structure
 * @param has_valid_remote_gps Pointer to remote GPS validity flag
 * @param last_rf_packet_time Pointer to last RF packet time
 * @param rf_packet_count Pointer to RF packet counter
 * @retval None
 */
/* Display mode functions moved to display_modes.c */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Reinitialize the SysTick with the new clock */
  HAL_InitTick(TICK_INT_PRIORITY);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  /* Initialize display */
  Display_Init();
  
  /* Initialize remote GPS data structure with safe default values */
  memset(&remote_gps_data, 0, sizeof(GPS_Data));
  /* Set default coordinates far away from any likely location (0,0 is in the ocean) */
  remote_gps_data.latitude = 0.0f;
  remote_gps_data.longitude = 0.0f;
  remote_gps_data.fix = 0;
  strcpy(remote_gps_data.fix_str, "0");
  remote_gps_data.satellites = 0;
  strcpy(remote_gps_data.satellites_str, "0");
  /* Initialize debug strings to prevent garbage output */
  strcpy(remote_gps_data.debug_lat, "NO_DATA");
  strcpy(remote_gps_data.debug_lon, "NO_DATA");
  
  /* Display is now fixed in 90° orientation */
  
  Display_Clear();
  /* Draw test pattern to diagnose rotation issues - adjusted for 64x128 display with 270° rotation */
  Display_DrawTextRowCol(0, 0, "TL");      /* Top-left */
  Display_DrawTextRowCol(15, 0, "BL");    /* Bottom-left */
  Display_DrawTextRowCol(0, 8, "TR");     /* Top-right, adjusted position */
  Display_DrawTextRowCol(15, 8, "BR");   /* Bottom-right, adjusted position */
  Display_DrawTextRowCol(7, 2, "CENTER"); /* Center, adjusted position */
  Display_Update();
  HAL_Delay(2000);
  
  /* Initialize GPS */
  if (GPS_Init() != GPS_OK) {
    Display_Clear();
    Display_DrawTextRowCol(1, 1, "GPS Init Error");
    Display_Update();
    
    /* Blink LED to indicate error but don't halt */
    while(1) {
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
      HAL_Delay(250);
    }
  }
  
  /* Initialize RF receiver */
  uint8_t rf_result = RF_Receiver_Init();
  rf_initialized = (rf_result == RF_OK) ? 1 : 0;
  if (!rf_initialized) {
    Display_Clear();
    char err_msg[32];
    snprintf(err_msg, sizeof(err_msg), "RF Init Err: 0x%02X", rf_result);
    Display_DrawTextRowCol(3, 1, err_msg);
    Display_Update();
    HAL_Delay(3000);
  } else {
    Display_Clear();
    Display_DrawTextRowCol(3, 2, "RF Init");
    Display_DrawTextRowCol(4, 3, "OK");
    Display_DrawTextRowCol(5, 1, "1200 baud");
    Display_Update();
    HAL_Delay(1000);
  }
  
  /* Initialize button */
  Button_Init();
  
  /* Track whether SD card is available for logging */
  static uint8_t sd_card_ok = 0;

  /* Initialize SD card (SPI1 for SD, SPI2 for LoRa - separate buses, no contention) */
  SD_Card_Status sd_status = SD_Card_Init();
  if (sd_status == SD_CARD_OK) {
    Display_Clear();
    Display_DrawTextRowCol(3, 2, "SD Card");
    Display_DrawTextRowCol(4, 3, "Ready");
    Display_Update();
    HAL_Delay(1000);
    
    sd_card_ok = 1;

    /* Write/read-back self-test before committing to logging */
    SD_Card_Status test_result = SD_Card_SelfTest();
    Display_Clear();
    if (test_result == SD_CARD_OK) {
      Display_DrawTextRowCol(2, 0, "SD Self-Test");
      Display_DrawTextRowCol(3, 4, "PASS");
    } else {
      uint8_t st_step = 0, st_fres = 0;
      SD_Card_GetSelfTestError(&st_step, &st_fres);
      char st_buf[24];
      Display_DrawTextRowCol(2, 0, "SD Self-Test FAIL");
      snprintf(st_buf, sizeof(st_buf), "S:%d FR:%d", st_step, st_fres);
      Display_DrawTextRowCol(3, 0, st_buf);
      /* CMD24 R1 response and data token - from sd_diskio diagnostics */
      snprintf(st_buf, sizeof(st_buf), "CMD:%02X DT:%02X T:%d",
               sd_diag_cmd_resp, sd_diag_data_token, (int)SD_Type);
      Display_DrawTextRowCol(4, 0, st_buf);
      /* CMD:XX = R1 from CMD24 (00=ok, else error bits) */
      /* DT:XX  = data response token (05=ok, 0B=CRC err, 0D=write err, FF=no resp) */
      /* W:1    = pre-write busy wait timed out */
      sd_card_ok = 0;
    }
    Display_Update();
    HAL_Delay(3000);

    if (sd_card_ok) {
      char log_filename[32];
      if (SD_Card_CreateLogFile(log_filename, sizeof(log_filename)) == SD_CARD_OK) {
        SD_Card_LogEvent("System startup - GPS Radio Beacon Receiver initialized");
      }
    }

    /* Load last known beacon location so navigation works immediately on boot */
    float saved_lat = 0.0f, saved_lon = 0.0f, saved_alt = 0.0f;
    if (SD_Card_LoadLastBeacon(&saved_lat, &saved_lon, &saved_alt) == SD_CARD_OK) {
      remote_gps_data.latitude       = saved_lat;
      remote_gps_data.longitude      = saved_lon;
      remote_gps_data.altitude       = saved_alt;
      remote_gps_data.fix            = 1;
      has_valid_remote_gps           = 1;
      last_good_remote_gps.latitude  = saved_lat;
      last_good_remote_gps.longitude = saved_lon;
      last_good_remote_gps.altitude  = saved_alt;
      last_good_remote_gps.fix       = 1;
      has_last_good_remote_gps       = 1;

      Display_Clear();
      Display_DrawTextRowCol(2, 0, "Last Beacon");
      char coord_buf[24];
      snprintf(coord_buf, sizeof(coord_buf), "%.5f", saved_lat);
      Display_DrawTextRowCol(3, 0, coord_buf);
      snprintf(coord_buf, sizeof(coord_buf), "%.5f", saved_lon);
      Display_DrawTextRowCol(4, 0, coord_buf);
      Display_DrawTextRowCol(5, 0, "Loaded");
      Display_Update();
      HAL_Delay(1500);
    }
  } else {
    SD_Card_Info sd_dbg;
    SD_Card_GetInfo(&sd_dbg);
    Display_Clear();
    Display_DrawTextRowCol(2, 0, "SD Card Error");
    char err_buf[24];
    snprintf(err_buf, sizeof(err_buf), "Stat:%d CMD0:0x%02X", sd_status, sd_dbg.cmd0_response);
    Display_DrawTextRowCol(3, 0, err_buf);
    snprintf(err_buf, sizeof(err_buf), "FS:%lu DET:%d", sd_dbg.write_errors, sd_dbg.detect_pin_state);
    Display_DrawTextRowCol(4, 0, err_buf);
    Display_Update();
    HAL_Delay(3000);
  }
  
  /* Initialize compass */
  if (Compass_Init() != COMPASS_OK) {
    /* Run I2C scan and display results instead of just showing error */
    Compass_DisplayI2CScan();
    
    HAL_Delay(5000); /* Show scan results for 5 seconds */
    
    /* Display I2C scanner results */
    uint8_t device_count = Compass_GetFoundDeviceCount();
    
    /* Display I2C scanner results */
    char buffer[24]; /* Max chars per line on display */
    
    if (device_count > 0) {
      const uint8_t* addresses = Compass_GetFoundAddresses();
      
      Display_Clear();
      snprintf(buffer, sizeof(buffer), "Found %d I2C device(s)", device_count);
      Display_DrawTextRowCol(1, 1, buffer);
      
      /* Display up to 4 addresses */
      for (uint8_t i = 0; i < device_count && i < 4; i++) {
        snprintf(buffer, sizeof(buffer), "0x%02X", addresses[i]);
        Display_DrawTextRowCol(3, 1 + (i * 5), buffer);
      }
      
      /* If more than 4 addresses, show on next line */
      for (uint8_t i = 4; i < device_count && i < 8; i++) {
        snprintf(buffer, sizeof(buffer), "0x%02X", addresses[i]);
        Display_DrawTextRowCol(5, 1 + ((i-4) * 5), buffer);
      }
      
      Display_Update();
      HAL_Delay(5000); /* Show addresses for 5 seconds */
    } else {
      Display_Clear();
      Display_DrawTextRowCol(2, 1, "No I2C devices found!");
      Display_Update();
      HAL_Delay(3000);
    }
    
    /* Set a flag to indicate compass error but continue execution */
    uint8_t compass_error = 1;
    
    /* Blink LED 3 times to indicate error but continue */
    for (int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
      HAL_Delay(200);
    }
  } else {
    /* BNO055 initialized successfully - try to restore saved calibration */
    if (sd_card_ok) {
      uint8_t cal_data[BNO055_CAL_DATA_LEN];
      if (SD_Card_LoadCompassCal(cal_data, sizeof(cal_data)) == SD_CARD_OK) {
        Compass_SetCalibrationData(cal_data, sizeof(cal_data));
        Display_Clear();
        Display_DrawTextRowCol(1, 1, "BNO055 Initialized");
        Display_DrawTextRowCol(2, 1, "Cal: Restored");
        Display_Update();
      } else {
        Display_Clear();
        Display_DrawTextRowCol(1, 1, "BNO055 Initialized");
        Display_DrawTextRowCol(2, 1, "Cal: None saved");
        Display_Update();
      }
    } else {
      Display_Clear();
      Display_DrawTextRowCol(1, 1, "BNO055 Initialized");
      Display_Update();
    }
    HAL_Delay(1000);
  }
  
  /* USER CODE END 2 */

  /* Create persistent GPS data struct outside the loop to preserve state */
  GPS_Data gps_data;
  GPS_Data rf_gps_data;
  memset(&gps_data, 0, sizeof(GPS_Data));
  memset(&rf_gps_data, 0, sizeof(GPS_Data));
  
  /* Initialize debug strings with safe values */
  strncpy(gps_data.debug_lat, "INIT-LAT", GPS_DEBUG_BUFFER_SIZE - 1);
  gps_data.debug_lat[GPS_DEBUG_BUFFER_SIZE - 1] = '\0';
  
  strncpy(gps_data.debug_lon, "INIT-LON", GPS_DEBUG_BUFFER_SIZE - 1);
  gps_data.debug_lon[GPS_DEBUG_BUFFER_SIZE - 1] = '\0';
  
  strncpy(gps_data.debug_sats, "INIT-SAT", GPS_DEBUG_BUFFER_SIZE - 1);
  gps_data.debug_sats[GPS_DEBUG_BUFFER_SIZE - 1] = '\0';
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Variables for data display */
    char buffer[32];
    uint8_t raw_bytes[6];
    uint8_t status_reg;
    uint8_t active_addr;
    
    /* Clear display */
    Display_Clear();
    
    /* Turn LED ON (PC13 is active LOW on most STM32 boards) */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
    
    /* Update button state */
    Button_Update();
    
    /* Check for button press to change display mode */
    if (Button_WasPressed()) {
      /* Cycle to next display mode */
      current_display_mode = (current_display_mode + 1) % 6;
      mode_change_time = HAL_GetTick();
      
      /* LED is already on from main loop, no additional flash needed */
    }

    /* Auto-return to navigation screen after 120s of no button presses */
    if (current_display_mode != DISPLAY_MODE_NAVIGATION &&
        HAL_GetTick() - mode_change_time > 120000) {
      current_display_mode = DISPLAY_MODE_NAVIGATION;
    }
    
    /* Update GPS data */
    uint8_t gps_update_result = GPS_Update(&gps_data);
    
    /* Validate GPS data before marking as valid */
    if ((gps_update_result == GPS_OK && gps_data.fix) || GPS_IsFixed()) {
      /* Additional validation to ensure coordinates are reasonable */
      if (gps_data.latitude != 0.0f && gps_data.longitude != 0.0f && 
          gps_data.latitude >= -90.0f && gps_data.latitude <= 90.0f &&
          gps_data.longitude >= -180.0f && gps_data.longitude <= 180.0f) {
        /* We have valid GPS data - copy it and save as last known good location */
        memcpy(&local_gps_data, &gps_data, sizeof(GPS_Data));
        has_valid_local_gps = 1;
        memcpy(&last_good_local_gps, &gps_data, sizeof(GPS_Data));
        has_last_good_local_gps = 1;
        last_good_gps_time = HAL_GetTick();
        
        /* SD card logging disabled */
        /* static uint32_t last_gps_log_time = 0;
        if (HAL_GetTick() - last_gps_log_time > 10000) {
          SD_Card_LogGPS(&local_gps_data, LOG_TYPE_GPS_LOCAL);
          last_gps_log_time = HAL_GetTick();
        } */
      } else {
        /* Invalid coordinates despite having a fix - don't update local_gps_data */
        has_valid_local_gps = 0;
      }
    } else {
      /* No fix or error in GPS update - don't update local_gps_data */
      has_valid_local_gps = 0;
    }
    
    /* If we don't have valid current GPS data but have last known good data,
       use that for navigation calculations */
    if (!has_valid_local_gps && has_last_good_local_gps) {
      /* Use last known good GPS data for navigation, but keep current data for display */
      /* This allows navigation to continue while showing the current GPS status */
    }
    
    /* Save compass calibration once when BNO055 becomes fully calibrated */
    if (sd_card_ok) {
      static uint8_t compass_cal_saved = 0;
      if (!compass_cal_saved) {
        uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
        if (Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal) == COMPASS_OK
            && mag_cal == 3) {
          uint8_t cal_data[BNO055_CAL_DATA_LEN];
          if (Compass_GetCalibrationData(cal_data, sizeof(cal_data)) == COMPASS_OK) {
            SD_Card_SaveCompassCal(cal_data, sizeof(cal_data));
          }
          compass_cal_saved = 1; /* Don't retry even if save failed */
        }
      }
    }

    /* Check for RF data - check MULTIPLE times to catch queued packets */
    /* A packet may arrive while processing the first one - catch it immediately */
    for (int rf_check = 0; rf_check < 3; rf_check++) {
      if (rf_initialized && RF_Receiver_DataAvailable()) {
        uint8_t status = RF_Receiver_GetGPSData(&rf_gps_data); 
        if (status == RF_OK) {
          /* Validate remote GPS data before marking as valid */
          if (rf_gps_data.latitude != 0.0f && rf_gps_data.longitude != 0.0f && 
              rf_gps_data.latitude >= -90.0f && rf_gps_data.latitude <= 90.0f &&
              rf_gps_data.longitude >= -180.0f && rf_gps_data.longitude <= 180.0f) {
            /* Valid remote GPS data - copy it and save as last known good */
            memcpy(&remote_gps_data, &rf_gps_data, sizeof(GPS_Data));
            memcpy(&last_good_remote_gps, &rf_gps_data, sizeof(GPS_Data));
            has_valid_remote_gps = 1;
            has_last_good_remote_gps = 1;

            /* Persist beacon location and log navigation data point */
            if (sd_card_ok) {
              /* Rate-limit BEACON.TXT persistence.
               * This does FA_CREATE_ALWAYS + f_sync + f_close, which is the
               * most expensive FatFS operation (can block 100ms-1s+ during SD
               * garbage collection). Post-launch the transmitter goes into
               * continuous-send mode (~1.72s cadence), so doing this on every
               * packet starves the main loop and causes the display to update
               * in delayed bursts. BEACON.TXT is only read at boot for
               * last-known-position recovery; staleness of 10s is harmless. */
              static uint32_t last_beacon_save_ms = 0;
              uint32_t now_ms = HAL_GetTick();
              if (now_ms - last_beacon_save_ms >= 10000U) {
                SD_Card_SaveLastBeacon(rf_gps_data.latitude,
                                       rf_gps_data.longitude,
                                       rf_gps_data.altitude);
                last_beacon_save_ms = now_ms;
              }

              /* Get RSSI of this packet */
              int16_t pkt_rssi;
              int8_t  pkt_snr;
              RF_Receiver_GetSignalQuality(&pkt_rssi, &pkt_snr);

              /* Log navigation snapshot (uses current calculated values) */
              GPS_Data *base_ptr = has_valid_local_gps ? &local_gps_data : 
                                   (has_last_good_local_gps ? &last_good_local_gps : NULL);
              SD_Card_LogNavigation(&rf_gps_data, base_ptr,
                                    distance_to_tx, direction_to_tx,
                                    compass_heading, pkt_rssi);
            }
          } else {
            /* Invalid coordinates despite receiving a packet */
            has_valid_remote_gps = 0;
          }
          uint32_t current_packet_time = HAL_GetTick();
          
          /* Calculate how many packets we missed in the interval since last packet */
          if (rf_packet_count > 0 && last_rf_packet_time > 0) {
            uint32_t time_since_last = current_packet_time - last_rf_packet_time;
            
            /* Calculate expected packets in this interval (with 2.5s tolerance) */
            /* If interval >= 7.5s (5s + 2.5s tolerance), we missed at least 1 packet */
            if (time_since_last >= 7500) {
              /* Number of 5-second periods that have passed */
              uint32_t intervals = (time_since_last + 2500) / 5000;  /* Round to nearest */
              
              /* We got 1 packet, so missed = intervals - 1 */
              uint32_t missed_in_gap = intervals - 1;
              rf_missed_packet_count += missed_in_gap;
            }
          }
          
          /* Recalibrate: reset timer from this packet */
          last_rf_packet_time = current_packet_time;
          
          /* Track first packet time for reference */
          if (rf_packet_count == 0) {
            first_rf_packet_time = last_rf_packet_time;
          }
          
          rf_packet_count++;
          
          /* Brief LED pulse to indicate packet received - non-blocking */
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET); // LED ON (active low)
        }
      } else {
        /* No more packets waiting - break early */
        break;
      }
    }
    
    /* Periodically check RF status even if no new packets are available */
    /* This ensures the navigation screen updates the time since last packet */
    {
      static uint32_t last_rf_check_time = 0;
      uint32_t current_check_time = HAL_GetTick();
      
      /* Check every second */
      if (current_check_time - last_rf_check_time > 1000) {
        last_rf_check_time = current_check_time;
        
        /* If it's been more than 30 seconds since last packet, reset valid flag */
        if (last_rf_packet_time > 0 && current_check_time - last_rf_packet_time > 180000) {
          has_valid_remote_gps = 0;
        }
      }
    }
    
    /* Calculate distance and direction if we have valid GPS data (current or last known good) and remote GPS */
    if ((has_valid_local_gps || has_last_good_local_gps) && has_valid_remote_gps) {
      /* Use math_utils.c functions for accurate distance and bearing calculations */
      
      /* Calculate distance using Haversine formula */
      float calc_distance;
      
      /* Use current GPS data if valid, otherwise use last known good GPS data */
      if (has_valid_local_gps) {
        calc_distance = calculate_distance(
          local_gps_data.latitude, local_gps_data.longitude,
          remote_gps_data.latitude, remote_gps_data.longitude);
      } else {
        /* Use last known good GPS data */
        calc_distance = calculate_distance(
          last_good_local_gps.latitude, last_good_local_gps.longitude,
          remote_gps_data.latitude, remote_gps_data.longitude);
      }
      
      /* Only update if we got a valid distance */
      if (calc_distance >= 0.0f) {
        distance_to_tx = calc_distance / 1000.0f; /* Convert to km */
        /* Store as last known good distance */
        last_known_good_distance = distance_to_tx;
      } else {
        /* Use last known good distance if available */
        if (last_known_good_distance >= 0.0f) {
          distance_to_tx = last_known_good_distance;
        } else {
          distance_to_tx = -1.0f; /* Invalid distance */
        }
      }
      
      /* Calculate bearing (direction) */
      float calc_bearing;
      
      /* Use current GPS data if valid, otherwise use last known good GPS data */
      if (has_valid_local_gps) {
        calc_bearing = calculate_bearing(
          local_gps_data.latitude, local_gps_data.longitude,
          remote_gps_data.latitude, remote_gps_data.longitude);
      } else {
        /* Use last known good GPS data */
        calc_bearing = calculate_bearing(
          last_good_local_gps.latitude, last_good_local_gps.longitude,
          remote_gps_data.latitude, remote_gps_data.longitude);
      }
      
      /* Only update if we got a valid bearing */
      if (calc_bearing >= 0.0f) {
        direction_to_tx = calc_bearing;
        
        /* Calculate relative direction based on compass heading */
        if (compass_heading >= 0.0f && compass_heading <= 360.0f) {
          /* Calculate the relative direction to make arrow point correctly */
          /* This is the angle FROM the compass heading TO the transmitter direction */
          /* We want the arrow to point toward the transmitter */
          /* When facing north (0°), and transmitter is east (90°), arrow should point right (90°) */
          /* When facing east (90°), and transmitter is north (0°), arrow should point left (270°) */
          relative_direction = direction_to_tx - compass_heading;
          /* Normalize to 0-360 range */
          while (relative_direction >= 360.0f) relative_direction -= 360.0f;
          while (relative_direction < 0.0f) relative_direction += 360.0f;
        } else {
          /* Invalid compass heading */
          relative_direction = -1.0f;
        }
      } else {
        /* Invalid bearing */
        direction_to_tx = -1.0f;
        relative_direction = -1.0f;
      }
    }
  
  /* Get current time for throttling and stale data checks */
  uint32_t current_time = HAL_GetTick();
  
  /* Throttle compass updates to prevent I2C blocking during packet reception */
  /* Packets have absolute priority - compass updates are secondary */
  static uint32_t last_compass_update = 0;
  if (current_time - last_compass_update >= 1000) { /* Update compass every 1000ms (1Hz) */
    if (Compass_Update(&compass_data) == COMPASS_OK) {
      compass_heading = compass_data.heading;
    }
    last_compass_update = current_time;
  }
  
  /* Check if RF data is stale (no updates for more than 30 seconds) */
  if (last_rf_packet_time > 0 && current_time - last_rf_packet_time > 180000) {
    has_valid_remote_gps = 0; /* Mark remote GPS as invalid if too old */
    
    /* With stale data, we should still update the relative direction based on compass heading,
       but we should NOT add 180 degrees - that would point the arrow in the wrong direction */
    if (direction_to_tx >= 0.0f && direction_to_tx <= 360.0f && 
        compass_heading >= 0.0f && compass_heading <= 360.0f) {
      /* Calculate the relative direction to make arrow point correctly */
      /* Use the same formula as fresh data: angle FROM compass heading TO transmitter */
      relative_direction = direction_to_tx - compass_heading;
      /* Normalize to 0-360 range */
      while (relative_direction < 0.0f) relative_direction += 360.0f;
      while (relative_direction >= 360.0f) relative_direction -= 360.0f;
    }
  }
  
  /* Clear display buffer before drawing new content */
  Display_Clear();
  
  /* Use button-controlled display mode */
  DisplayMode_t display_mode = current_display_mode;
    
  switch (display_mode) {
    case DISPLAY_MODE_GPS:
      DisplayMode_GPS(&local_gps_data, &last_good_local_gps, has_last_good_local_gps);
      break;
      
    case DISPLAY_MODE_COMPASS_VISUAL:
      DisplayMode_CompassVisual(&compass_data, &compass_heading);
      break;

    case DISPLAY_MODE_COMPASS_HEADING:
      DisplayMode_CompassHeading(&compass_data, &compass_heading);
      break;
      
    case DISPLAY_MODE_RF:
      DisplayMode_RF(&remote_gps_data, &has_valid_remote_gps, &last_rf_packet_time, &rf_packet_count,
                     &last_good_remote_gps, has_last_good_remote_gps);
      break;
      
    case DISPLAY_MODE_NAVIGATION:
      DisplayMode_Navigation(has_valid_local_gps, has_valid_remote_gps, 
                                &local_gps_data, &remote_gps_data, compass_heading,
                                last_rf_packet_time, rf_packet_count, rf_missed_packet_count);
      break;
      
    case DISPLAY_MODE_SD_CARD:
      DisplayMode_SDCard();
      break;
      
    default:
      /* Unknown mode, reset to GPS */
      display_mode = DISPLAY_MODE_GPS;
      break;
  }
  
  /* Output calibration signal periodically for oscilloscope measurement */
  static uint32_t last_calibration_time = 0;
  if (current_time - last_calibration_time > 5000) { /* Every 5 seconds */
    RF_Receiver_OutputCalibrationSignal(20); /* Output calibration pattern */
    last_calibration_time = current_time;
  }
  
  /* Throttle display updates to prevent I2C blocking during packet reception */
  static uint32_t last_display_update = 0;
  if (current_time - last_display_update >= 500) { /* Update display every 500ms to reduce I2C blocking */
    Display_Update();
    last_display_update = current_time;
  }
  
  /* No delay - check packets continuously with zero blocking */
}
/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* For 25MHz crystal: PLLM=25 gives 1MHz PLL input */
  RCC_OscInitStruct.PLL.PLLM = 25;
  /* PLLN=84 gives 84MHz VCO output (reduced from 168) */
  RCC_OscInitStruct.PLL.PLLN = 84;
  /* PLLP=2 gives 42MHz system clock (reduced from 84MHz) */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  /* PLLQ=3.5 gives 24MHz for USB (adjusted for 42MHz system) */
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;  /* Revert to DIV4 since empirical UART timing works */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure LED pin (PC13) */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
  
  /* Initialize LED to OFF state (LED is active LOW on most STM32 boards) */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
  
  /* Configure SD Card CS pin */
  GPIO_InitStruct.Pin = SD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
  
  /* Initialize SD Card CS pin to HIGH (deselected) */
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);
  
  /* Configure SD Card detect pin */
  GPIO_InitStruct.Pin = SD_DETECT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // APB2 = 21MHz, prescaler 256 gives ~82kHz (ultra-safe for SD card init)
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    /* SCK and MOSI: master-driven, no pull needed */
    GPIO_InitStruct.Pin = SD_SPI_SCK_PIN|SD_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* MISO: pull-up required - floats when SD card is deselected/powering up */
    GPIO_InitStruct.Pin = SD_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure CS pin as GPIO output (software controlled) */
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET); /* CS idle high */
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief  SysTick handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HardFault_Handler(void)
{
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
