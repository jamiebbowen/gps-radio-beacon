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
#include "lora.h"      /* LORA_CHANNEL_FREQ_MHZ for log entries */
#include "rf_parser.h" /* RF_PARSER_MAX_CALLSIGN_LEN for callsign logging */
#include "display_modes/sd_card_mode.h"

/* Note: distance/bearing math lives in math_utils.c and button debouncing
 * (50 ms) in button.c - no local constants needed here. */

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
  DISPLAY_MODE_SD_CARD = 5,
  DISPLAY_MODE_CHANNEL = 6,     /* Rocket select: long press cycles channel */
  DISPLAY_MODE_RF_STATS = 7,    /* Full radio statistics */
  DISPLAY_MODE_PREFLIGHT = 8,   /* Pad-side go/no-go checklist */
  DISPLAY_MODE_COUNT            /* Keep last - used for button cycling */
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
/* Set on a button press so the 250 ms display throttle is bypassed for one
 * frame; otherwise a mode change could take up to 250 ms to become visible. */
static uint8_t force_display_update = 0;

/* Flags and counters */
uint8_t has_valid_local_gps = 0;
uint8_t has_valid_remote_gps = 0;
uint8_t has_last_good_local_gps = 0;  /* Flag for last known good local GPS */
uint8_t has_last_good_remote_gps = 0; /* Flag for last known good remote GPS */
uint8_t rf_initialized = 0;
uint32_t last_rf_packet_time = 0;
uint32_t rf_packet_count = 0;
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

  /* Arm the independent watchdog FIRST (LSI/256, full reload: ~32 s).
   * It used to be armed only after all one-time init, so a boot-time
   * hang - wedged I2C scan, SD bring-up, a stuck HAL init - bricked the
   * ground station until a manual power cycle, exactly when the operator
   * can least afford it. Fed once per boot stage below and once per main
   * loop iteration; no single stage legitimately approaches 32 s.
   * Direct register access because the HAL IWDG module isn't in the build.
   * Note: once started, the IWDG cannot be stopped except by reset. */
  IWDG->KR  = 0x5555;   /* unlock PR/RLR */
  IWDG->PR  = 6;        /* prescaler /256 -> 32 kHz LSI / 256 = 125 Hz */
  IWDG->RLR = 0x0FFF;   /* max reload: 4096 / 125 Hz = ~32.8 s */
  IWDG->KR  = 0xCCCC;   /* start */
#define IWDG_FEED() (IWDG->KR = 0xAAAA)
  
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
  /* Corner/centre test pattern to diagnose orientation issues (shared with
   * the test-pattern display mode; see test_pattern.c for geometry notes). */
  Display_ShowTestPattern();
  Display_Update();
  HAL_Delay(2000);
  
  /* Initialize GPS. A failure here must NOT brick the receiver: local GPS
   * only provides the operator's own position for distance/bearing - the
   * core job (hearing the beacon, showing its coordinates) works without
   * it. This used to blink-loop forever, turning a broken local GPS into
   * a completely dead ground station on launch day. */
  uint8_t local_gps_ok = (GPS_Init() == GPS_OK);
  if (!local_gps_ok) {
    Display_Clear();
    Display_DrawTextRowCol(1, 1, "GPS Init Error");
    Display_DrawTextRowCol(3, 1, "No dist/bearing;");
    Display_DrawTextRowCol(4, 1, "RX still works");
    Display_Update();
    HAL_Delay(3000);
  }
  IWDG_FEED();
  
  /* Initialize RF receiver. Retry before giving up: the SX1268 bring-up
   * (reset timing, BUSY, TCXO settle) can fail transiently at power-on,
   * and a single failed attempt used to leave the unit deaf for the whole
   * session - "No signal" with the beacon transmitting a metre away -
   * until a power cycle. Same rationale as the SD mount's 3 attempts. */
  uint8_t rf_result = RF_ERROR;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) HAL_Delay(250);  /* let the radio settle */
    rf_result = RF_Receiver_Init();
    if (rf_result == RF_OK) break;
  }
  rf_initialized = (rf_result == RF_OK) ? 1 : 0;
  if (!rf_initialized) {
    Display_Clear();
    char err_msg[32];
    snprintf(err_msg, sizeof(err_msg), "RF Init Err: 0x%02X", rf_result);
    Display_DrawTextRowCol(3, 1, err_msg);
    Display_DrawTextRowCol(4, 1, "retrying in bg...");
    Display_Update();
    HAL_Delay(3000);
  } else {
    Display_Clear();
    Display_DrawTextRowCol(3, 2, "RF Init");
    Display_DrawTextRowCol(4, 3, "OK");
    Display_DrawTextRowCol(5, 1, "LoRa 433MHz");
    Display_Update();
    HAL_Delay(1000);
    
    /* Scan all rocket channels until the first beacon is heard. Cancelled
     * automatically by a manual channel pick on the Rocket Select page. */
    RF_Receiver_StartScan();
  }
  
  IWDG_FEED();

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

    /* A watchdog-forced reboot must never pass for a normal boot: it means
     * the firmware hung for 32 s. Leave durable evidence, then clear the
     * reset flags so the next boot reads clean. */
    if (sd_card_ok && (RCC->CSR & RCC_CSR_IWDGRSTF)) {
      SD_Card_EnsureLogFile();
      SD_Card_LogError("Boot after IWDG watchdog reset (firmware hang)");
    }

    /* Log file is created lazily on the first RF packet (see
     * SD_Card_LogNavigation in sd_card.c) so no-RF boots leave no artifact. */

    /* Record the RF starting point (boot scan begins at this channel) */
    if (rf_initialized) {
      char rf_msg[48];
      uint8_t boot_ch = RF_Receiver_GetChannel();
      snprintf(rf_msg, sizeof(rf_msg), "RF scan start CH%u %.2fMHz",
               (unsigned)boot_ch, LORA_CHANNEL_FREQ_MHZ(boot_ch));
      SD_Card_LogEvent(rf_msg);
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

  /* Clear the reset flags unconditionally: if SD init failed above, a set
   * IWDGRSTF would otherwise survive into a later boot and be logged then
   * as a spurious "Boot after IWDG watchdog reset". */
  RCC->CSR |= RCC_CSR_RMVF;

  IWDG_FEED();

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
        Display_Clear();
        Display_DrawTextRowCol(1, 1, "BNO055 Initialized");
        if (Compass_SetCalibrationData(cal_data, sizeof(cal_data)) == COMPASS_OK) {
          Display_DrawTextRowCol(2, 1, "Cal: Restored");
        } else {
          /* Write happened but read-back verification failed - the compass
           * is running UNCALIBRATED despite a valid file on the SD card. */
          Display_DrawTextRowCol(2, 1, "Cal: RESTORE FAILED");
        }
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
  
  /* Re-arm the scan dwell now that we can actually service it. StartScan ran
   * ~5 s into boot, but the init screens above take long enough that the boot
   * channel's entire dwell expired before the first ScanUpdate(), so the scan
   * hopped off CH-boot without ever listening there and burned a full 52 s
   * lap before coming back (measured: every logged boot locked at ~61 s). */
  if (rf_initialized && RF_Receiver_IsScanning()) {
    RF_Receiver_StartScan();
  }

  /* The IWDG was armed at the top of boot; the loop just feeds it. */
  IWDG_FEED();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    IWDG->KR = 0xAAAA;  /* feed the watchdog */

    /* USER CODE BEGIN 3 */
    /* LED = RF activity indicator: pulse for 100 ms after each packet
     * (PC13 is active LOW). Previously the LED was held solid on, which
     * conveyed nothing and contradicted the "pulse" comments. */
    {
      uint8_t led_on = (last_rf_packet_time > 0) &&
                       (HAL_GetTick() - last_rf_packet_time < 100);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN,
                        led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    
    /* Update button state */
    Button_Update();
    
    /* Check for button press to change display mode */
    if (Button_WasPressed()) {
      /* Cycle to next display mode */
      current_display_mode = (current_display_mode + 1) % DISPLAY_MODE_COUNT;
      mode_change_time = HAL_GetTick();
      force_display_update = 1;  /* Show the new mode this iteration */
    }

    /* Channel switching: button 2 on the Rocket Select page, or a long press
     * of button 1 as a fallback for units without the second button wired.
     * A long press anywhere else just cycles the page (a held press must
     * never be a no-op). Button 2 off the Rocket Select page jumps straight
     * home to Navigation. */
    uint8_t cycle_channel = 0;
    if (Button_WasLongPressed()) {
      if (current_display_mode == DISPLAY_MODE_CHANNEL) {
        cycle_channel = 1;
      } else {
        /* A long press only has a special meaning on the Rocket Select
         * page. Anywhere else, fall back to the normal page cycle so a
         * press held a beat too long still does SOMETHING - otherwise
         * holds >=700 ms were silently swallowed and the button felt dead. */
        current_display_mode = (current_display_mode + 1) % DISPLAY_MODE_COUNT;
        mode_change_time = HAL_GetTick();
        force_display_update = 1;
      }
    }
    if (Button2_WasPressed()) {
      if (current_display_mode == DISPLAY_MODE_CHANNEL) {
        cycle_channel = 1;
      } else {
        current_display_mode = DISPLAY_MODE_NAVIGATION;
        mode_change_time = HAL_GetTick();
        force_display_update = 1;
      }
    }
    if (cycle_channel) {
      /* A manual pick overrides the boot scan */
      RF_Receiver_StopScan();
      uint8_t before = RF_Receiver_GetChannel();
      uint8_t after = RF_Receiver_NextChannel();
      if (after != before) {
        /* Everything received so far belongs to the previous rocket */
        has_valid_remote_gps = 0;
        has_last_good_remote_gps = 0;
        last_rf_packet_time = 0;
        if (sd_card_ok) {
          char ch_msg[48];
          snprintf(ch_msg, sizeof(ch_msg), "RF manual CH%u %.2fMHz",
                   (unsigned)after, LORA_CHANNEL_FREQ_MHZ(after));
          SD_Card_LogEvent(ch_msg);
        }
      }
      mode_change_time = HAL_GetTick();
      force_display_update = 1;
    }

    /* Radio dead since boot: keep retrying init every 10 s instead of
     * demanding a power cycle. A recovery mid-session behaves like a fresh
     * boot: start the channel scan and leave a durable note on the card. */
    if (!rf_initialized) {
      static uint32_t last_rf_reinit_ms = 0;
      if (HAL_GetTick() - last_rf_reinit_ms >= 10000u) {
        last_rf_reinit_ms = HAL_GetTick();
        if (RF_Receiver_Init() == RF_OK) {
          rf_initialized = 1;
          RF_Receiver_StartScan();
          if (sd_card_ok) {
            SD_Card_EnsureLogFile();
            SD_Card_LogError("RF radio recovered by runtime re-init");
          }
          force_display_update = 1;
        }
      }
    }

    /* Mid-session radio wedge recoveries (chip fell out of RX): durable
     * breadcrumb so field logs show when the RF front end had to be
     * kicked back to life. */
    if (rf_initialized && sd_card_ok) {
      static uint32_t wedges_logged = 0;
      uint32_t wedges = RF_Receiver_GetWedgesRecovered();
      if (wedges != wedges_logged) {
        char w_msg[40];
        snprintf(w_msg, sizeof(w_msg), "RF wedge recovered (#%lu)",
                 (unsigned long)wedges);
        SD_Card_EnsureLogFile();
        SD_Card_LogError(w_msg);
        wedges_logged = wedges;
      }
    }

    /* Drive the boot channel scan; returns 1 the moment a packet locks it */
    if (rf_initialized && RF_Receiver_ScanUpdate()) {
      if (sd_card_ok) {
        char ch_msg[48];
        uint8_t locked_ch = RF_Receiver_GetChannel();
        snprintf(ch_msg, sizeof(ch_msg), "RF scan locked CH%u %.2fMHz",
                 (unsigned)locked_ch, LORA_CHANNEL_FREQ_MHZ(locked_ch));
        /* A lock proves a beacon is on the air; the lazily-created log file
         * may not exist yet (no NAV row before the first position packet) */
        SD_Card_EnsureLogFile();
        SD_Card_LogEvent(ch_msg);
      }
      force_display_update = 1;
    }

    /* Log beacon heartbeats (no-fix keepalives). Rate-limited to one per
     * 5 s at the transmitter, so logging each one is cheap and gives a
     * pad-wait record of sat acquisition progress. */
    if (rf_initialized && sd_card_ok) {
      HeartbeatPacket_t hb;
      if (RF_Receiver_GetHeartbeat(&hb)) {
        int16_t hb_rssi;
        int8_t  hb_snr;
        RF_Receiver_GetSignalQuality(&hb_rssi, &hb_snr);
        /* gps= verdict: acq (acquiring, normal) / SILENT (UART dead - wiring)
         * / GARBLED (bytes but no NMEA) / unk (pre-health TX firmware) */
        static const char *hb_gps_names[] = {"unk", "SILENT", "GARBLED", "acq"};
        uint8_t hb_gps = HB_GPS_STATE(hb.gps_health);
        char hb_msg[80];
        snprintf(hb_msg, sizeof(hb_msg),
                 "HEARTBEAT id=%u CH%u sats=%u fix=%u up=%us rssi=%d gps=%s rst=%u",
                 (unsigned)hb.rocket_id, (unsigned)hb.channel,
                 (unsigned)hb.satellites, (unsigned)hb.fix_quality,
                 (unsigned)hb.uptime_s, (int)hb_rssi,
                 (hb_gps <= HB_GPS_ACQUIRING) ? hb_gps_names[hb_gps] : "?",
                 (unsigned)HB_GPS_RESETS(hb.gps_health));
        SD_Card_EnsureLogFile();
        SD_Card_LogEvent(hb_msg);
      }
    }

    /* Log noise-alert transitions (edge-triggered; the alert itself has
     * hysteresis in rf_receiver). Deliberately no EnsureLogFile: noise can
     * occur with no beacon on the air, and a no-RF boot must not leave
     * artifacts on the card - the event is dropped until a real packet
     * (NAV/heartbeat/scan lock) creates the file. */
    if (rf_initialized) {
      static uint8_t prev_noise_alert = 0;
      uint8_t noise_alert = RF_Receiver_NoiseAlert();
      if (noise_alert != prev_noise_alert) {
        prev_noise_alert = noise_alert;
        if (sd_card_ok) {
          int16_t nf = 0;
          uint8_t nf_valid = RF_Receiver_GetNoiseFloor(&nf);
          char nf_msg[48];
          uint8_t ch = RF_Receiver_GetChannel();
          if (nf_valid) {
            snprintf(nf_msg, sizeof(nf_msg), "RF NOISE %s CH%u %.2fMHz nf=%ddBm",
                     noise_alert ? "HIGH" : "clear",
                     (unsigned)ch, LORA_CHANNEL_FREQ_MHZ(ch), (int)nf);
          } else {
            /* No floor to report: the window was just reset by a channel
             * change (which is also what cleared the alert). Logging the
             * uninitialized 0 as "nf=0dBm" made channel hops look like a
             * noise event with an impossible floor. */
            snprintf(nf_msg, sizeof(nf_msg), "RF NOISE %s CH%u %.2fMHz",
                     noise_alert ? "HIGH" : "clear",
                     (unsigned)ch, LORA_CHANNEL_FREQ_MHZ(ch));
          }
          SD_Card_LogEvent(nf_msg);
        }
        force_display_update = 1;  /* Surface the warning immediately */
      }
    }

    /* Auto-return to navigation screen after 120s of no button presses */
    if (current_display_mode != DISPLAY_MODE_NAVIGATION &&
        HAL_GetTick() - mode_change_time > 120000) {
      current_display_mode = DISPLAY_MODE_NAVIGATION;
    }
    
    /* Update GPS data (skip if init failed - its UART was never set up) */
    uint8_t gps_update_result = local_gps_ok ? GPS_Update(&gps_data) : GPS_ERROR;
    
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
        
        /* Local-GPS rows are deliberately not logged - the NAV rows carry
         * the base position already; duplicating it just wears the card. */
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
    
    /* Persist compass calibration as soon as it is "good enough" (mag_cal >= 2),
     * and re-save whenever calibration quality improves. Only latch on a
     * successful save so transient SD errors are retried. */
    if (sd_card_ok) {
      static uint8_t best_mag_cal_saved = 0; /* 0 = nothing persisted yet */
      static uint32_t last_cal_save_ms = 0;

      uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
      if (Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal) == COMPASS_OK
          && mag_cal >= 2
          && mag_cal > best_mag_cal_saved
          && (HAL_GetTick() - last_cal_save_ms) > 2000) {
        uint8_t cal_data[BNO055_CAL_DATA_LEN];
        if (Compass_GetCalibrationData(cal_data, sizeof(cal_data)) == COMPASS_OK
            && SD_Card_SaveCompassCal(cal_data, sizeof(cal_data)) == SD_CARD_OK) {
          best_mag_cal_saved = mag_cal;
        }
        last_cal_save_ms = HAL_GetTick();
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
              /* Rate-limit BEACON.TXT persistence. A save is a full
               * littlefs close/commit cycle (can block ~100 ms+ during flash
               * GC). Post-launch packets arrive at ~2 s cadence, so doing
               * this on every packet starves the main loop and the display
               * updates in delayed bursts. BEACON.TXT is only read at boot
               * for last-known-position recovery; staleness of 10 s is
               * harmless. */
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
                                    compass_heading, pkt_rssi, pkt_snr);
            }
          } else {
            /* Invalid coordinates despite receiving a packet */
            has_valid_remote_gps = 0;
          }
          last_rf_packet_time = HAL_GetTick();
          rf_packet_count++;
          /* LED pulse handled at top of loop from last_rf_packet_time */
          
          /* Log the transmitting rocket's ID once per change: the beacon's
           * periodic callsign packet is "CALL-<id> CH<n>" and the parser
           * retains the last one heard across packets. */
          if (sd_card_ok) {
            static char last_logged_callsign[RF_PARSER_MAX_CALLSIGN_LEN] = "";
            char heard_callsign[RF_PARSER_MAX_CALLSIGN_LEN] = "";
            GPS_Data cs_scratch;
            if (RF_Receiver_GetParsedData(&cs_scratch, heard_callsign,
                                          sizeof(heard_callsign), NULL) &&
                heard_callsign[0] != '\0' &&
                strcmp(heard_callsign, last_logged_callsign) != 0) {
              char cs_msg[48];
              snprintf(cs_msg, sizeof(cs_msg), "CALLSIGN %s rx-ch CH%u",
                       heard_callsign, (unsigned)RF_Receiver_GetChannel());
              SD_Card_EnsureLogFile();
              if (SD_Card_LogEvent(cs_msg) == SD_CARD_OK) {
                strncpy(last_logged_callsign, heard_callsign,
                        sizeof(last_logged_callsign) - 1);
                last_logged_callsign[sizeof(last_logged_callsign) - 1] = '\0';
              }
            }
          }
        }
      } else {
        /* No more packets waiting - break early */
        break;
      }
    }
    
    /* (Stale-RF invalidation happens in the every-loop check below.) */

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
  if (current_time - last_compass_update >= 200) { /* 5 Hz: walking-speed heading
                                                     * feedback; each update is ~1ms of I2C */
    if (Compass_Update(&compass_data) == COMPASS_OK) {
      compass_heading = compass_data.heading;
    }
    last_compass_update = current_time;
  }
  
  /* SD session marker: log the moment contact is lost (once per loss), so
   * post-flight log review can tell "signal lost at T" from "operator
   * stopped logging". Re-arms when contact resumes. 5 min matches the
   * auto re-scan threshold: far beyond the slowest beacon cadence (60 s). */
  static uint8_t lost_logged = 0;
  if (last_rf_packet_time > 0 && current_time - last_rf_packet_time > 300000) {
    if (!lost_logged) {
      lost_logged = 1;
      SD_Card_LogEvent("RF LOST (>5min silence)");
    }
  } else if (last_rf_packet_time > 0) {
    lost_logged = 0;
  }

  /* Check if RF data is stale (no updates for more than 180 seconds) */
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
                                last_rf_packet_time, rf_packet_count);
      break;
      
    case DISPLAY_MODE_SD_CARD:
      DisplayMode_SDCard();
      break;
      
    case DISPLAY_MODE_CHANNEL:
      DisplayMode_ChannelSelect();
      break;
      
    case DISPLAY_MODE_RF_STATS:
      DisplayMode_RFStats();
      break;

    case DISPLAY_MODE_PREFLIGHT: {
      /* Link freshness: the gate must EXCEED the pre-launch cadence
       * (PRE_LAUNCH_INTERVAL_SEC = 20 s), otherwise the check flaps CHK
       * for the last 5 s of every pad cycle. 25 s covers one full cycle
       * plus margin; heartbeats (5 s) fill the gap in no-fix state. */
      uint8_t pf_link_ok = 0;
      uint32_t pf_link_age_s = 0;
      if (last_rf_packet_time > 0) {
        pf_link_age_s = (current_time - last_rf_packet_time) / 1000;
        pf_link_ok = (pf_link_age_s <= 25);
      }
      int16_t pf_rssi = 0;
      int8_t pf_snr = 0;
      RF_Receiver_GetSignalQuality(&pf_rssi, &pf_snr);

      /* Heartbeat health is only meaningful when the heartbeat is fresh;
       * once position packets flow it goes quiet by design. */
      HeartbeatPacket_t pf_hb;
      uint32_t pf_hb_age = 0;
      uint8_t pf_hb_state = 0xFF;
      if (RF_Receiver_GetLastHeartbeat(&pf_hb, &pf_hb_age) && pf_hb_age < 15000) {
        pf_hb_state = HB_GPS_STATE(pf_hb.gps_health);
      }

      DisplayMode_Preflight(pf_link_ok, pf_link_age_s, pf_rssi,
                            remote_gps_data.fix, (uint8_t)remote_gps_data.satellites,
                            pf_hb_state,
                            has_valid_local_gps, (uint8_t)local_gps_data.satellites,
                            compass_data.heading_valid && !compass_data.heading_stale,
                            sd_card_ok);
      break;
    }
      
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
  
  /* Throttle display updates to prevent I2C blocking during packet reception.
   * A button-triggered mode change bypasses the throttle so the new screen
   * appears immediately. */
  static uint32_t last_display_update = 0;
  if (force_display_update || current_time - last_display_update >= 250) {
    Display_Update();
    last_display_update = current_time;
    force_display_update = 0;
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
  /* PLLQ=4 gives 21MHz on the Q output (USB unused on this board) */
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
  /* Reset immediately rather than spinning until the IWDG fires (~32 s):
   * HAL errors land here with the system in an unknown state, and every
   * second spent hung is a second of beacon packets missed. The IWDG
   * remains the backstop if the reset itself fails. */
  __disable_irq();
  NVIC_SystemReset();
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
  /* Corrupt CPU state: reboot now, not after the 32 s IWDG timeout. */
  NVIC_SystemReset();
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
