/**
 * @file main.h
 * @brief Main header file for the GPS Radio Beacon Receiver
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "compass.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);

/* Display mode functions */
void DisplayMode_GPS(GPS_Data *gps_data, GPS_Data *last_good_gps, uint8_t has_last_good);
void DisplayMode_Compass(Compass_Data *compass_data, float *compass_heading);
void DisplayMode_RF(GPS_Data *remote_gps_data, uint8_t *has_valid_remote_gps, 
    uint32_t *last_rf_packet_time, uint32_t *rf_packet_count,
    GPS_Data *last_good_remote_gps, uint8_t has_last_good_remote);
void DisplayMode_Navigation(uint8_t has_valid_local_gps, uint8_t has_valid_remote_gps,
                           GPS_Data *local_gps_data, GPS_Data *remote_gps_data,
                           float compass_heading, uint32_t last_rf_packet_time,
                           uint32_t rf_packet_count);
/* USER CODE END EFP */
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define LED_PIN                     GPIO_PIN_13
#define LED_GPIO_PORT               GPIOC

/* SD Card SPI Configuration */
#define SD_SPI_INSTANCE             SPI1
#define SD_CS_PIN                   GPIO_PIN_4
#define SD_CS_GPIO_PORT             GPIOA
#define SD_DETECT_PIN               GPIO_PIN_9
#define SD_DETECT_GPIO_PORT         GPIOB  /* Moved from PA8 to PB9 to avoid LoRa RESET conflict */

/* SPI1 Pin Configuration */
#define SD_SPI_SCK_PIN              GPIO_PIN_5
#define SD_SPI_SCK_GPIO_PORT        GPIOA
#define SD_SPI_MISO_PIN             GPIO_PIN_6
#define SD_SPI_MISO_GPIO_PORT       GPIOA
#define SD_SPI_MOSI_PIN             GPIO_PIN_7
#define SD_SPI_MOSI_GPIO_PORT       GPIOA

/* --------------------------------------------------------------------------
 * Receiver v2.0 hardware (see docs/hardware-receiver.md v2.0)
 * SPI1 is a shared bus: SD card + SHARP memory LCD + SSD1680 eInk (+ SRAM).
 * NOTE mixed CS polarity: SHARP CS is ACTIVE HIGH, everything else LOW.
 * ------------------------------------------------------------------------- */

/* SHARP Memory LCD (Adafruit 3502, 168x144) on SPI1 */
#define SHARP_CS_PIN                GPIO_PIN_10  /* PB10 - CS, ACTIVE HIGH */
#define SHARP_CS_GPIO_PORT          GPIOB
#define SHARP_DISP_PIN              GPIO_PIN_8   /* PB8 - display on/off */
#define SHARP_DISP_GPIO_PORT        GPIOB

/* SSD1680 2.9" tri-color eInk FeatherWing (Adafruit 4778) on SPI1 */
#define EINK_CS_PIN                 GPIO_PIN_15  /* PC15 - ECS, active low (was PB11, which
                                                    the BlackPill 2x20 header doesn't expose;
                                                    PC15 is free - no RTC/LSE in this build) */
#define EINK_CS_GPIO_PORT           GPIOC
#define EINK_DC_PIN                 GPIO_PIN_3   /* PB3 - Data/Command */
#define EINK_DC_GPIO_PORT           GPIOB
#define EINK_RST_PIN                GPIO_PIN_4   /* PB4 - eInk reset */
#define EINK_RST_GPIO_PORT          GPIOB
#define EINK_BUSY_PIN               GPIO_PIN_1   /* PA1 - busy, HIGH = busy */
#define EINK_BUSY_GPIO_PORT         GPIOA
#define EINK_SRCS_PIN               GPIO_PIN_5   /* PB5 - SRAM CS (wired, unused:
                                                    framebuffer lives in MCU RAM) */
#define EINK_SRCS_GPIO_PORT         GPIOB

/* Battery voltage divider: 100k over 330k, midpoint -> PA0 (ADC1_IN0).
 * Full-scale 4.30V -> 3.3V; 100nF to GND at the midpoint steadies the
 * ~77k source resistance (requires 480-cycle sampling). */
#define BATTERY_ADC_PIN             GPIO_PIN_0
#define BATTERY_ADC_GPIO_PORT       GPIOA
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_0

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
