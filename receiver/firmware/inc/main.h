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
                           float compass_heading, uint32_t last_rf_packet_time, uint32_t rf_packet_count,
                           uint32_t rf_missed_packet_count);
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

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
