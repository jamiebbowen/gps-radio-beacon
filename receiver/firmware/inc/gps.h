/**
 * @file gps.h
 * @brief GPS module driver header for NEO6M
 */

#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>

/* GPS status codes */
#define GPS_OK      0
#define GPS_ERROR   1
#define GPS_TIMEOUT 2

/* Debug buffer size */
#define GPS_DEBUG_BUFFER_SIZE 16

/* GPS data structure */
typedef struct {
  float latitude;          /* Latitude in decimal degrees */
  float longitude;         /* Longitude in decimal degrees */
  float altitude;          /* Altitude in meters */
  float speed;             /* Speed over ground in km/h */
  float course;            /* Course over ground in degrees */
  uint8_t satellites;      /* Number of satellites in use (numeric value) */
  uint8_t fix;             /* GPS fix status (numeric value) */
  char satellites_str[4];  /* Number of satellites as string */
  char fix_str[2];         /* Fix status as string */
  uint8_t hour;            /* Hour (UTC) */
  uint8_t minute;          /* Minute (UTC) */
  uint8_t second;          /* Second (UTC) */
  uint8_t day;             /* Day (1-31) */
  uint8_t month;           /* Month (1-12) */
  uint16_t year;           /* Year (2000+) */
  uint32_t timestamp;      /* Timestamp of last update (system ticks) */
  /* Launch detection fields (from RF beacon) */
  uint8_t launch_detected; /* Launch status: 1 if launched, 0 if not */
  uint32_t time_since_launch; /* Time since launch in seconds (0 if not launched) */
  /* Debug fields */
  char debug_lat[GPS_DEBUG_BUFFER_SIZE];  /* Raw latitude string from NMEA */
  char debug_lon[GPS_DEBUG_BUFFER_SIZE];  /* Raw longitude string from NMEA */
  char debug_sats[GPS_DEBUG_BUFFER_SIZE]; /* Raw satellites string from NMEA */
} GPS_Data;

/* Function prototypes */
uint8_t GPS_Init(void);
uint8_t GPS_Update(GPS_Data *gps_data);
uint8_t GPS_IsFixed(void);
void GPS_GetRawBytes(uint8_t *bytes);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */
