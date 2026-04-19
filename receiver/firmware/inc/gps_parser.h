/**
 * @file gps_parser.h
 * @brief GPS NMEA parser module header
 */

#ifndef __GPS_PARSER_H
#define __GPS_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gps.h"
#include <stdint.h>

/* Parser status codes */
#define GPS_PARSER_OK                0
#define GPS_PARSER_ERROR             1

/* Global variables */
extern uint32_t gps_checksum_errors;
extern uint32_t gps_invalid_coordinate_errors;

/* Function prototypes */
uint8_t GPS_ParseNMEA(const char* nmea_sentence, GPS_Data *gps_data);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_PARSER_H */
