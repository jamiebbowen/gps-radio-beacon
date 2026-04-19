/**
 * @file rf_parser.c
 * @brief RF packet parser implementation for GPS radio beacon
 */

#include "rf_parser.h"
#include <string.h>  /* For memcpy and strstr functions */
#include <stdlib.h>  /* For atof and atoi functions */
#include <ctype.h>   /* For isdigit function */
#include <stdio.h>   /* For snprintf function */
#include <math.h>    /* For fabs function */

/* Private variables */
static GPS_Data parsed_gps_data;
static char parsed_callsign[RF_PARSER_MAX_CALLSIGN_LEN];
static uint8_t parsed_data_ready;
static char last_valid_packet[RF_PARSER_BUFFER_SIZE] = {0};
static char last_raw_packet[RF_PARSER_BUFFER_SIZE]; /* Store the last raw packet for debugging */

/*  Diagnostic counters */
static uint32_t parse_attempts = 0;
static uint32_t parse_successes = 0;
static uint32_t parse_failures = 0;

/* Detailed failure counters */
static uint32_t parse_failures_null_packet = 0;
static uint32_t parse_failures_insufficient_fields = 0;

/**
 * @brief Convert NMEA coordinate format to decimal degrees (handles signed coordinates)
 * @param nmea_coord NMEA coordinate string (e.g. "3953.40284" or "-3953.40284")
 * @retval Decimal degrees value (positive or negative)
 */
static float nmea_to_decimal_degrees(const char* nmea_coord) {
  float value = 0.0f;
  
  if (!nmea_coord || !*nmea_coord) {
    return 0.0f;
  }
  
  /* Check for negative sign */
  int is_negative = 0;
  const char* coord_ptr = nmea_coord;
  
  if (*coord_ptr == '-') {
    is_negative = 1;
    coord_ptr++; /* Skip the minus sign */
  }
  
  /* Check if we have valid data after processing sign */
  /* NMEA coordinates need at least 7 characters: DDMM.MM or DDDMM.M */
  if (strlen(coord_ptr) < 7) {
    return 0.0f; /* Invalid coordinate format */
  }
  
  /* Convert the coordinate part to a float */
  float nmea_value = atof(coord_ptr);
  
  /* Validate that we have a reasonable NMEA value */
  /* Should be at least 100.0 for valid DDMM.MMMMM format */
  if (nmea_value < 100.0f) {
    return 0.0f; /* Invalid NMEA format */
  }
  
  /* For NMEA format, we need to extract degrees and minutes */
  /* Degrees are the integer part of dividing by 100 */
  int degrees = (int)(nmea_value / 100.0f);
  
  /* Minutes are the remainder after removing degrees*100 */
  float minutes = nmea_value - (degrees * 100.0f);
  
  /* Validate minutes are in valid range (0-59.999...) */
  if (minutes < 0.0f || minutes >= 60.0f) {
    return 0.0f; /* Invalid minutes value */
  }
  
  /* Convert to decimal degrees using the formula: degrees + minutes/60 */
  value = (float)degrees + (minutes / 60.0f);
  
  /* Apply sign if negative */
  if (is_negative) {
    value = -value;
  }
  
  return value;
}

/**
 * @brief Initialize RF parser
 * @retval Status code
 */
uint8_t RF_Parser_Init(void)
{
  /* Initialize parser state */
  memset(&parsed_gps_data, 0, sizeof(GPS_Data));
  memset(parsed_callsign, 0, sizeof(parsed_callsign));
  parsed_data_ready = 0;
  
  /* Initialize diagnostic counters */
  parse_attempts = 0;
  parse_successes = 0;
  parse_failures = 0;
  
  /* Initialize last_valid_packet with a test string for diagnostic purposes */
  strcpy(last_valid_packet, "PARSER_INITIALIZED");
  
  return RF_PARSER_OK;
}

/**
 * @brief Parse ASCII packet into GPS data structure
 * @param packet ASCII packet string to parse
 * @retval Status code (RF_PARSER_OK if successful, RF_PARSER_ERROR otherwise)
 */

uint8_t RF_Parser_ParseAsciiPacket(const char *packet) {
  parse_attempts++;
  
  /* Store the raw packet for debugging */
  strncpy(last_raw_packet, packet, RF_PARSER_BUFFER_SIZE - 1);
  last_raw_packet[RF_PARSER_BUFFER_SIZE - 1] = '\0';
  
  if (!packet || packet[0] == '\0') {
    parse_failures++;
    parse_failures_null_packet++;
    return RF_PARSER_ERROR;
  }
  
  /* Expected formats (LoRa handles framing and CRC):
   * Fast packet: "±ddmm.mmmmm,±dddmm.mmmmm,alt" (3 fields)
   * Full packet: "±ddmm.mmmmm,±dddmm.mmmmm,alt,sats" (4 fields)
   * Callsign: "CALLSIGN" (no commas)
   */
  char *token;
  char packet_copy[RF_PARSER_BUFFER_SIZE];
  
  /* Make a copy of the packet for strtok_r which modifies the string */
  strncpy(packet_copy, packet, RF_PARSER_BUFFER_SIZE - 1);
  packet_copy[RF_PARSER_BUFFER_SIZE - 1] = '\0';
  
  /* Check if this is a callsign packet (no commas) */
  if (strchr(packet_copy, ',') == NULL) {
    /* This is a callsign packet - extract callsign and return */
    if (strlen(packet_copy) > 0) {
      strncpy(parsed_callsign, packet_copy, RF_PARSER_MAX_CALLSIGN_LEN - 1);
      parsed_callsign[RF_PARSER_MAX_CALLSIGN_LEN - 1] = '\0';
      parsed_data_ready = 1;
      parse_successes++;
      return RF_PARSER_OK;
    }
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Reset parsed_data_ready flag when starting new packet parsing */
  parsed_data_ready = 0;
  
  /* Reset GPS data structure */
  memset(&parsed_gps_data, 0, sizeof(GPS_Data));
  
  /* Start tokenizing the GPS data by commas */
  char *saveptr;
  token = strtok_r(packet_copy, ",", &saveptr);
  
  /* Track which field we're processing - positional parsing */
  int field_index = 0;
  
  /* Process GPS data tokens: lat,lon,alt[,sats] */
  while (token != NULL) {
    switch (field_index) {
      case 0: /* Latitude */
        parsed_gps_data.latitude = nmea_to_decimal_degrees(token);
        break;
        
      case 1: /* Longitude */
        parsed_gps_data.longitude = nmea_to_decimal_degrees(token);
        break;
        
      case 2: /* Altitude */
        parsed_gps_data.altitude = atof(token);
        break;
        
      case 3: /* Satellites (optional - only in full packets) */
        parsed_gps_data.satellites = atoi(token);
        break;
        
      default:
        /* Ignore extra fields */
        break;
    }
    
    /* Get next token */
    token = strtok_r(NULL, ",", &saveptr);
    field_index++;
  }
  
  /* Set default values for fields not present in fast packets */
  if (field_index == 3) {
    /* Fast packet - set defaults for missing fields */
    parsed_gps_data.satellites = 0;  /* Unknown */
    parsed_gps_data.fix = 1;         /* Assume valid fix */
    parsed_gps_data.launch_detected = 1;  /* Fast packets only sent during launch */
    parsed_gps_data.time_since_launch = 0; /* Unknown */
  } else if (field_index == 4) {
    /* Full packet - set defaults for launch status */
    parsed_gps_data.fix = 1;         /* Assume valid fix */
    parsed_gps_data.launch_detected = 0;  /* Full packets sent in non-launch states */
    parsed_gps_data.time_since_launch = 0;
  }
  
  /* Check if we have at least 3 fields (lat,lon,alt) */
  if (field_index < 3) {
    parse_failures++;
    parse_failures_insufficient_fields++;
    return RF_PARSER_ERROR;
  }
  
  /* Validate coordinates before accepting the packet */
  if (parsed_gps_data.latitude == 0.0f || parsed_gps_data.longitude == 0.0f || 
      parsed_gps_data.latitude < -90.0f || parsed_gps_data.latitude > 90.0f ||
      parsed_gps_data.longitude < -180.0f || parsed_gps_data.longitude > 180.0f) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Store the last valid packet */
  strncpy(last_valid_packet, packet, RF_PARSER_BUFFER_SIZE - 1);
  last_valid_packet[RF_PARSER_BUFFER_SIZE - 1] = '\0';
  
  /* Mark data as ready */
  parsed_data_ready = 1;
  
  /* Update diagnostic counters */
  parse_successes++;
  
  return RF_PARSER_OK;
}

/**
 * @brief Get parsed data from the last valid packet
 * @param gps_data Pointer to GPS_Data structure to fill
 * @param callsign Buffer to store callsign
 * @param callsign_size Size of callsign buffer
 * @param checksum Pointer to store checksum
 * @retval Status (1 if data available, 0 if not)
 */
uint8_t RF_Parser_GetParsedData(GPS_Data *gps_data, char *callsign, uint16_t callsign_size, uint8_t *checksum)
{
  if (!parsed_data_ready) {
    return 0;
  }
  
  /* Copy data */
  memcpy(gps_data, &parsed_gps_data, sizeof(GPS_Data));
  
  if (callsign && callsign_size > 0) {
    strncpy(callsign, parsed_callsign, callsign_size - 1);
    callsign[callsign_size - 1] = '\0';
  }
  
  /* checksum parameter kept for API compatibility but unused with LoRa */
  (void)checksum;
    
  return 1;
}

/**
 * @brief Get checksum error count (deprecated - LoRa has built-in CRC)
 * @param checksum_errors Pointer to store checksum error count
 * @retval None
 */
void RF_Parser_GetChecksumErrors(uint16_t *checksum_errors)
{
  /* Function kept for API compatibility but always returns 0 with LoRa */
  if (checksum_errors) {
    *checksum_errors = 0;
  }
}

/**
 * @brief Get the last valid packet from the parser
 * @param buffer Buffer to store the last valid packet
 * @param max_len Maximum length of the buffer
 * @retval Length of the packet stored in the buffer
 */
uint16_t RF_Parser_GetLastValidPacket(char *buffer, uint16_t max_len)
{
  uint16_t packet_len = 0;
  
  if (buffer && max_len > 0) {
    packet_len = strlen(last_valid_packet);
    if (packet_len > max_len - 1) {
      packet_len = max_len - 1;
    }
    
    strncpy(buffer, last_valid_packet, packet_len);
    buffer[packet_len] = '\0'; /* Ensure null termination */
  }
  
  return packet_len;
}

/**
 * @brief Get the last raw packet from the parser
 * @param buffer Buffer to store the last raw packet
 * @param max_len Maximum length of the buffer
 * @retval Length of the packet stored in the buffer
 */
uint16_t RF_Parser_GetLastRawPacket(char *buffer, uint16_t max_len)
{
  uint16_t packet_len = 0;
  
  if (buffer && max_len > 0) {
    packet_len = strlen(last_raw_packet);
    if (packet_len > max_len - 1) {
      packet_len = max_len - 1;
    }
    
    strncpy(buffer, last_raw_packet, packet_len);
    buffer[packet_len] = '\0'; /* Ensure null termination */
  }
  
  return packet_len;
}

/**
 * @brief Get diagnostics from the parser
 * @param attempts Pointer to store number of attempts
 * @param successes Pointer to store number of successes
 * @param failures Pointer to store number of failures
 * @retval None
 */
void RF_Parser_GetDiagnostics(uint32_t *attempts, uint32_t *successes, uint32_t *failures)
{
  if (attempts) {
    *attempts = parse_attempts;
  }
  
  if (successes) {
    *successes = parse_successes;
  }
  
  if (failures) {
    *failures = parse_failures;
  }
}

/**
 * @brief Get detailed failure diagnostics
 * @param null_packet Pointer to store null packet failures
 * @param insufficient_fields Pointer to store insufficient fields failures
 * @param checksum_invalid Pointer to store checksum failures
 * @param marker_not_found Pointer to store marker failures
 * @param callsign_too_short Pointer to store callsign failures
 * @retval None
 */
void RF_Parser_GetDetailedFailures(uint32_t *null_packet, uint32_t *insufficient_fields, 
                                   uint32_t *checksum_invalid, uint32_t *marker_not_found,
                                   uint32_t *callsign_too_short)
{
  if (null_packet) {
    *null_packet = parse_failures_null_packet;
  }
  
  if (insufficient_fields) {
    *insufficient_fields = parse_failures_insufficient_fields;
  }
  
  /* These counters are deprecated with LoRa but kept for API compatibility */
  if (checksum_invalid) {
    *checksum_invalid = 0;
  }
  
  if (marker_not_found) {
    *marker_not_found = 0;
  }
  
  if (callsign_too_short) {
    *callsign_too_short = 0;
  }
}

/**
 * @brief Parse binary GPS packet (13 bytes)
 * @param data Pointer to binary packet data
 * @param length Length of packet in bytes
 * @retval Status code (RF_PARSER_OK if successful, RF_PARSER_ERROR otherwise)
 */
uint8_t RF_Parser_ParseBinaryPacket(const uint8_t *data, uint16_t length) {
  parse_attempts++;
  
  /* Verify packet length */
  if (length != 13) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Verify packet type */
  if (data[0] != 0x01) {  /* 0x01 = GPS packet type */
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Extract binary packet structure */
  /* Note: We manually extract fields to handle any potential alignment issues */
  int32_t latitude_encoded;
  int32_t longitude_encoded;
  int16_t altitude_encoded;
  uint8_t satellites;
  uint8_t flags;
  
  /* Extract latitude (bytes 1-4, little-endian) */
  latitude_encoded = (int32_t)((uint32_t)data[1] | 
                                ((uint32_t)data[2] << 8) | 
                                ((uint32_t)data[3] << 16) | 
                                ((uint32_t)data[4] << 24));
  
  /* Extract longitude (bytes 5-8, little-endian) */
  longitude_encoded = (int32_t)((uint32_t)data[5] | 
                                 ((uint32_t)data[6] << 8) | 
                                 ((uint32_t)data[7] << 16) | 
                                 ((uint32_t)data[8] << 24));
  
  /* Extract altitude (bytes 9-10, little-endian) */
  altitude_encoded = (int16_t)((uint16_t)data[9] | 
                                ((uint16_t)data[10] << 8));
  
  /* Extract satellites (byte 11) */
  satellites = data[11];
  
  /* Extract flags (byte 12) */
  flags = data[12];
  
  /* Convert to decimal degrees (divide by 10^7) */
  double latitude_decimal = (double)latitude_encoded / 10000000.0;
  double longitude_decimal = (double)longitude_encoded / 10000000.0;
  
  /* Validate coordinates (basic range check) */
  if (latitude_decimal < -90.0 || latitude_decimal > 90.0 ||
      longitude_decimal < -180.0 || longitude_decimal > 180.0) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Store parsed data */
  parsed_gps_data.latitude = latitude_decimal;
  parsed_gps_data.longitude = longitude_decimal;
  parsed_gps_data.altitude = (float)altitude_encoded;
  parsed_gps_data.satellites = satellites;
  
  /* Parse flags */
  parsed_gps_data.launch_detected = (flags & 0x80) ? 1 : 0;  /* Bit 7 */
  parsed_gps_data.fix = flags & 0x0F;  /* Bits 3-0: GPS fix type */
  
  /* Mark data as ready */
  parsed_data_ready = 1;
  parse_successes++;
  
  return RF_PARSER_OK;
}

/**
 * @brief Reset parser state
 * @retval None
 */
void RF_Parser_Reset(void)
{
  /* Reset parser state */
  memset(&parsed_gps_data, 0, sizeof(GPS_Data));
  memset(parsed_callsign, 0, sizeof(parsed_callsign));
  memset(last_valid_packet, 0, sizeof(last_valid_packet));
  parsed_data_ready = 0;
  
  /* Reset diagnostic counters */
  parse_attempts = 0;
  parse_successes = 0;
  parse_failures = 0;
}
