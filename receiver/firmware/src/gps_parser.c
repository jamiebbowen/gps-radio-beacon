/**
 * @file gps_parser.c
 * @brief GPS NMEA parser module implementation
 */

#include "gps_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Private defines */
#define GPS_BUFFER_SIZE             128

/* Valid coordinate ranges */
#define GPS_MIN_VALID_LATITUDE      -90.0f
#define GPS_MAX_VALID_LATITUDE      90.0f
#define GPS_MIN_VALID_LONGITUDE     -180.0f
#define GPS_MAX_VALID_LONGITUDE     180.0f

/* Diagnostic counters */
uint32_t gps_checksum_errors = 0;
uint32_t gps_invalid_coordinate_errors = 0;

/* Last known good coordinates */
static float last_valid_latitude = 0.0f;
static float last_valid_longitude = 0.0f;
static uint8_t has_last_valid_position = 0;

/* Private function prototypes */
static uint8_t GPS_ParseGPRMC(const char* nmea_sentence, GPS_Data *gps_data);
static uint8_t GPS_ParseGPGGA(const char* nmea_sentence, GPS_Data *gps_data);
static uint8_t GPS_ValidateChecksum(const char* nmea_sentence);
static uint8_t GPS_ValidateCoordinates(float latitude, float longitude);
static float GPS_NMEAStringToDecimalDegrees(const char* nmea_coord);

/**
 * @brief Parse NMEA sentence
 * @param nmea_sentence NMEA sentence string
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
uint8_t GPS_ParseNMEA(const char* nmea_sentence, GPS_Data *gps_data)
{
  uint8_t status = GPS_PARSER_ERROR;
  
  /* Set debug strings that fit within 10-character display limit */
  if (nmea_sentence != NULL) {
    /* First debug field: NMEA sentence type (first 6 chars) */
    memset(gps_data->debug_lat, 0, GPS_DEBUG_BUFFER_SIZE);
    if (strlen(nmea_sentence) >= 6) {
      strncpy(gps_data->debug_lat, nmea_sentence, 6);
    } else {
      strcpy(gps_data->debug_lat, "EMPTY");
    }
    
    /* Second debug field: Length of NMEA sentence */
    memset(gps_data->debug_lon, 0, GPS_DEBUG_BUFFER_SIZE);
    if (nmea_sentence != NULL) {
      snprintf(gps_data->debug_lon, 10, "LEN:%d", (int)strlen(nmea_sentence));
    }
    
    /* Third debug field: First few chars after type */
    memset(gps_data->debug_sats, 0, GPS_DEBUG_BUFFER_SIZE);
    if (strlen(nmea_sentence) > 7) {
      strncpy(gps_data->debug_sats, nmea_sentence + 7, 9);
    } else {
      strcpy(gps_data->debug_sats, "NO_DATA");
    }
  } else {
    strcpy(gps_data->debug_lat, "NULL");
    strcpy(gps_data->debug_lon, "NULL");
    strcpy(gps_data->debug_sats, "NULL");
  }
  
  /* First perform basic structure validation of the NMEA sentence */
  if (nmea_sentence == NULL || strlen(nmea_sentence) < 10) {
    /* Sentence is too short to be valid */
    return GPS_PARSER_ERROR;
  }
  
  /* Check for proper start character */
  if (nmea_sentence[0] != '$') {
    return GPS_PARSER_ERROR;
  }
  
  /* Check for minimum number of commas (at least 3) */
  int comma_count = 0;
  for (const char* p = nmea_sentence; *p; p++) {
    if (*p == ',') comma_count++;
  }
  
  if (comma_count < 3) {
    /* Not enough data fields */
    return GPS_PARSER_ERROR;
  }
  
  /* Validate the checksum if present */
  uint8_t checksum_valid = 0; /* Assume invalid if no checksum */
  
  if (strchr(nmea_sentence, '*') != NULL) {
    checksum_valid = GPS_ValidateChecksum(nmea_sentence);
    if (!checksum_valid) {
      gps_checksum_errors++;
      
      /* Add checksum error info to debug fields */
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "CSUM ERR");
      
      /* Return error for invalid checksum */
      return GPS_PARSER_ERROR;
    }
  } else {
    /* No checksum marker found - reject the sentence */
    return GPS_PARSER_ERROR;
  }
  
  /* Check for GPRMC sentence (Recommended Minimum data) */
  if (nmea_sentence != NULL && strncmp((char*)nmea_sentence, "$GPRMC", 6) == 0) {
    status = GPS_ParseGPRMC(nmea_sentence, gps_data);
  }
  /* Check for GPGGA sentence (Fix information) */
  else if (nmea_sentence != NULL && strncmp((char*)nmea_sentence, "$GPGGA", 6) == 0) {
    status = GPS_ParseGPGGA(nmea_sentence, gps_data);
  }
  
  /* Update timestamp */
  if (status == GPS_PARSER_OK) {
    /* Check if we have a valid GPS fix */
    if (!gps_data->fix) {
      /* No valid fix, reject the data */
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "NO FIX");
      return GPS_PARSER_ERROR;
    }
    
    /* Validate coordinates before accepting them */
    if (!GPS_ValidateCoordinates(gps_data->latitude, gps_data->longitude)) {
      /* Add invalid coordinate info to debug fields */
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "COORD ERR");
      
      /* Return error for invalid coordinates */
      return GPS_PARSER_ERROR;
    }
    
    gps_data->timestamp = HAL_GetTick();
  }
  
  return status;
}

/**
 * @brief Parse GPRMC sentence
 * @param nmea_sentence NMEA sentence string
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
static uint8_t GPS_ParseGPRMC(const char* nmea_sentence, GPS_Data *gps_data)
{
  char *token;
  char *saveptr;
  char buffer[GPS_BUFFER_SIZE];
  uint8_t token_count = 0;
  
  /* Copy sentence to buffer for tokenization */
  strncpy(buffer, nmea_sentence, GPS_BUFFER_SIZE);
  
  /* Get first token */
  token = strtok_r(buffer, ",", &saveptr);
  
  /* Parse tokens */
  while (token != NULL) {
    switch (token_count) {
      case 1: /* Time */
        if (strlen(token) >= 6) {
          gps_data->hour = (token[0] - '0') * 10 + (token[1] - '0');
          gps_data->minute = (token[2] - '0') * 10 + (token[3] - '0');
          gps_data->second = (token[4] - '0') * 10 + (token[5] - '0');
        }
        break;
        
      case 2: /* Status (A=active, V=void) */
        gps_data->fix = (token[0] == 'A') ? 1 : 0;
        break;
        
      case 3: /* Latitude */
        if (strlen(token) >= 4) {
          /* Use the more robust string-based conversion */
          gps_data->latitude = GPS_NMEAStringToDecimalDegrees(token);
        }
        break;
        
      case 4: /* N/S indicator */
        if (token[0] == 'S') {
          gps_data->latitude = -gps_data->latitude;
        }
        break;
        
      case 5: /* Longitude */
        if (strlen(token) >= 4) {
          /* Use the more robust string-based conversion */
          gps_data->longitude = GPS_NMEAStringToDecimalDegrees(token);
        }
        break;
        
      case 6: /* E/W indicator */
        if (token[0] == 'W') {
          gps_data->longitude = -gps_data->longitude;
        }
        break;
        
      case 7: /* Speed over ground in knots */
        gps_data->speed = atof(token) * 1.852f; /* Convert knots to km/h */
        break;
        
      case 8: /* Course over ground in degrees */
        gps_data->course = atof(token);
        break;
        
      case 9: /* Date (DDMMYY) */
        if (strlen(token) >= 6) {
          gps_data->day = (token[0] - '0') * 10 + (token[1] - '0');
          gps_data->month = (token[2] - '0') * 10 + (token[3] - '0');
          gps_data->year = 2000 + (token[4] - '0') * 10 + (token[5] - '0');
        }
        break;
    }
    
    /* Get next token */
    token = strtok_r(NULL, ",", &saveptr);
    token_count++;
  }
  
  return (gps_data->fix) ? GPS_PARSER_OK : GPS_PARSER_ERROR;
}

/**
 * @brief Parse GPGGA sentence
 * @param nmea_sentence NMEA sentence string
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
static uint8_t GPS_ParseGPGGA(const char* nmea_sentence, GPS_Data *gps_data)
{
  char *token;
  char *saveptr;
  char buffer[GPS_BUFFER_SIZE];
  uint8_t token_count = 0;
  
  /* Copy sentence to buffer for tokenization */
  strncpy(buffer, nmea_sentence, GPS_BUFFER_SIZE);
  
  /* Get first token */
  token = strtok_r(buffer, ",", &saveptr);
  
  /* Parse tokens */
  while (token != NULL) {
    switch (token_count) {
      case 1: /* Time */
        if (strlen(token) >= 6) {
          gps_data->hour = (token[0] - '0') * 10 + (token[1] - '0');
          gps_data->minute = (token[2] - '0') * 10 + (token[3] - '0');
          gps_data->second = (token[4] - '0') * 10 + (token[5] - '0');
        }
        break;
        
      case 2: /* Latitude */
        if (strlen(token) >= 4) {
          /* Use the more robust string-based conversion */
          gps_data->latitude = GPS_NMEAStringToDecimalDegrees(token);
          /* Store raw latitude string for debugging */
          strncpy(gps_data->debug_lat, token, GPS_DEBUG_BUFFER_SIZE);
        }
        break;
        
      case 3: /* N/S indicator */
        if (token[0] == 'S') {
          gps_data->latitude = -gps_data->latitude;
        }
        break;
        
      case 4: /* Longitude */
        if (strlen(token) >= 4) {
          /* Use the more robust string-based conversion */
          gps_data->longitude = GPS_NMEAStringToDecimalDegrees(token);
          /* Store raw longitude string for debugging */
          strncpy(gps_data->debug_lon, token, GPS_DEBUG_BUFFER_SIZE);
        }
        break;
        
      case 5: /* E/W indicator */
        if (token[0] == 'W') {
          gps_data->longitude = -gps_data->longitude;
        }
        break;
        
      case 6: /* Fix quality (0=no fix, 1=GPS fix, 2=DGPS fix) */
        gps_data->fix = atoi(token);
        break;
        
      case 7: /* Number of satellites */
        gps_data->satellites = atoi(token);
        /* Store raw satellites string for debugging */
        strncpy(gps_data->debug_sats, token, GPS_DEBUG_BUFFER_SIZE);
        break;
        
      case 9: /* Altitude */
        gps_data->altitude = atof(token);
        break;
    }
    
    /* Get next token */
    token = strtok_r(NULL, ",", &saveptr);
    token_count++;
  }
  
  return (gps_data->fix) ? GPS_PARSER_OK : GPS_PARSER_ERROR;
}

/**
 * @brief Convert NMEA coordinate string to decimal degrees
 * @param nmea_coord NMEA coordinate string (e.g. "3953.40284")
 * @retval Decimal degrees value
 */
static float GPS_NMEAStringToDecimalDegrees(const char* nmea_coord)
{
  float value = 0.0f;
  
  if (nmea_coord && *nmea_coord) {
    /* Check if we have valid data */
    if (strlen(nmea_coord) < 3) {
      gps_invalid_coordinate_errors++;
      return 0.0f; /* Invalid coordinate format */
    }
    
    /* Verify the string contains only valid characters (digits, decimal point) */
    for (const char* p = nmea_coord; *p; p++) {
      if (!(*p >= '0' && *p <= '9') && *p != '.') {
        gps_invalid_coordinate_errors++;
        return 0.0f; /* Invalid character in coordinate */
      }
    }
    
    /* Check for decimal point and proper format */
    const char* decimal = strchr(nmea_coord, '.');
    if (decimal == NULL || decimal - nmea_coord < 3) {
      /* No decimal point or it's too close to the beginning */
      gps_invalid_coordinate_errors++;
      return 0.0f;
    }
    
    /* Convert the entire string to a float */
    float nmea_value = atof(nmea_coord);
    
    /* Check for obviously invalid values */
    if (nmea_value == 0.0f) {
      gps_invalid_coordinate_errors++;
      return 0.0f;
    }
        
    /* Degrees are the integer part of dividing by 100 */
    int degrees = (int)(nmea_value / 100.0f);
    
    /* Minutes are the remainder after removing degrees*100 */
    float minutes = nmea_value - (degrees * 100.0f);
    
    /* Validate minutes - should be between 0 and 60 */
    if (minutes < 0.0f || minutes >= 60.0f) {
      /* Invalid minutes value, likely a parsing error */
      gps_invalid_coordinate_errors++;
      return 0.0f;
    }
    
    /* Validate degrees range */
    if (degrees < 0 || degrees > 180) {
      gps_invalid_coordinate_errors++;
      return 0.0f;
    }
    
    /* Convert to decimal degrees using the formula: degrees + minutes/60 */
    value = (float)degrees + (minutes / 60.0f);
    
    /* Final sanity check on the result */
    if (value < -180.0f || value > 180.0f) {
      gps_invalid_coordinate_errors++;
      return 0.0f;
    }
  } else {
    gps_invalid_coordinate_errors++;
  }
  
  return value;
}

/**
 * @brief Validate GPS coordinates for plausibility
 * @param latitude Latitude in decimal degrees
 * @param longitude Longitude in decimal degrees
 * @retval 1 if coordinates are valid, 0 if invalid
 */
static uint8_t GPS_ValidateCoordinates(float latitude, float longitude)
{
  /* Check if coordinates are within valid ranges */
  if (latitude < GPS_MIN_VALID_LATITUDE || latitude > GPS_MAX_VALID_LATITUDE ||
      longitude < GPS_MIN_VALID_LONGITUDE || longitude > GPS_MAX_VALID_LONGITUDE) {
    gps_invalid_coordinate_errors++;
    return 0;
  }
  
  /* Additional validation for longitude - check for common error patterns */
  /* Reject values that are suspiciously close to common error values */
  if (fabs(longitude) > 180.0f || 
      (fabs(longitude) > 0.0f && fabs(longitude) < 1.0f)) {
    /* These are likely parsing errors - reject them */
    gps_invalid_coordinate_errors++;
    return 0;
  }
  
  /* Check for unrealistic values (0,0 is in the ocean off Africa) */
  if (fabs(latitude) < 0.01f && fabs(longitude) < 0.01f) {
    gps_invalid_coordinate_errors++;
    return 0;
  }
  
  /* If we have previous valid coordinates, check for impossible jumps */
  if (has_last_valid_position) {
    /* Calculate rough distance using squared differences (not exact but fast) */
    /* This is a simplified distance check - not accurate but good enough to catch huge jumps */
    float lat_diff = latitude - last_valid_latitude;
    float lon_diff = longitude - last_valid_longitude;
    float squared_distance = lat_diff * lat_diff + lon_diff * lon_diff;
    
    /* Reject if movement is more than ~100km in one update (very approximate) */
    /* 1 degree is roughly 111km, so squared distance of ~0.8 is about 100km */
    if (squared_distance > 0.8f) {
      gps_invalid_coordinate_errors++;
      return 0;
    }
  }
  
  /* Update last valid position */
  last_valid_latitude = latitude;
  last_valid_longitude = longitude;
  has_last_valid_position = 1;
  
  return 1;
}

/**
 * @brief Validate NMEA sentence checksum
 * @param nmea_sentence NMEA sentence string
 * @retval 1 if checksum is valid, 0 if invalid
 */
static uint8_t GPS_ValidateChecksum(const char* nmea_sentence)
{
  /* NMEA checksum is XOR of all characters between $ and * */
  uint8_t calculated_checksum = 0;
  uint8_t expected_checksum = 0;
  
  /* Check for valid sentence format */
  if (nmea_sentence == NULL || nmea_sentence[0] != '$') {
    return 0;
  }
  
  /* Find the checksum marker (*) */
  const char* checksum_marker = strchr(nmea_sentence, '*');
  if (checksum_marker == NULL || strlen(checksum_marker) < 3) {
    /* No checksum in sentence or invalid format */
    return 0;
  }
  
  /* Calculate checksum - XOR all characters between $ and * */
  for (const char* p = nmea_sentence + 1; p < checksum_marker; p++) {
    calculated_checksum ^= *p;
  }
  
  /* Extract expected checksum from the sentence */
  char checksum_str[3];
  checksum_str[0] = *(checksum_marker + 1);
  checksum_str[1] = *(checksum_marker + 2);
  checksum_str[2] = '\0';
  
  /* Convert hex string to integer */
  expected_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
  
  /* Compare calculated and expected checksums */
  return (calculated_checksum == expected_checksum) ? 1 : 0;
}
