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
/* Relocation candidate: a fix too far from the anchor to be plausible.
 * If the NEXT fix agrees with it (within ~1 km) the receiver really did
 * move - e.g. powered up at a new site after travelling with the fix
 * lost - and the anchor follows instead of rejecting forever. */
static float jump_candidate_lat = 0.0f;
static float jump_candidate_lon = 0.0f;
static uint8_t jump_candidate_valid = 0;
#define GPS_DEG2RAD 0.017453292519943295f

/* Private function prototypes */
static uint8_t GPS_ParseGPRMC(const char* nmea_sentence, GPS_Data *gps_data);
static uint8_t GPS_ParseGPGGA(const char* nmea_sentence, GPS_Data *gps_data);
static uint8_t GPS_ValidateChecksum(const char* nmea_sentence);
static uint8_t GPS_ValidateCoordinates(float latitude, float longitude);
static float GPS_NMEAStringToDecimalDegrees(const char* nmea_coord);

/* atof() parses "nan"/"inf" literally; a corrupt field must not put NaN
 * into GPS_Data where it persists into logs/displays. 0 = "unknown". */
static float nmea_atof_finite(const char *s) {
  float v = s ? (float)atof(s) : 0.0f;
  return isfinite(v) ? v : 0.0f;
}

/* strtol saturates (atoi on an out-of-int-range field is UB) and the
 * [0, cap] clamp keeps corrupt fields like "999" from wrapping at a
 * uint8_t cast. */
static uint8_t nmea_atoi_u8(const char *s, uint8_t cap) {
  char *endp;
  long v = s ? strtol(s, &endp, 10) : 0;
  if (!s || endp == s || v <= 0) return 0;
  return (v > (long)cap) ? cap : (uint8_t)v;
}

/**
 * @brief Parse NMEA sentence
 * @param nmea_sentence NMEA sentence string
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
uint8_t GPS_ParseNMEA(const char* nmea_sentence, GPS_Data *gps_data)
{
  uint8_t status = GPS_PARSER_ERROR;
  
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
      
      /* Return error for invalid checksum */
      return GPS_PARSER_ERROR;
    }
  } else {
    /* No checksum marker found - reject the sentence */
    return GPS_PARSER_ERROR;
  }
  
  /* Dispatch by sentence type. NMEA sentences are "$<talker><type>,..."
   * where talker is 2 chars (GP=GPS, GL=GLONASS, GA=Galileo, GB=BeiDou,
   * GN=combined GNSS) and type is 3 chars. u-blox multi-GNSS modules with
   * GLONASS/Galileo enabled default to GN as the talker, so matching only
   * $GP* silently drops the date-bearing RMC sentence. Match on the type
   * suffix instead so every talker variant is accepted. */
  /* Parse into a scratch copy: nothing reaches the caller's struct until
   * the position has been validated. (Writing first and validating after
   * let a rejected teleport overwrite the position and leave fix=1 behind
   * for GPS_IsFixed() and main.c to use.) */
  GPS_Data parsed = *gps_data;
  uint8_t recognized = 1;
  if (nmea_sentence[0] == '$' &&
      strncmp(&nmea_sentence[3], "RMC,", 4) == 0) {
    status = GPS_ParseGPRMC(nmea_sentence, &parsed);
  }
  else if (nmea_sentence[0] == '$' &&
           strncmp(&nmea_sentence[3], "GGA,", 4) == 0) {
    status = GPS_ParseGPGGA(nmea_sentence, &parsed);
  }
  else {
    recognized = 0;
  }

  if (status == GPS_PARSER_OK) {
    /* fix is guaranteed non-zero here: both GPS_ParseGPRMC and
     * GPS_ParseGPGGA return GPS_PARSER_ERROR when the sentence carries
     * no fix (see their final "return (gps_data->fix) ? ..." lines). */
    if (!GPS_ValidateCoordinates(parsed.latitude, parsed.longitude)) {
      /* Keep the last good coordinates, but this is not a usable fix */
      gps_data->fix = 0;
      return GPS_PARSER_ERROR;
    }
    parsed.timestamp = HAL_GetTick();
    *gps_data = parsed;
  } else if (recognized) {
    /* A GGA/RMC without a usable fix: publish the fix state (never a
     * fix) and the acquisition progress, never the coordinates. */
    gps_data->fix = 0;
    gps_data->satellites = parsed.satellites;
    gps_data->hdop = parsed.hdop;
    memcpy(gps_data->satellites_str, parsed.satellites_str,
           sizeof(gps_data->satellites_str));
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
  
  /* Copy sentence to buffer for tokenization. Force NUL termination since
   * strncpy with bound == sizeof(dst) does not guarantee one. */
  strncpy(buffer, nmea_sentence, GPS_BUFFER_SIZE - 1);
  buffer[GPS_BUFFER_SIZE - 1] = '\0';
  
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
        gps_data->speed = nmea_atof_finite(token) * 1.852f; /* knots -> km/h */
        break;
        
      case 8: /* Course over ground in degrees */
        gps_data->course = nmea_atof_finite(token);
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
  
  /* Copy sentence to buffer for tokenization. Force NUL termination since
   * strncpy with bound == sizeof(dst) does not guarantee one. */
  strncpy(buffer, nmea_sentence, GPS_BUFFER_SIZE - 1);
  buffer[GPS_BUFFER_SIZE - 1] = '\0';
  
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
        }
        break;
        
      case 5: /* E/W indicator */
        if (token[0] == 'W') {
          gps_data->longitude = -gps_data->longitude;
        }
        break;
        
      case 6: /* Fix quality (0=no fix, 1=GPS fix, 2=DGPS fix) */
        /* Spec'd 0-8; a corrupt "9" must not claim a fix - reads as 0. */
        {
          uint8_t fq = nmea_atoi_u8(token, 255);
          gps_data->fix = (fq > 8) ? 0 : fq;
        }
        break;
        
      case 7: /* Number of satellites */
        gps_data->satellites = nmea_atoi_u8(token, 99);
        break;

      case 8: /* HDOP - geometry quality, separates "few sats" from "bad geometry" */
        gps_data->hdop = nmea_atof_finite(token);
        break;

      case 9: /* Altitude */
        gps_data->altitude = nmea_atof_finite(token);
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
    
    /* Check for obviously invalid values. isfinite first: atof() parses
     * "nan"/"inf" literally (NaN != 0.0f slips the old check), and the
     * (int) cast below is UB on non-finite input. */
    if (!isfinite(nmea_value) || nmea_value == 0.0f) {
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
  
  /* GPS_NMEAStringToDecimalDegrees returns exactly 0.0 for a field it
   * could not convert (minutes >= 60, garbage, no decimal point): an exact
   * zero in either axis is a parse failure, not the equator or the prime
   * meridian. (Nearby real positions - London's -0.12 deg - are fine.) */
  if (latitude == 0.0f || longitude == 0.0f) {
    gps_invalid_coordinate_errors++;
    return 0;
  }

  /* Check for unrealistic values (0,0 is in the ocean off Africa) */
  if (fabs(latitude) < 0.01f && fabs(longitude) < 0.01f) {
    gps_invalid_coordinate_errors++;
    return 0;
  }
  
  /* If we have previous valid coordinates, check for impossible jumps.
   * Flat-earth distance in degrees with the longitude scaled by cos(lat):
   * crude, but plenty to catch a ~100 km teleport (0.8 deg^2). */
  if (has_last_valid_position) {
    float lat_diff = latitude - last_valid_latitude;
    float lon_diff = (longitude - last_valid_longitude) *
                     cosf(last_valid_latitude * GPS_DEG2RAD);
    if (lat_diff * lat_diff + lon_diff * lon_diff > 0.8f) {
      /* A genuine relocation repeats: two consecutive far fixes within
       * ~1 km (0.01 deg) of each other move the anchor. */
      uint8_t confirmed = 0;
      if (jump_candidate_valid) {
        float c_lat = latitude - jump_candidate_lat;
        float c_lon = (longitude - jump_candidate_lon) *
                      cosf(jump_candidate_lat * GPS_DEG2RAD);
        confirmed = (c_lat * c_lat + c_lon * c_lon) < 0.0001f;
      }
      if (!confirmed) {
        jump_candidate_lat = latitude;
        jump_candidate_lon = longitude;
        jump_candidate_valid = 1;
        gps_invalid_coordinate_errors++;
        return 0;
      }
    }
  }
  jump_candidate_valid = 0;

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
