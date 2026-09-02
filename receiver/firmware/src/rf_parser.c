/**
 * @file rf_parser.c
 * @brief RF packet parser implementation for GPS radio beacon
 */

#include "rf_parser.h"
#include "packet_format.h"
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
  
  /* Convert in DOUBLE precision. NMEA ddmm.mmmmm carries ~9 significant
   * digits; single-precision float only holds ~7, which both loses ~1 m of
   * coordinate precision and can round invalid minutes (e.g. 3999.99999 ->
   * 4000.0f) past the minutes-range validation below. */
  double nmea_value = atof(coord_ptr);
  
  /* Validate that we have a reasonable NMEA value */
  /* Should be at least 100.0 for valid DDMM.MMMMM format */
  if (nmea_value < 100.0) {
    return 0.0f; /* Invalid NMEA format */
  }
  
  /* For NMEA format, we need to extract degrees and minutes */
  /* Degrees are the integer part of dividing by 100 */
  int degrees = (int)(nmea_value / 100.0);
  
  /* Minutes are the remainder after removing degrees*100 */
  double minutes = nmea_value - (degrees * 100.0);
  
  /* Validate minutes are in valid range (0-59.999...) */
  if (minutes < 0.0 || minutes >= 60.0) {
    return 0.0f; /* Invalid minutes value */
  }
  
  /* Convert to decimal degrees using the formula: degrees + minutes/60,
   * rounding to float only once at the very end. */
  double value = (double)degrees + (minutes / 60.0);
  
  /* Apply sign if negative */
  if (is_negative) {
    value = -value;
  }
  
  return (float)value;
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
  parse_failures_null_packet = 0;
  parse_failures_insufficient_fields = 0;
  
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
  
  /* Reject NULL/empty packets BEFORE touching the debug copy - copying from
   * a NULL pointer is undefined behavior (was a HardFault waiting to happen). */
  if (!packet || packet[0] == '\0') {
    parse_failures++;
    parse_failures_null_packet++;
    return RF_PARSER_ERROR;
  }
  
  /* Store the raw packet for debugging */
  strncpy(last_raw_packet, packet, RF_PARSER_BUFFER_SIZE - 1);
  last_raw_packet[RF_PARSER_BUFFER_SIZE - 1] = '\0';
  
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
    /* Callsign packet - extract callsign and return. Cannot be empty:
     * NULL/empty packets were rejected at function entry above. */
    strncpy(parsed_callsign, packet_copy, RF_PARSER_MAX_CALLSIGN_LEN - 1);
    parsed_callsign[RF_PARSER_MAX_CALLSIGN_LEN - 1] = '\0';
    parsed_data_ready = 1;
    parse_successes++;
    return RF_PARSER_OK;
  }
  
  /* Reset parsed_data_ready flag when starting new packet parsing */
  parsed_data_ready = 0;

  /* Reset only the fields this NMEA-style parser is responsible for writing.
   * Crucially we do NOT touch v_north / v_east / v_down / is_fused / fused_*
   * here - those fields are owned by the FUSED-packet parser and must
   * persist across GPS packets so the navigation display can show the last
   * known ground speed even while GPS packets are arriving. A blanket memset
   * zeroed them every GPS packet, which made "S:13 12.3m/s" flicker back
   * to "S:13  0.0m/s" on every GPS update. */
  parsed_gps_data.latitude  = 0.0f;
  parsed_gps_data.longitude = 0.0f;
  parsed_gps_data.altitude  = 0.0f;
  parsed_gps_data.satellites = 0;
  parsed_gps_data.fix = 0;
  parsed_gps_data.launch_detected = 0;
  parsed_gps_data.time_since_launch = 0;
  
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
  
  /* Verify pointer and packet length (13 bytes = sizeof(BinaryGPSPacket_t)) */
  if (data == NULL || length != 13) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  
  /* Verify packet type */
  if (data[0] != PACKET_TYPE_GPS) {
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

  /* Parse flags. Landed is carried on this stream as well (FLAG_LANDED):
   * the nav page chips read it from either packet type, and updating it
   * here keeps LANDED from flapping between the FUS stream (sets it) and
   * the raw GPS stream (would otherwise clear it every 5 s). */
  parsed_gps_data.launch_detected = (flags & FLAG_LAUNCH_DETECTED) ? 1 : 0;
  parsed_gps_data.fused_landed    = (flags & FLAG_LANDED)          ? 1 : 0;
  parsed_gps_data.fix = flags & FLAG_FIX_TYPE_MASK;  /* Bits 3-0: GPS fix type */

  /* Clear fused metadata - this packet is a raw GPS fix, not EKF output.
   *
   * NOTE: we deliberately do NOT clear v_north / v_east / v_down here.
   * Those are set only by FUSED packets and represent the last known
   * velocity from the TX-side EKF. Keeping them across GPS packets lets the
   * navigation display keep showing the last good ground speed instead of
   * flickering to 0.0 m/s each time a raw GPS packet lands. */
  parsed_gps_data.is_fused          = 0;
  parsed_gps_data.fused_dr          = 0;
  parsed_gps_data.fused_gps_fresh   = 0;
  parsed_gps_data.fused_imu_healthy = 0;
  /* (fused_landed is deliberately NOT zeroed here: raw GPS packets carry
   * their own FLAG_LANDED, parsed above, so both streams drive it.) */
  parsed_gps_data.fused_age_ds      = 0;

  /* Mark data as ready */
  parsed_data_ready = 1;
  parse_successes++;

  return RF_PARSER_OK;
}

/**
 * @brief Parse a PACKET_TYPE_FUSED payload (21 bytes)
 *
 * Layout (little-endian):
 *   [0]      packet_type (0x04)
 *   [1..4]   latitude  (int32, deg*1e7)
 *   [5..8]   longitude (int32, deg*1e7)
 *   [9..12]  altitude  (int32, centimeters MSL)
 *   [13..14] v_n       (int16, cm/s)
 *   [15..16] v_e       (int16, cm/s)
 *   [17..18] v_d       (int16, cm/s)
 *   [19]     age_ds    (uint8, deciseconds since TX-side last GPS fix)
 *   [20]     flags     (FUSED_FLAG_*)
 *
 * Populates parsed_gps_data with decoded values, including the v_north /
 * v_east / v_down m/s fields and the is_fused / fused_* metadata so the UI
 * layer can distinguish fused-position packets from raw GPS packets.
 */
uint8_t RF_Parser_ParseFusedPacket(const uint8_t *data, uint16_t length)
{
  parse_attempts++;

  if (data == NULL || length != FUSED_PACKET_SIZE) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }
  if (data[0] != PACKET_TYPE_FUSED) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }

  /* Manual little-endian decode (same pattern as binary GPS parser for
   * portability across alignment-picky targets). */
  int32_t lat_enc  = (int32_t)((uint32_t)data[1]  | ((uint32_t)data[2]  << 8)
                             | ((uint32_t)data[3]  << 16) | ((uint32_t)data[4]  << 24));
  int32_t lon_enc  = (int32_t)((uint32_t)data[5]  | ((uint32_t)data[6]  << 8)
                             | ((uint32_t)data[7]  << 16) | ((uint32_t)data[8]  << 24));
  int32_t alt_cm   = (int32_t)((uint32_t)data[9]  | ((uint32_t)data[10] << 8)
                             | ((uint32_t)data[11] << 16) | ((uint32_t)data[12] << 24));
  int16_t v_n_cms  = (int16_t)((uint16_t)data[13] | ((uint16_t)data[14] << 8));
  int16_t v_e_cms  = (int16_t)((uint16_t)data[15] | ((uint16_t)data[16] << 8));
  int16_t v_d_cms  = (int16_t)((uint16_t)data[17] | ((uint16_t)data[18] << 8));
  uint8_t age_ds   = data[19];
  uint8_t flags    = data[20];

  double lat = (double)lat_enc / 10000000.0;
  double lon = (double)lon_enc / 10000000.0;

  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
    parse_failures++;
    return RF_PARSER_ERROR;
  }

  parsed_gps_data.latitude  = (float)lat;
  parsed_gps_data.longitude = (float)lon;
  parsed_gps_data.altitude  = (float)alt_cm * 0.01f;  /* cm -> m */
  parsed_gps_data.v_north   = (float)v_n_cms * 0.01f; /* cm/s -> m/s */
  parsed_gps_data.v_east    = (float)v_e_cms * 0.01f;
  parsed_gps_data.v_down    = (float)v_d_cms * 0.01f;

  /* Satellites aren't carried in fused packets; leave whatever was last
   * seen.
   *
   * Do NOT overwrite `fix` here. `fix` is the GPS-fix quality (0=NoFix,
   * 1=2D, 2/3=3D) and is only meaningful for raw GPS packets - it is the
   * field that drives the "B:3D" / "B:NoFix" label on the navigation
   * display. If we clobber it with GPS_FRESH every FUSED packet, the
   * display flaps between "3D" (from the last GPS) and "2D"/"NoFix" (from
   * alternating FUSED packets), even though the beacon hasn't actually
   * lost fix.
   *
   * The fused-packet state the UI needs is already published separately
   * below (is_fused / fused_dr / fused_gps_fresh / fused_age_ds), so
   * navigation_mode.c can render "FUS" vs "DR" without touching `fix`. */
  parsed_gps_data.launch_detected = (flags & FUSED_FLAG_LAUNCH_DETECTED) ? 1 : 0;

  parsed_gps_data.is_fused          = 1;
  parsed_gps_data.fused_dr          = (flags & FUSED_FLAG_DEAD_RECKONING) ? 1 : 0;
  parsed_gps_data.fused_gps_fresh   = (flags & FUSED_FLAG_GPS_FRESH)      ? 1 : 0;
  parsed_gps_data.fused_imu_healthy = (flags & FUSED_FLAG_IMU_HEALTHY)    ? 1 : 0;
  parsed_gps_data.fused_landed      = (flags & FUSED_FLAG_LANDED)         ? 1 : 0;
  parsed_gps_data.fused_age_ds      = age_ds;

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
  parse_failures_null_packet = 0;
  parse_failures_insufficient_fields = 0;
}
