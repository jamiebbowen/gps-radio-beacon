#include "include/gps.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "include/uart.h"
#include "include/radio.h"
#include "include/launch_detect.h"
#include "include/beacon.h"

// Global variables
volatile uint8_t gpsInitialized = 0;

// Global coordinates storage
static GPSCoordinates_t current_coords = {
    "0",    // lat
    "0",    // lon
    'N',    // lat_dir
    'E',    // lon_dir
    0,      // valid
    0,      // fix_quality
    "0",    // satellites
    "0"     // altitude
};

// Define states for NMEA parsing state machine
typedef enum {
    NMEA_WAIT_FOR_START,    // Waiting for $ character
    NMEA_READ_TYPE,         // Reading sentence type (GNGGA, GNRMC)
    NMEA_SKIP_TO_LAT,       // Skip to latitude field
    NMEA_READ_LAT,          // Reading latitude value
    NMEA_READ_LAT_DIR,      // Reading N/S indicator
    NMEA_SKIP_TO_LON,       // Skip to longitude field
    NMEA_READ_LON,          // Reading longitude value
    NMEA_READ_LON_DIR,      // Reading E/W indicator
    NMEA_WAIT_FOR_END       // Wait for end of sentence
} NMEAParseState;

// Initialize GPS for u-blox MAX-M10S
void gps_init(void) {
    // Basic initialization
    gpsInitialized = 1;
    
    // Allow more time for GPS to boot fully
    delay(10);
    
    // For u-blox MAX-M10S, configure NMEA protocol
    // NMEA Standard messages: GGA, RMC, GSA, GSV, GLL, VTG
    // Enable only NMEA GSV, GGA, and RMC messages
    uart_tx_string("$PUBX,40,GGA,1,1,0,0,0,0*5B\r\n"); // Enable GGA on UART1
    delay(10);
    uart_tx_string("$PUBX,40,GSV,1,1,0,0,0,0*5A\r\n"); // Enable GSV for satellite info
    delay(10);
    
    // Configure other NMEA messages
    uart_tx_string("$PUBX,40,GSA,1,0,0,0,0,0*4E\r\n"); // Disable GSA
    delay(10); 
    uart_tx_string("$PUBX,40,RMC,1,1,0,0,0,0*25\r\n"); // Enable RMC on UART1
    delay(10);
    uart_tx_string("$PUBX,40,GLL,1,0,0,0,0,0*5C\r\n"); // Disable GLL
    delay(10);
    uart_tx_string("$PUBX,40,VTG,1,0,0,0,0,0*5E\r\n"); // Disable VTG
    delay(10);
    
    // Configure update rate to 1Hz
    uart_tx_string("$PUBX,40,00,0,1,0,0,0,0*A7\r\n");
    delay(10);
}

// Poll for GPS data and extract lat/lon using buffer-based approach
// Returns number of bytes processed, 0 if none
uint8_t gps_poll_rx(void) {
    uint16_t bytes_processed = 0;
    static char nmea_buffer[84]; // NMEA spec: max 80 characters per sentence plus a few extra
    static char lat_buffer[12] = {0};
    static char lon_buffer[13] = {0};  // Increased for 11 chars + null terminator
    static char lat_dir = 'N';
    static char lon_dir = 'E';
    static uint16_t wait_timeout = 0;
    static uint32_t last_debug = 0;
    static uint32_t byte_count = 0;
    static uint32_t last_byte_count = 0;
    static uint32_t last_valid_time = 0;
    static uint32_t last_byte_time = 0;
    static uint8_t gps_recovery_attempts = 0;
    static uint8_t watchdog_initialized = 0;

    flush_uart_buffer();
    
    // GPS Watchdog: Detect and recover from GPS hangs
    uint32_t current_time = millis();
    
    // Initialize watchdog timers on first call
    if (!watchdog_initialized) {
        last_byte_time = current_time;
        last_valid_time = current_time;
        watchdog_initialized = 1;
    }
    
    // Track when we last received data
    if (byte_count != last_byte_count) {
        last_byte_time = current_time;
        last_byte_count = byte_count;
    }
    
    // Track when we last had valid GPS
    if (current_coords.valid) {
        last_valid_time = current_time;
        gps_recovery_attempts = 0;  // Reset recovery counter on success
    }
    
    // Check for GPS hang conditions
    uint32_t no_bytes_duration = current_time - last_byte_time;
    uint32_t no_valid_duration = current_time - last_valid_time;
    
    // Condition 1: No UART data for 30 seconds = GPS UART is dead
    if (no_bytes_duration > 30000 && gps_recovery_attempts < 3) {
        Serial.println(F("[GPS] ⚠️  Watchdog: No UART data for 30s, attempting recovery..."));
        gps_recovery_attempts++;
        
        // Recovery: Send GPS software reset command
        uart_tx_string("$PUBX,00*33\r\n");  // u-blox poll request
        delay(100);
        uart_tx_string("$PUBX,04,0,0,0,0,0*10\r\n");  // u-blox reset to factory defaults
        delay(500);
        
        // Reinitialize GPS
        gps_init();
        Serial.println(F("[GPS] Recovery attempt complete"));
        
        // Reset watchdog timer
        last_byte_time = current_time;
    }
    
    // Condition 2: Receiving bytes but no valid fix for 60 seconds = GPS is stuck
    else if (byte_count > 100 && no_valid_duration > 60000 && gps_recovery_attempts < 3) {
        Serial.println(F("[GPS] ⚠️  Watchdog: No valid fix for 60s, attempting recovery..."));
        gps_recovery_attempts++;
        
        // Recovery: Cold restart GPS
        uart_tx_string("$PUBX,04,0,0,2,0,0*12\r\n");  // u-blox controlled software reset (cold start)
        delay(1000);
        
        // Reinitialize GPS
        gps_init();
        Serial.println(F("[GPS] Recovery attempt complete"));
        
        // Reset watchdog timer
        last_valid_time = current_time;
    }
    
    // Give up after 3 recovery attempts
    if (gps_recovery_attempts >= 3 && current_time - last_debug > 30000) {
        Serial.println(F("[GPS] ✗ Recovery failed after 3 attempts - power cycle required"));
    }
    
    // Basic GPS debug every 5 seconds
    if (millis() - last_debug > 5000) {
        Serial.print(F("[GPS] Bytes rcvd: "));
        Serial.print(byte_count);
        Serial.print(F(" | Valid: "));
        Serial.println(current_coords.valid);
        last_debug = millis();
    }
      
    // First, wait for a '$' character to start a sentence
    while (1) {
        if (!uart_data_available()) {
            delayMicroseconds(10);
            wait_timeout++;
            if (wait_timeout > 5000) { // Timeout after ~50ms
                return bytes_processed;
            }
            continue;
        }
        
        uint8_t byte = uart_read_byte();
        bytes_processed++;
        byte_count++;  // Track total bytes for debug
        wait_timeout = 0;
        
        // Skip framing errors
        if (byte == 0xFE || byte == 0xFF) {
            continue;
        }
        
        // Found start of sentence
        if (byte == '$') {
            nmea_buffer[0] = byte;
            break;
        }
    }
    
    // Now read the rest of the sentence into the buffer
    uint8_t buffer_index = 1;
    uint8_t sentence_complete = 0;
    uint16_t sentence_timeout = 0;
    
    while (buffer_index < sizeof(nmea_buffer) - 1) {
        // Read data as fast as possible when available
        while (uart_data_available() && buffer_index < sizeof(nmea_buffer) - 1) {
            uint8_t byte = uart_read_byte();
            
            // Skip framing errors
            if (byte == 0xFE || byte == 0xFF) {
                continue;
            }
            
            nmea_buffer[buffer_index] = byte;
            buffer_index++;
            bytes_processed++;

            // Check for end of sentence
            if (byte == '\n') {
                sentence_complete = 1;
                break;
            }
        }
        
        if (sentence_complete) {
            break;
        }
    }
    
    // Null terminate the buffer
    nmea_buffer[buffer_index] = '\0';
    
    // Only process if we have a complete sentence
    if (sentence_complete) {
        // Debug: Show sentence type every 10 seconds
        static uint32_t last_sentence_debug = 0;
        if (millis() - last_sentence_debug > 10000) {
            Serial.print(F("[GPS] Sentence: "));
            Serial.write((uint8_t*)nmea_buffer, min(20, (int)strlen(nmea_buffer)));
            Serial.println();
            last_sentence_debug = millis();
        }
        
        char *token;
        char *saveptr;
        uint8_t field_index = 0;

        // Check if this is a GGA or RMC sentence (support both GP and GN prefixes)
        if (strncmp(nmea_buffer, "$GNGGA", 6) == 0 || 
                 strncmp(nmea_buffer, "$GNRMC", 6) == 0 ||
                 strncmp(nmea_buffer, "$GPGGA", 6) == 0 || 
                 strncmp(nmea_buffer, "$GPRMC", 6) == 0) {
            uint8_t is_rmc = (nmea_buffer[3] == 'R'); // Check if RMC
            
            // Parse GPS sentence
            token = strtok_r(nmea_buffer, ",", &saveptr);
            
            // For RMC sentences, check status field first (field 2)
            uint8_t rmc_valid = 0;
            if (is_rmc) {
                // Skip field 1 (time)
                token = strtok_r(NULL, ",", &saveptr);
                // Get field 2 (status)
                token = strtok_r(NULL, ",", &saveptr);
                if (token != NULL && token[0] == 'A') {
                    rmc_valid = 1;  // Status = Active/Valid
                    field_index = 2;  // We've consumed 2 fields
                } else {
                    return bytes_processed;  // Skip invalid RMC (status = V)
                }
            }

            // Process remaining tokens
            while ((token = strtok_r(NULL, ",", &saveptr)) != NULL) {
                field_index++;
                
                // Latitude field
                // GGA: field 2, RMC: field 3
                if ((is_rmc && field_index == 3) || (!is_rmc && field_index == 2)) {
                    if (token != NULL && strlen(token) > 0) {
                        uint8_t token_len = strlen(token);
                        uint8_t max_copy = sizeof(lat_buffer) - 1;  // Always leave room for null terminator
                        uint8_t copy_len = (token_len < max_copy) ? token_len : max_copy;
                        strncpy(lat_buffer, token, copy_len);
                        lat_buffer[copy_len] = '\0';
                    }
                }
                // Latitude direction (N/S)
                else if ((is_rmc && field_index == 4) || (!is_rmc && field_index == 3)) {
                    if (token[0] == 'N' || token[0] == 'S') {
                        lat_dir = token[0];
                    }
                }
                // Longitude field
                // GGA: field 4, RMC: field 5
                else if ((is_rmc && field_index == 5) || (!is_rmc && field_index == 4)) {
                    if (token != NULL && strlen(token) > 0) {
                        uint8_t token_len = strlen(token);
                        uint8_t max_copy = sizeof(lon_buffer) - 1;  // Always leave room for null terminator
                        uint8_t copy_len = (token_len < max_copy) ? token_len : max_copy;
                        strncpy(lon_buffer, token, copy_len);
                        lon_buffer[copy_len] = '\0';
                    }
                }
                // Longitude direction (E/W)
                else if ((is_rmc && field_index == 6) || (!is_rmc && field_index == 5)) {
                    if (token[0] == 'E' || token[0] == 'W') {
                        lon_dir = token[0];
                        
                        // We have lat/lon data, update current coordinates
                        if (lat_buffer[0] != '\0' && lon_buffer[0] != '\0') {
                            uint8_t lat_len = strlen(lat_buffer);
                            uint8_t max_copy_lat = sizeof(current_coords.lat) - 1;  // Always leave room for null terminator
                            uint8_t copy_len_lat = (lat_len < max_copy_lat) ? lat_len : max_copy_lat;
                            strncpy(current_coords.lat, lat_buffer, copy_len_lat);
                            current_coords.lat[copy_len_lat] = '\0';
                            
                            uint8_t lon_len = strlen(lon_buffer);
                            uint8_t max_copy_lon = sizeof(current_coords.lon) - 1;  // Always leave room for null terminator
                            uint8_t copy_len_lon = (lon_len < max_copy_lon) ? lon_len : max_copy_lon;
                            strncpy(current_coords.lon, lon_buffer, copy_len_lon);
                            current_coords.lon[copy_len_lon] = '\0';
                            
                            current_coords.lat_dir = lat_dir;
                            current_coords.lon_dir = lon_dir;
                            current_coords.valid = 1;
                            
                            // Coordinate update (log less frequently)
                            static uint32_t last_coord_log = 0;
                            if (millis() - last_coord_log > 10000) {  // Every 10 seconds
                                Serial.print(F("[GPS] ✓ Fix: "));
                                Serial.print(current_coords.lat);
                                Serial.print(current_coords.lat_dir);
                                Serial.print(F(", "));
                                Serial.print(current_coords.lon);
                                Serial.println(current_coords.lon_dir);
                                last_coord_log = millis();
                            }
                        }
                    }
                }
                // Fix quality (only in GGA, field 6)
                // 0=invalid, 1=GPS fix, 2=DGPS fix, 3=PPS fix, etc.
                else if (!is_rmc && field_index == 6) {
                    if (token != NULL && strlen(token) > 0) {
                        current_coords.fix_quality = atoi(token);
                    }
                }
                // Number of satellites (only in GGA, field 7)
                else if (!is_rmc && field_index == 7) {
                    if (token != NULL && strlen(token) > 0) {
                        uint8_t token_len = strlen(token);
                        uint8_t max_copy = sizeof(current_coords.satellites) - 1;
                        uint8_t copy_len = (token_len < max_copy) ? token_len : max_copy;
                        strncpy(current_coords.satellites, token, copy_len);
                        current_coords.satellites[copy_len] = '\0';
                    }
                }
                // Altitude (only in GGA, field 9)
                // GGA format: $GNGGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,altitude,M,...
                else if (!is_rmc && field_index == 9) {
                    if (token != NULL && strlen(token) > 0) {
                        uint8_t token_len = strlen(token);
                        uint8_t max_copy = sizeof(current_coords.altitude) - 1;
                        uint8_t copy_len = (token_len < max_copy) ? token_len : max_copy;
                        strncpy(current_coords.altitude, token, copy_len);
                        current_coords.altitude[copy_len] = '\0';
                    }
                }
            }
        }
    }
    
    return bytes_processed;
}

/**
 * Get pointer to current GPS coordinates structure
 * @return Pointer to current GPS coordinates (read-only)
 */
const GPSCoordinates_t* gps_get_current_coordinates(void) {
    return &current_coords;
}

/**
 * @brief Convert NMEA coordinate string to decimal degrees
 * @param nmea_coord NMEA coordinate string (e.g., "3953.40284" or "10506.93605")
 * @param direction Direction character ('N', 'S', 'E', 'W')
 * @return Decimal degrees (negative for S/W)
 */
float gps_nmea_to_decimal(const char* nmea_coord, char direction) {
    if (nmea_coord == NULL || strlen(nmea_coord) == 0) {
        return 0.0f;
    }
    
    // Convert entire coordinate to float
    float nmea_value = atof(nmea_coord);
    
    // Extract degrees (everything before last 2 digits before decimal)
    int degrees = (int)(nmea_value / 100.0f);
    
    // Extract minutes (last 2 digits before decimal + fraction)
    float minutes = nmea_value - (degrees * 100.0f);
    
    // Convert to decimal degrees
    float decimal = (float)degrees + (minutes / 60.0f);
    
    // Apply sign based on direction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

void gps_tx_string(const char* str) {
    // Send commands to GPS module
    if (str) {
        while (*str) {
            uart_tx_byte(*str++);
        }
    }
}
