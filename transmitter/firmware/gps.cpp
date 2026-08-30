#include "include/gps.h"
#include "include/nmea_fields.h"
#include "include/packet_format.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "include/uart.h"
#include "include/radio.h"
#include "include/launch_detect.h"
#include "include/beacon.h"
#include "include/nav.h"

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

/* GPS-health bookkeeping for the heartbeat's gps_health byte. Timestamps of
 * 0 mean "never since boot", which millis()-0 turns into the boot age - so a
 * GPS that was dead from power-on is reported without any special casing.
 * gps_recovery_attempts is shared with the watchdog in gps_poll_rx(). */
static uint32_t gps_last_byte_ms = 0;      // last UART byte received
static uint32_t gps_last_sentence_ms = 0;  // last recognizable GGA/RMC
static uint8_t  gps_recovery_attempts = 0; // watchdog resets issued

/* Grace periods before declaring a fault. The module emits NMEA in 1 Hz
 * bursts, so multi-second silence is already abnormal; kept well below the
 * 5 s heartbeat interval so the very first heartbeat carries a verdict. */
#define GPS_HEALTH_SILENT_MS   3000UL   // no bytes at all -> wiring/power
#define GPS_HEALTH_NO_NMEA_MS  5000UL   // bytes but no GGA/RMC -> baud/noise

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

/**
 * Report GPS receiver health for the no-fix heartbeat.
 *
 * Distinguishes the three no-fix situations an operator at the pad cannot
 * tell apart from "sats: 0" alone:
 *   HB_GPS_NO_DATA   - UART totally silent: broken wire, no power, dead module
 *   HB_GPS_NO_NMEA   - bytes arriving but nothing parses: baud mismatch/noise
 *   HB_GPS_ACQUIRING - sentences flowing, just no fix yet: be patient
 * The high nibble carries the watchdog's recovery-reset count (0-3).
 */
uint8_t gps_get_health(void) {
    uint32_t now = millis();
    uint8_t state;
    if (now - gps_last_byte_ms > GPS_HEALTH_SILENT_MS) {
        state = HB_GPS_NO_DATA;
    } else if (now - gps_last_sentence_ms > GPS_HEALTH_NO_NMEA_MS) {
        state = HB_GPS_NO_NMEA;
    } else {
        state = HB_GPS_ACQUIRING;
    }
    uint8_t resets = (gps_recovery_attempts > 15) ? 15 : gps_recovery_attempts;
    return HB_GPS_HEALTH(state, resets);
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
        gps_last_byte_ms = current_time;   // health: UART is alive
    }
    
    // Track when we last had a GOOD fix: quality >= 1 AND at least one
    // satellite tracked AND bytes actually flowing right now. This is what
    // the watchdog keys on - NOT current_coords.valid, which latching (any
    // sentence with lat/lon, even a stale cached hot-start position with
    // fix_quality 0) would otherwise disarm recovery forever. The byte
    // freshness check matters too: during total UART silence no sentence
    // arrives to unlatch valid, so without it a stale good fix would
    // cancel every silent-UART recovery attempt one poll later.
    // Field evidence 2026-08-30: TX hot-started with a cached position,
    // valid latched 1, sats dropped to 0, and the watchdog sat disarmed
    // for 20 minutes (rst=0 in every heartbeat) while the user power-
    // cycled to no avail.
    uint32_t no_bytes_duration = current_time - last_byte_time;
    uint8_t good_fix = current_coords.valid
                    && current_coords.fix_quality >= 1
                    && atoi(current_coords.satellites) > 0
                    && no_bytes_duration <= GPS_HEALTH_SILENT_MS;
    if (good_fix) {
        last_valid_time = current_time;
        gps_recovery_attempts = 0;  // Reset recovery counter on success
    }
    
    // Check for GPS hang conditions
    uint32_t no_valid_duration = current_time - last_valid_time;
    
    // Condition 1: No UART data for 30 seconds = GPS UART is dead
    if (no_bytes_duration > 30000 && gps_recovery_attempts < 3) {
        Serial.println(F("[GPS] ⚠️  Watchdog: No UART data for 30s, attempting recovery..."));
        gps_recovery_attempts++;
        
        // Recovery: controlled software reset (cold start). Do NOT use
        // the factory-defaults variant ($PUBX,04,0,0,0,...) here: it
        // reverts UART1 to 9600 baud and we talk at 115200 - recovery
        // would permanently deafen the link it was meant to restore.
        uart_tx_string("$PUBX,00*33\r\n");  // u-blox poll request
        delay(100);
        uart_tx_string("$PUBX,04,0,0,2,0,0*12\r\n");  // u-blox cold start
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
    
    // After 3 failed recovery attempts, cool down instead of giving up
    // forever: "power cycle required" is not an option on a rocket in a
    // tree. A 10-minute pause avoids a cold-restart loop that would keep
    // discarding the almanac, then recovery re-arms for another round.
    // (Re-arming zeroes the HB_GPS_RESETS nibble in the heartbeat; the
    // receiver just sees the count restart, which is fine.)
    static uint32_t recovery_exhausted_ms = 0;
    if (gps_recovery_attempts >= 3) {
        if (recovery_exhausted_ms == 0) {
            recovery_exhausted_ms = current_time;
        } else if (current_time - recovery_exhausted_ms > 600000UL) {
            Serial.println(F("[GPS] Watchdog: cooldown over, re-arming recovery"));
            gps_recovery_attempts = 0;
            recovery_exhausted_ms = 0;
            last_byte_time = current_time;   // full grace period before retry
            last_valid_time = current_time;
        }
        if (current_time - last_debug > 30000) {
            Serial.println(F("[GPS] ✗ Recovery failed after 3 attempts - cooling down before retry"));
        }
    } else {
        recovery_exhausted_ms = 0;
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
        
        // Check if this is a GGA or RMC sentence (support both GP and GN prefixes)
        if (strncmp(nmea_buffer, "$GNGGA", 6) == 0 || 
                 strncmp(nmea_buffer, "$GNRMC", 6) == 0 ||
                 strncmp(nmea_buffer, "$GPGGA", 6) == 0 || 
                 strncmp(nmea_buffer, "$GPRMC", 6) == 0) {
            /* Health: a recognizable NMEA sentence arrived, fix or not.
             * Recorded BEFORE any validity checks - "the GPS is talking
             * sense" is exactly what distinguishes a cold module still
             * acquiring from a broken wire or a baud-rate mismatch. */
            gps_last_sentence_ms = millis();
            uint8_t is_rmc = (nmea_buffer[3] == 'R'); // Check if RMC

            /* Extract fields by ABSOLUTE position (see nmea_fields.h).
             * The old strtok_r loop collapsed empty fields, so a partial
             * fix (e.g. "$GPGGA,123519,,,,,0,00,...") shifted every later
             * field into the wrong index. nmea_get_field preserves empty
             * fields and cannot misalign. */
            char fbuf[16];

            // For RMC sentences, check status field first (field 2)
            if (is_rmc) {
                if (nmea_get_field(nmea_buffer, RMC_FIELD_STATUS,
                                   fbuf, sizeof(fbuf)) < 1 || fbuf[0] != 'A') {
                    /* Status V = receiver says NO FIX right now. Clear the
                     * latched valid flag: a stale hot-start position must
                     * not masquerade as a live fix (it previously stayed
                     * latched forever, hiding total signal loss). */
                    current_coords.valid = 0;
                    return bytes_processed;  // Skip invalid RMC (status = V)
                }
            }

            // Field positions per sentence type
            // GGA: time=1 lat=2 N/S=3 lon=4 E/W=5 quality=6 sats=7 hdop=8 alt=9
            // RMC: time=1 status=2 lat=3 N/S=4 lon=5 E/W=6 ...
            const uint8_t lat_idx     = is_rmc ? RMC_FIELD_LAT     : 2;
            const uint8_t lat_dir_idx = is_rmc ? RMC_FIELD_LAT_DIR : 3;
            const uint8_t lon_idx     = is_rmc ? RMC_FIELD_LON     : 4;
            const uint8_t lon_dir_idx = is_rmc ? RMC_FIELD_LON_DIR : 5;

            // Latitude (keep previous value if the field is empty)
            if (nmea_get_field(nmea_buffer, lat_idx, fbuf, sizeof(fbuf)) > 0) {
                strncpy(lat_buffer, fbuf, sizeof(lat_buffer) - 1);
                lat_buffer[sizeof(lat_buffer) - 1] = '\0';
            }

            // Latitude direction (N/S)
            if (nmea_get_field(nmea_buffer, lat_dir_idx, fbuf, sizeof(fbuf)) > 0
                && (fbuf[0] == 'N' || fbuf[0] == 'S')) {
                lat_dir = fbuf[0];
            }

            // Longitude (keep previous value if the field is empty)
            if (nmea_get_field(nmea_buffer, lon_idx, fbuf, sizeof(fbuf)) > 0) {
                strncpy(lon_buffer, fbuf, sizeof(lon_buffer) - 1);
                lon_buffer[sizeof(lon_buffer) - 1] = '\0';
            }

            // Longitude direction (E/W) - also commits the coordinate pair
            if (nmea_get_field(nmea_buffer, lon_dir_idx, fbuf, sizeof(fbuf)) > 0
                && (fbuf[0] == 'E' || fbuf[0] == 'W')) {
                lon_dir = fbuf[0];

                // We have lat/lon data, update current coordinates
                if (lat_buffer[0] != '\0' && lon_buffer[0] != '\0') {
                    strncpy(current_coords.lat, lat_buffer,
                            sizeof(current_coords.lat) - 1);
                    current_coords.lat[sizeof(current_coords.lat) - 1] = '\0';

                    strncpy(current_coords.lon, lon_buffer,
                            sizeof(current_coords.lon) - 1);
                    current_coords.lon[sizeof(current_coords.lon) - 1] = '\0';

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

            // GGA-only fields
            if (!is_rmc) {
                // Fix quality (field 6): 0=invalid, 1=GPS, 2=DGPS, 3=PPS, ...
                if (nmea_get_field(nmea_buffer, 6, fbuf, sizeof(fbuf)) > 0) {
                    current_coords.fix_quality = atoi(fbuf);
                    /* Quality 0 = no fix: unlatch. valid is set again by
                     * the lon_dir branch above when a real position
                     * sentence arrives. */
                    if (current_coords.fix_quality == 0) {
                        current_coords.valid = 0;
                    }
                }

                // Number of satellites (field 7)
                if (nmea_get_field(nmea_buffer, 7, fbuf, sizeof(fbuf)) > 0) {
                    strncpy(current_coords.satellites, fbuf,
                            sizeof(current_coords.satellites) - 1);
                    current_coords.satellites[sizeof(current_coords.satellites) - 1] = '\0';
                }

                // Altitude (field 9)
                if (nmea_get_field(nmea_buffer, 9, fbuf, sizeof(fbuf)) > 0) {
                    strncpy(current_coords.altitude, fbuf,
                            sizeof(current_coords.altitude) - 1);
                    current_coords.altitude[sizeof(current_coords.altitude) - 1] = '\0';

                    /* Feed the EKF with the freshly parsed GGA fix.  GGA
                     * provides fix_quality (field 6), sats (field 7), and
                     * altitude (field 9); lat/lon came from an earlier GGA
                     * field or the preceding RMC.  We only update when the
                     * fix looks plausible; nav_update_from_gps also rejects
                     * low sat counts / no fix internally. */
                    if (current_coords.valid
                        && current_coords.fix_quality >= 1
                        && current_coords.lat[0] != '0'
                        && current_coords.lat[0] != '\0') {
                        float lat = gps_nmea_to_decimal(current_coords.lat,
                                                       current_coords.lat_dir);
                        float lon = gps_nmea_to_decimal(current_coords.lon,
                                                       current_coords.lon_dir);
                        float alt = atof(current_coords.altitude);
                        uint8_t sats = (uint8_t)atoi(current_coords.satellites);
                        nav_update_from_gps(lat, lon, alt,
                                            sats, current_coords.fix_quality);
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
    /* Thin wrapper: the conversion lives in nmea_fields.c so it can be
     * unit-tested on the host (and shares the double-precision math). */
    return nmea_coord_to_decimal(nmea_coord, direction);
}

void gps_tx_string(const char* str) {
    // Send commands to GPS module
    if (str) {
        while (*str) {
            uart_tx_byte(*str++);
        }
    }
}
