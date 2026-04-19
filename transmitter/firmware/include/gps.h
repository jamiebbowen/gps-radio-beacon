#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include "uart.h"         // Include the hardware UART header
#include "mpu_config.h" // Include hardware configuration

// Pin definitions for GPS are now in mpu_config.h
// USART0_RX_PIN and USART0_TX_PIN

// Maximum size of a raw GNRMC sentence buffer
// This is intentionally small to fit in limited RAM
// We'll use incremental parsing to handle larger sentences
#define MAX_GNRMC_SIZE 32

// Field indices in RMC sentence
#define RMC_FIELD_TIME      1
#define RMC_FIELD_STATUS    2
#define RMC_FIELD_LAT       3
#define RMC_FIELD_LAT_DIR   4
#define RMC_FIELD_LON       5
#define RMC_FIELD_LON_DIR   6

// Parser state machine values
#define NMEA_PARSER_IDLE        0
#define NMEA_PARSER_IN_SENTENCE 1
#define NMEA_PARSER_IN_RMC      2 
#define NMEA_PARSER_IN_CHECKSUM 3

// GPS data - now we'll store direct fields instead of the full sentence
extern volatile uint8_t gpsInitialized;

// GPS coordinate structure (raw string storage to save flash)
typedef struct {
    char lat[12];        // Raw latitude string from NMEA
    char lon[13];        // Raw longitude string from NMEA (increased for 11 chars + null)
    char lat_dir;        // 'N' or 'S'
    char lon_dir;        // 'E' or 'W'
    uint8_t valid;       // 1 if valid, 0 if not
    uint8_t fix_quality; // GGA fix quality: 0=invalid, 1=GPS, 2=DGPS, 3=PPS, etc.
    char satellites[4];  // Number of satellites in view (string)
    char altitude[8];    // Altitude in meters (string)
} GPSCoordinates_t;

// Public function declarations
void gps_init(void);
uint8_t gps_poll_rx(void);
const GPSCoordinates_t* gps_get_current_coordinates(void);

// Convert NMEA coordinate to decimal degrees
float gps_nmea_to_decimal(const char* nmea_coord, char direction);

// Send data to GPS
void gps_tx_string(const char* str);

#endif // GPS_H
