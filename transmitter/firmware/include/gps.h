#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include "uart.h"         // Include the hardware UART header
#include "mpu_config.h" // Include hardware configuration

// Field indices in RMC sentence
#define RMC_FIELD_STATUS    2
#define RMC_FIELD_LAT       3
#define RMC_FIELD_LAT_DIR   4
#define RMC_FIELD_LON       5
#define RMC_FIELD_LON_DIR   6

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
uint16_t gps_poll_rx(void);
const GPSCoordinates_t* gps_get_current_coordinates(void);

// GPS receiver health for the heartbeat (HB_GPS_* in packet_format.h):
// distinguishes "still acquiring" from "UART silent" / "data garbled".
uint8_t gps_get_health(void);

/* Main loop, every pass: nonzero during LAUNCH/POST_LAUNCH. In flight the
 * GPS watchdog re-sends settings but never resets the receiver; when the
 * flight ends the reset ladder (hot -> warm -> cold) starts fresh. */
void gps_set_in_flight(uint8_t in_flight);

// Milliseconds since the last accepted fix; UINT32_MAX if never had one.
uint32_t gps_get_fix_age_ms(void);

// Convert NMEA coordinate to decimal degrees
double gps_nmea_to_decimal(const char* nmea_coord, char direction);


#endif // GPS_H
