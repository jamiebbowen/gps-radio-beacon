#ifndef _PACKET_FORMAT_H_
#define _PACKET_FORMAT_H_

#include <stdint.h>

// Packet format selection
#define USE_BINARY_PACKETS  1  // Set to 1 for binary, 0 for ASCII

// Binary GPS packet structure (13 bytes total)
typedef struct __attribute__((packed)) {
    uint8_t packet_type;     // 0x01 = GPS position packet
    int32_t latitude;        // Latitude * 10^7 (e.g., 39.890075° = 398900750)
    int32_t longitude;       // Longitude * 10^7 (e.g., -105.115510° = -1051155100)
    int16_t altitude;        // Altitude in meters (range: -32768 to +32767)
    uint8_t satellites;      // Number of satellites (0-255)
    uint8_t flags;          // Status flags (see below)
} BinaryGPSPacket_t;

// Packet types
#define PACKET_TYPE_GPS         0x01
#define PACKET_TYPE_CALLSIGN    0x02  // Future use
#define PACKET_TYPE_TELEMETRY   0x03  // Future use

// Flags byte bit definitions
#define FLAG_LAUNCH_DETECTED    0x80  // Bit 7: 1 = launched, 0 = on ground
#define FLAG_FIX_QUALITY_GOOD   0x40  // Bit 6: 1 = good fix, 0 = poor
#define FLAG_FIX_TYPE_MASK      0x0F  // Bits 3-0: GPS fix type (0=none, 1=GPS, 2=DGPS, etc.)

// Helper macros for encoding/decoding
#define GPS_COORD_SCALE         10000000.0f  // Scale factor for lat/lon (10^7)

// Convert decimal degrees to encoded integer
#define ENCODE_COORD(deg)       ((int32_t)((deg) * GPS_COORD_SCALE))

// Convert encoded integer back to decimal degrees
#define DECODE_COORD(val)       ((float)(val) / GPS_COORD_SCALE)

#endif // _PACKET_FORMAT_H_
