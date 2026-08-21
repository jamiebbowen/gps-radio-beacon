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
#define PACKET_TYPE_FUSED       0x04  // EKF-fused position + velocity
#define PACKET_TYPE_HEARTBEAT   0x05  // No-fix keepalive (see HeartbeatPacket_t)

/* Heartbeat: 7 bytes. Sent instead of a GPS packet when no transmittable fix
 * exists (no fix / <4 sats), so the receiver's channel scan can lock and the
 * operator can see the beacon is alive on the pad. Must stay in sync with
 * the receiver's copy of this header. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;   // PACKET_TYPE_HEARTBEAT
    uint8_t  rocket_id;     // ROCKET_ID of this airframe
    uint8_t  channel;       // Active (jumper-resolved) channel
    uint8_t  satellites;    // Satellites currently tracked
    uint8_t  fix_quality;   // GGA fix quality (0 = none)
    uint16_t uptime_s;      // Seconds since beacon boot (saturates at 65535)
} HeartbeatPacket_t;
#define HEARTBEAT_PACKET_SIZE   7

// Flags byte bit definitions
#define FLAG_LAUNCH_DETECTED    0x80  // Bit 7: 1 = launched, 0 = on ground
#define FLAG_FIX_QUALITY_GOOD   0x40  // Bit 6: 1 = good fix, 0 = poor
#define FLAG_FIX_TYPE_MASK      0x0F  // Bits 3-0: GPS fix type (0=none, 1=GPS, 2=DGPS, etc.)

/* FusedPosPacket_t flags (separate bitmap from the GPS packet flags) */
#define FUSED_FLAG_LAUNCH_DETECTED   0x80  // Bit 7: 1 = launched
#define FUSED_FLAG_GPS_FRESH         0x40  // Bit 6: 1 = GPS fix used within the last second
#define FUSED_FLAG_IMU_HEALTHY       0x20  // Bit 5: 1 = BNO085 streaming, not saturated
#define FUSED_FLAG_DEAD_RECKONING    0x10  // Bit 4: 1 = no GPS for > NAV_DR_TIMEOUT_S
#define FUSED_FLAG_RESERVED_MASK     0x0F  // Bits 3-0: reserved

/* Fused packet: 21 bytes. Transmitted with PACKET_TYPE_FUSED.
 *
 *   lat/lon       : same scaling as BinaryGPSPacket_t (deg * 10^7)
 *   altitude_cm   : centimeters MSL (int32 gives ±21 km at 1 cm resolution)
 *   v_n/v_e/v_d   : NED velocity in cm/s (int16 gives ±327 m/s per axis)
 *   age_ds        : deciseconds since last GPS update (0..255 -> 0..25.5 s,
 *                   saturates at 255 meaning "GPS lost long ago")
 *   flags         : FUSED_FLAG_* bits above
 */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FUSED
    int32_t  latitude;       // deg * 10^7
    int32_t  longitude;      // deg * 10^7
    int32_t  altitude_cm;    // cm MSL
    int16_t  v_n_cms;        // North velocity, cm/s
    int16_t  v_e_cms;        // East  velocity, cm/s
    int16_t  v_d_cms;        // Down  velocity, cm/s
    uint8_t  age_ds;         // Deciseconds since last GPS update (saturating)
    uint8_t  flags;          // FUSED_FLAG_*
} FusedPosPacket_t;

// Helper macros for encoding/decoding
#define GPS_COORD_SCALE         10000000.0f  // Scale factor for lat/lon (10^7)

// Convert decimal degrees to encoded integer
#define ENCODE_COORD(deg)       ((int32_t)((deg) * GPS_COORD_SCALE))

// Convert encoded integer back to decimal degrees
#define DECODE_COORD(val)       ((float)(val) / GPS_COORD_SCALE)

#endif // _PACKET_FORMAT_H_
