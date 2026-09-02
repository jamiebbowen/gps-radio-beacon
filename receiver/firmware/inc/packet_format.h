#ifndef _PACKET_FORMAT_H_
#define _PACKET_FORMAT_H_

#include <stdint.h>

// Packet format selection (must match transmitter!)
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
#define PACKET_TYPE_FUSED       0x04  // EKF-fused position + velocity (TX nav module)
#define PACKET_TYPE_HEARTBEAT   0x05  // No-fix keepalive (see HeartbeatPacket_t)

/* Heartbeat: 8 bytes. The transmitter sends this instead of a GPS packet
 * when it has no transmittable fix (no fix / <4 sats), so the channel scan
 * can lock and the operator can see the beacon is alive on the pad. Must
 * stay in sync with the transmitter's copy of this header. The receiver
 * also accepts the older 7-byte layout (no gps_health) from beacons on
 * previous firmware; gps_health then decodes as HB_GPS_UNKNOWN. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;   // PACKET_TYPE_HEARTBEAT
    uint8_t  rocket_id;     // ROCKET_ID of the airframe
    uint8_t  channel;       // Active (jumper-resolved) channel
    uint8_t  satellites;    // Satellites currently tracked
    uint8_t  fix_quality;   // GGA fix quality (0 = none)
    uint16_t uptime_s;      // Seconds since beacon boot (saturates at 65535)
    uint8_t  gps_health;    // GPS receiver health, see HB_GPS_* below
} HeartbeatPacket_t;
#define HEARTBEAT_PACKET_SIZE     8
#define HEARTBEAT_PACKET_SIZE_V1  7   // legacy layout without gps_health

/* gps_health encoding: low nibble = state, high nibble = watchdog recovery
 * attempts (0-3). Answers the pad question "is the GPS still ACQUIRING, or
 * is the wiring/module dead?" - satellites=0 alone can't tell those apart. */
#define HB_GPS_UNKNOWN     0   // old TX firmware (byte absent, decoded as 0)
#define HB_GPS_NO_DATA     1   // GPS UART silent: wiring/power/module fault
#define HB_GPS_NO_NMEA     2   // bytes arriving but no parseable NMEA (baud?)
#define HB_GPS_ACQUIRING   3   // NMEA flowing, waiting for a fix - normal
#define HB_GPS_STATE(h)    ((uint8_t)((h) & 0x0F))
#define HB_GPS_RESETS(h)   ((uint8_t)(((h) >> 4) & 0x0F))
#define HB_GPS_HEALTH(state, resets) \
    ((uint8_t)((((resets) & 0x0F) << 4) | ((state) & 0x0F)))

// Flags byte bit definitions
#define FLAG_LAUNCH_DETECTED    0x80  // Bit 7: 1 = launched, 0 = on ground
#define FLAG_FIX_QUALITY_GOOD   0x40  // Bit 6: 1 = good fix, 0 = poor
#define FLAG_LANDED             0x10  // Bit 4: 1 = landing detected (latched)
#define FLAG_FIX_TYPE_MASK      0x0F  // Bits 3-0: GPS fix type (0=none, 1=GPS, 2=DGPS, etc.)

/* FusedPosPacket_t flags (must match transmitter/firmware/include/packet_format.h) */
#define FUSED_FLAG_LAUNCH_DETECTED   0x80
#define FUSED_FLAG_GPS_FRESH         0x40
#define FUSED_FLAG_IMU_HEALTHY       0x20
#define FUSED_FLAG_DEAD_RECKONING    0x10
#define FUSED_FLAG_LANDED            0x08

/* Fused packet: 21 bytes. See transmitter include for full field semantics. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FUSED
    int32_t  latitude;       // deg * 10^7
    int32_t  longitude;      // deg * 10^7
    int32_t  altitude_cm;    // cm MSL
    int16_t  v_n_cms;        // cm/s
    int16_t  v_e_cms;        // cm/s
    int16_t  v_d_cms;        // cm/s
    uint8_t  age_ds;         // deciseconds since TX-side last GPS fix
    uint8_t  flags;          // FUSED_FLAG_*
} FusedPosPacket_t;

#define FUSED_PACKET_SIZE       21

// Helper macros for encoding/decoding
#define GPS_COORD_SCALE         10000000.0  // Scale factor for lat/lon (10^7)

// Convert decimal degrees to encoded integer
#define ENCODE_COORD(deg)       ((int32_t)((deg) * GPS_COORD_SCALE))

// Convert encoded integer back to decimal degrees
#define DECODE_COORD(val)       ((double)(val) / GPS_COORD_SCALE)

#endif // _PACKET_FORMAT_H_
