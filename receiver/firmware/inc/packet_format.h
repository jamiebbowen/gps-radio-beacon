#ifndef _PACKET_FORMAT_H_
#define _PACKET_FORMAT_H_

#include <stdint.h>

// Packet format selection (must match transmitter!)
#define USE_BINARY_PACKETS  1  // Set to 1 for binary, 0 for ASCII

/* Binary GPS packet structure: V2 = 14 bytes, V1 (legacy, no rocket_id)
 * = 13 bytes. rocket_id lets us drop a foreign beacon on the same channel
 * running this same firmware (see the airframe binding in rf_receiver.c).
 * Valid IDs are 0..254; 255 is the receiver's "unbound" sentinel. */
typedef struct __attribute__((packed)) {
    uint8_t packet_type;     // 0x01 = GPS position packet
    int32_t latitude;        // Latitude * 10^7 (e.g., 39.890075° = 398900750)
    int32_t longitude;       // Longitude * 10^7 (e.g., -105.115510° = -1051155100)
    int16_t altitude;        // Altitude in meters (range: -32768 to +32767)
    uint8_t satellites;      // Number of satellites (0-255)
    uint8_t flags;          // Status flags (see below)
    uint8_t rocket_id;      // ROCKET_ID of the airframe (V2)
} BinaryGPSPacket_t;
#define GPS_PACKET_SIZE       14
#define GPS_PACKET_SIZE_V1    13   // legacy layout without rocket_id

// Packet types
#define PACKET_TYPE_GPS         0x01
#define PACKET_TYPE_CALLSIGN    0x02  // Future use
#define PACKET_TYPE_TELEMETRY   0x03  // Future use
#define PACKET_TYPE_FUSED       0x04  // EKF-fused position + velocity (TX nav module)
#define PACKET_TYPE_HEARTBEAT   0x05  // No-fix keepalive (see HeartbeatPacket_t)
#define PACKET_TYPE_IMU         0x06  // Inertial trace during ascent (forensics)
#define PACKET_TYPE_LAUNCH_T0   0x07  // "T0 declared, uptime=N" - one-shot
#define PACKET_TYPE_CMD         0x08  // RX -> TX command channel (v2 radio)
#define PACKET_TYPE_ACK         0x09  // TX -> RX acknowledgement of command
#define PACKET_TYPE_FLIGHT_EVENT 0x0A // Apogee/deploy/landing + flight anomaly
#define PACKET_TYPE_MAXIMA     0x0B // Running flight maxima recap (in flight)
#define PACKET_TYPE_HELLO      0x0C // Boot identity: fw hash (one-shot)

/* Two-way channel: 6-byte frames; see transmitter copy for field notes. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;
    uint8_t  rocket_id;
    uint8_t  cmd_code;
    uint8_t  seq_hi;
    uint8_t  seq_lo;
    uint8_t  param;
} CmdPacket_t;
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;
    uint8_t  rocket_id;
    uint8_t  cmd_code;
    uint8_t  seq_hi;
    uint8_t  seq_lo;
    uint8_t  echo;
} AckPacket_t;
#define CMD_ACK_PACKET_SIZE   6
#define CMD_PING              0x01
#define CMD_TUNE_NEXT_CH      0x02

/* Heartbeat: V3 = 9 bytes (V2 = 8, V1 = 7). The transmitter sends this
 * instead of a GPS packet when it has no transmittable fix (no fix / <4
 * sats), so the channel scan can lock and the operator can see the beacon
 * is alive on the pad. Must stay in sync with the transmitter's copy of
 * this header. Shorter legacy layouts zero-fill (unknown reset_info 00). */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;   // PACKET_TYPE_HEARTBEAT
    uint8_t  rocket_id;     // ROCKET_ID of the airframe
    uint8_t  channel;       // Active (jumper-resolved) channel
    uint8_t  satellites;    // Satellites currently tracked
    uint8_t  fix_quality;   // GGA fix quality (0 = none)
    uint16_t uptime_s;      // Seconds since beacon boot (saturates at 65535)
    uint8_t  gps_health;    // GPS receiver health, see HB_GPS_* below
    uint8_t  reset_info;    // V3: SAMD51 RSTC_RCAUSE on the TX boot that
                            // started this session (HB_RESET_* bit masks).
} HeartbeatPacket_t;
#define HEARTBEAT_PACKET_SIZE     9
#define HEARTBEAT_PACKET_SIZE_V2  8   // legacy layout without reset_info
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

/* reset_info bits (verbatim SAMD51 RSTC_RCAUSE, component/rstc.h) */
#define HB_RESET_POR     0x01        /* Power-on reset                   */
#define HB_RESET_BODCORE 0x02        /* Brown-out, core rail (VDDCORE)   */
#define HB_RESET_BODVDD  0x04        /* Brown-out drain, 3.3V rail       */
#define HB_RESET_NVM     0x08        /* NVM underway reboot              */
#define HB_RESET_EXT     0x10        /* External reset pin               */
#define HB_RESET_WDT     0x20        /* Watchdog timeout                 */
#define HB_RESET_SYST    0x40        /* NVIC system reset request        */
#define HB_RESET_BACKUP  0x80        /* Backup/hibernate exit            */
#define HB_RESET_NAME(i)  ( \
    ((i) & HB_RESET_WDT)     ? "WDT"  : \
    ((i) & HB_RESET_BODCORE) ? "BOD-c" : \
    ((i) & HB_RESET_BODVDD)  ? "BOD-v" : \
    ((i) & HB_RESET_EXT)     ? "EXT"  : \
    ((i) & HB_RESET_POR)     ? "POR"  : \
    ((i) & HB_RESET_SYST)    ? "SYST" : \
    ((i) & HB_RESET_NVM)     ? "NVM"  : \
    ((i) & HB_RESET_BACKUP)  ? "BKP"  : "?" )

// Flags byte bit definitions
#define FLAG_LAUNCH_DETECTED    0x80  // Bit 7: 1 = launched, 0 = on ground
#define FLAG_FIX_QUALITY_GOOD   0x40  // Bit 6: 1 = good fix, 0 = poor
#define FLAG_LOW_SATS           0x20  // Bit 5: post-landing 2D fix (3 sats) - horizontal only
#define FLAG_LANDED             0x10  // Bit 4: 1 = landing detected (latched)
#define FLAG_FIX_TYPE_MASK      0x0F  // Bits 3-0: GPS fix type (0=none, 1=GPS, 2=DGPS, etc.)

/* FusedPosPacket_t flags (must match transmitter/firmware/include/packet_format.h) */
#define FUSED_FLAG_LAUNCH_DETECTED   0x80  /* Bit 7: 1 = launched                         */
#define FUSED_FLAG_GPS_FRESH         0x40  /* Bit 6: 1 = GPS fix used within last second  */
#define FUSED_FLAG_IMU_HEALTHY       0x20  /* Bit 5: 1 = BNO085 streaming               */
#define FUSED_FLAG_DEAD_RECKONING    0x10  /* Bit 4: 1 = no GPS for > NAV_DR_TIMEOUT_S    */
#define FUSED_FLAG_LANDED            0x08  /* Bit 3: 1 = landing detected (latched)       */
#define FUSED_FLAG_SENSOR_DEGRADED   0x04  /* Bit 2: 1 = BNO085 dead or in retry          */
#define FUSED_FLAG_GATE_REJECT       0x02  /* Bit 1: EKF gate rejected a fix since last
                                                * fused packet (delta, not latched)      */
#define FUSED_FLAG_V3                0x01  /* Bit 0: V3 wire marker (always set
                                              * on the air; distinguishes V3
                                              * from legacy 19-byte V1)      */
#define FUSED_FLAG_RESERVED_MASK     FUSED_FLAG_V3  /* old name for bit 0    */

/* Fused packet: V3 = 19 bytes (no age_ds: the receiver derives staleness
 * from GPS_FRESH + inter-arrival time; the V2 rocket_id byte had pushed
 * the packet across the 987->1118 ms airtime step). See the transmitter
 * include for field semantics; the two copies must never drift (host
 * tests pin the literals on both sides).
 * Versions: len 20 = V2, len 19 with flags bit0 set = V3, len 19
 * without = V1 (pre-rocket_id legacy). */
#define FUSED_ALT_FLOOR_M   500.0f   /* subtracted from alt_m when encoding */
#define FUSED_ALT_SCALE     4.0f     /* quarter-meters per count            */

typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FUSED
    int32_t  latitude;       // deg * 10^7
    int32_t  longitude;      // deg * 10^7
    uint16_t alt_qm;         // (alt_m + 500) * 4, quarter-meters
    int16_t  v_n_cms;        // cm/s
    int16_t  v_e_cms;        // cm/s
    int16_t  v_d_cms;        // cm/s
    uint8_t  flags;          // FUSED_FLAG_* (bit0 = FUSED_FLAG_V3 marker)
    uint8_t  rocket_id;      // ROCKET_ID of the airframe
} FusedPosPacket_t;

#define FUSED_PACKET_SIZE       19   /* V3 */
#define FUSED_PACKET_SIZE_V2    20   /* age_ds + rocket_id                */
#define FUSED_PACKET_SIZE_V1    19   /* legacy: age_ds, no rocket_id; told
                                        apart from V3 by flags bit0      */

/* LAUNCH_T0: 6 bytes - one-shot on the air at the instant detection trips,
 * so any crash record (beacon silent) still carries a certified T0. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_LAUNCH_T0
    uint8_t  rocket_id;
    uint16_t uptime_s;       // TX uptime at T0
    uint16_t age_ds;         // age of the newest GPS fix at T0 (deciseconds)
} LaunchT0Packet_t;
#define LAUNCH_T0_PACKET_SIZE   6

/* IMU inertial trace: 16 bytes, interleaved with fused packets during
 * LAUNCH/POST_LAUNCH at ~1 Hz. The missing-narrative parts of a
 * shred-the-airframe failure mode. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_IMU
    uint8_t  rocket_id;
    uint16_t ts_ms;          // ms mod 1000
    int16_t  accel_x_cg;     // centi-g, BNO085 linear accel
    int16_t  accel_y_cg;
    int16_t  accel_z_cg;
    int16_t  gyro_x_cds;     // centi-deg/s
    int16_t  gyro_y_cds;
    int16_t  gyro_z_cds;
    int16_t  peak_accel_mg;  // milli-g peak |a| since previous IMU packet
    int16_t  temp_c10;       // V1 only: BNO085 temperature * 10 (never
                             // wired; zero-filled on V2-length packets)
} ImuTracePacket_t;
#define IMU_TRACE_PACKET_SIZE    18   /* V2: temp dropped (never wired) */
#define IMU_TRACE_PACKET_SIZE_V1 20   /* legacy layout with temp_c10    */

/* FLIGHT_EVENT: 8 bytes; must match the transmitter copy (host tests pin
 * the literals on both sides). One packet type carries the certified
 * one-shot life-cycle events (apogee, drogue, main, landed) and the
 * flight-anomaly codes; value semantics per code:
 *   APOGEE:        fused altitude, whole meters (int16)
 *   DROGUE / MAIN: descent rate, cm/s at declaration
 *   LANDED:        0
 *   anomalies:     descent rate cm/s at announcement */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FLIGHT_EVENT
    uint8_t  rocket_id;      // ROCKET_ID of the airframe
    uint8_t  code;           // FLIGHT_EVENT_*
    uint16_t uptime_s;       // TX uptime at TX time
    int16_t  value;          // per-code, see above
} FlightEventPacket_t;
#define FLIGHT_EVENT_PACKET_SIZE    7
#define FLIGHT_EVENT_PACKET_SIZE_V1 8   /* legacy layout with spare byte  */

#define FLIGHT_EVENT_NONE             0x00
#define FLIGHT_EVENT_APOGEE           0x01
#define FLIGHT_EVENT_DROGUE           0x02
#define FLIGHT_EVENT_MAIN             0x03
#define FLIGHT_EVENT_LANDED           0x04
#define FLIGHT_EVENT_ANOM_BALLISTIC   0x10
#define FLIGHT_EVENT_ANOM_TUMBLE      0x11
#define FLIGHT_EVENT_ANOM_SENSOR_LOSS 0x12
#define FLIGHT_EVENT_ANOM_GPS_OUTAGE  0x13
#define FLIGHT_EVENT_ANOM_REBOOT      0x14  /* reserved */
#define FLIGHT_EVENT_IS_ANOMALY(c)    ((c) >= 0x10)
#define FLIGHT_EVENT_NAME(c) ( \
    ((c) == FLIGHT_EVENT_APOGEE)           ? "APOGEE"      : \
    ((c) == FLIGHT_EVENT_DROGUE)           ? "DROGUE"      : \
    ((c) == FLIGHT_EVENT_MAIN)             ? "MAIN"        : \
    ((c) == FLIGHT_EVENT_LANDED)           ? "LANDED"      : \
    ((c) == FLIGHT_EVENT_ANOM_BALLISTIC)   ? "ANOM-BALLISTIC" : \
    ((c) == FLIGHT_EVENT_ANOM_TUMBLE)      ? "ANOM-TUMBLE"    : \
    ((c) == FLIGHT_EVENT_ANOM_SENSOR_LOSS) ? "ANOM-SENSOR"    : \
    ((c) == FLIGHT_EVENT_ANOM_GPS_OUTAGE)  ? "ANOM-GPS"       : \
    ((c) == FLIGHT_EVENT_ANOM_REBOOT)      ? "ANOM-REBOOT"    : "EVENT-?" )

/* MAXIMA: 12 bytes; must match the transmitter copy. Re-announced every
 * few seconds while airborne + one copy at the landing latch: the running
 * flight maxima, so the last packet before a total loss still certifies
 * how high/fast/hard the airframe had flown up to that second. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;     // PACKET_TYPE_MAXIMA
    uint8_t  rocket_id;
    int16_t  max_alt_m;       // highest fused altitude so far, m MSL
    uint16_t t_maxalt_s;      // TX uptime at that altitude
    uint16_t max_speed_cms;   // peak |v| cm/s
    uint16_t max_accel_cg;    // peak |linear accel| centi-g
    uint8_t  max_gyro_dps16;  // peak |gyro| in units of 16 dps
} MaximaPacket_t;
#define MAXIMA_GYRO_DPS_SCALE  16u
#define MAXIMA_PACKET_SIZE     11
#define MAXIMA_PACKET_SIZE_V1  12   /* legacy: u16 max_gyro_dps          */

/* HELLO: 8 bytes; must match the transmitter copy. One-shot at boot: the
 * beacon's firmware identity (git short hash, MSB = dirty tree). */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_HELLO
    uint8_t  rocket_id;
    uint32_t fw_hash;
    uint16_t uptime_s;       /* V1 only; absent on the wire in V2        */
} HelloPacket_t;
#define HELLO_PACKET_SIZE    6   /* V2: uptime dropped (always ~2 s)     */
#define HELLO_PACKET_SIZE_V1 8   /* legacy layout with uptime_s          */

// Helper macros for encoding/decoding
#define GPS_COORD_SCALE         10000000.0  // Scale factor for lat/lon (10^7)

// Convert decimal degrees to encoded integer
#define ENCODE_COORD(deg)       ((int32_t)((deg) * GPS_COORD_SCALE))

// Convert encoded integer back to decimal degrees
#define DECODE_COORD(val)       ((double)(val) / GPS_COORD_SCALE)

#endif // _PACKET_FORMAT_H_
