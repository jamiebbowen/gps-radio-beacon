#ifndef _PACKET_FORMAT_H_
#define _PACKET_FORMAT_H_

#include <stdint.h>

// Packet format selection
#define USE_BINARY_PACKETS  1  // Set to 1 for binary, 0 for ASCII

/* Binary GPS packet structure: V2 = 14 bytes, V1 (legacy, no rocket_id)
 * = 13 bytes. rocket_id lets the receiver ignore a foreign beacon on the
 * same channel running this same firmware. Valid IDs are 0..254; 255 is
 * reserved as the receiver's "unbound" sentinel. */
typedef struct __attribute__((packed)) {
    uint8_t packet_type;     // 0x01 = GPS position packet
    int32_t latitude;        // Latitude * 10^7 (e.g., 39.890075° = 398900750)
    int32_t longitude;       // Longitude * 10^7 (e.g., -105.115510° = -1051155100)
    int16_t altitude;        // Altitude in meters (range: -32768 to +32767)
    uint8_t satellites;      // Number of satellites (0-255)
    uint8_t flags;          // Status flags (see below)
    uint8_t rocket_id;      // ROCKET_ID of this airframe (V2)
} BinaryGPSPacket_t;
#define GPS_PACKET_SIZE       14
#define GPS_PACKET_SIZE_V1    13   // legacy layout without rocket_id

// Packet types
#define PACKET_TYPE_GPS         0x01
#define PACKET_TYPE_CALLSIGN    0x02  // Future use
#define PACKET_TYPE_TELEMETRY   0x03  // Future use
#define PACKET_TYPE_FUSED       0x04  // EKF-fused position + velocity
#define PACKET_TYPE_HEARTBEAT   0x05  // No-fix keepalive (see HeartbeatPacket_t)
#define PACKET_TYPE_IMU         0x06  // Inertial trace during ascent (forensics)
#define PACKET_TYPE_LAUNCH_T0   0x07  // "T0 declared, uptime=N" - one-shot
#define PACKET_TYPE_CMD         0x08  // RX -> TX command channel (v2 radio)
#define PACKET_TYPE_ACK         0x09  // TX -> RX acknowledgement of command
#define PACKET_TYPE_FLIGHT_EVENT 0x0A // Apogee/deploy/landing + flight anomaly
#define PACKET_TYPE_MAXIMA     0x0B // Running flight maxima recap (in flight)
#define PACKET_TYPE_HELLO      0x0C // Boot identity: fw hash (one-shot)

/* Heartbeat: 8 bytes. Sent instead of a GPS packet when no transmittable fix
 * exists (no fix / <4 sats), so the receiver's channel scan can lock and the
 * operator can see the beacon is alive on the pad. Must stay in sync with
 * the receiver's copy of this header. The receiver also accepts the older
 * 7-byte layout (no gps_health) from beacons on previous firmware. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;   // PACKET_TYPE_HEARTBEAT
    uint8_t  rocket_id;     // ROCKET_ID of this airframe
    uint8_t  channel;       // Active (jumper-resolved) channel
    uint8_t  satellites;    // Satellites currently tracked
    uint8_t  fix_quality;   // GGA fix quality (0 = none)
    uint16_t uptime_s;      // Seconds since beacon boot (saturates at 65535)
    uint8_t  gps_health;    // GPS receiver health, see HB_GPS_* below
    uint8_t  reset_info;    // V3: raw SAMD51 RSTC_RCAUSE of the boot that
                            // started this session (see HB_RESET_CAUSE_*).
                            // Receiver's uptime-regression check still fires
                            // for mid-flight resets; this adds the WHY.
} HeartbeatPacket_t;
#define HEARTBEAT_PACKET_SIZE     9
#define HEARTBEAT_PACKET_SIZE_V2  8   // without reset_info
#define HEARTBEAT_PACKET_SIZE_V1  7   // legacy layout without gps_health

/* reset_info: verbatim SAMD51 RSTC_RCAUSE register. From the CMSIS header
 * (component/rstc.h): POR bit0, BODCORE bit1, BODVDD bit2, NVM bit3,
 * EXT bit4, WDT bit5, SYST bit6, BACKUP bit7. */
#define HB_RESET_POR     0x01        /* Power-on reset                   */
#define HB_RESET_BODCORE 0x02        /* Brown-out, core rail (VDDCORE)   */
#define HB_RESET_BODVDD  0x04        /* Brown-out drain, 3.3V rail       */
#define HB_RESET_NVM     0x08        /* NVM underway reboot              */
#define HB_RESET_EXT     0x10        /* External reset pin               */
#define HB_RESET_WDT     0x20        /* Watchdog timeout                 */
#define HB_RESET_SYST    0x40        /* NVIC system reset request        */
#define HB_RESET_BACKUP  0x80        /* Backup/hibernate exit            */

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
#define FLAG_LOW_SATS           0x20  // Bit 5: post-landing 2D fix (3 sats) - horizontal only
#define FLAG_LANDED             0x10  // Bit 4: 1 = landing detected (latched)
#define FLAG_FIX_TYPE_MASK      0x0F  // Bits 3-0: GPS fix type (0=none, 1=GPS, 2=DGPS, etc.)

/* FusedPosPacket_t flags (separate bitmap from the GPS packet flags) */
#define FUSED_FLAG_LAUNCH_DETECTED   0x80  // Bit 7: 1 = launched
#define FUSED_FLAG_GPS_FRESH         0x40  // Bit 6: 1 = GPS fix used within the last second
#define FUSED_FLAG_IMU_HEALTHY       0x20  // Bit 5: 1 = BNO085 streaming, not saturated
#define FUSED_FLAG_DEAD_RECKONING    0x10  // Bit 4: 1 = no GPS for > NAV_DR_TIMEOUT_S
#define FUSED_FLAG_LANDED            0x08  // Bit 3: 1 = landing detected (latched)
#define FUSED_FLAG_SENSOR_DEGRADED   0x04  // Bit 2: 1 = BNO085 (the EKF's only inertial source) dead or in retry
#define FUSED_FLAG_GATE_REJECT       0x02  // Bit 1: the EKF innovation gate rejected >=1 GPS fix since
                                             // the previous fused packet (per-packet delta - transient,
                                             // not latched; a cluster of these precedes nav rescue)
#define FUSED_FLAG_V3                0x01  // Bit 0: V3 wire marker (always set on the air;
                                             // lets the RX tell V3 from legacy 19-byte V1)
#define FUSED_FLAG_RESERVED_MASK     FUSED_FLAG_V3  /* old name for bit 0 */

/* Fused packet: V3 = 19 bytes. V3 dropped age_ds to get back under the
 * SF10/BW62.5k airtime step the V2 rocket_id byte had crossed: 19 B costs
 * 987 ms where 20 B costs 1118 ms - a ~9% duty rebate on the densest
 * stream on the link. The receiver derives staleness itself (GPS_FRESH
 * flag + inter-arrival time), so the age field only ever fed a display
 * chip. Receiver versions: len 20 = V2 (age_ds + rocket_id), len 19 with
 * flags bit0 set = V3, len 19 without = V1 (pre-rocket_id legacy).
 *
 *   lat/lon       : same scaling as BinaryGPSPacket_t (deg * 10^7)
 *   alt_qm        : altitude in 0.25 m units with a 500 m MSL floor offset,
 *                   uint16: covers -500.0 m to +15883.75 m.
 *   v_n/v_e/v_d   : NED velocity in cm/s (int16 gives ±327 m/s per axis)
 *   flags         : FUSED_FLAG_* bits above (FUSED_FLAG_V3 always set)
 *   rocket_id     : ROCKET_ID of this airframe (0..254, 255 reserved)
 */
#define FUSED_ALT_FLOOR_M   500.0f   /* subtracted from alt_m when encoding */
#define FUSED_ALT_SCALE     4.0f     /* quarter-meters per count            */
#define FUSED_PACKET_SIZE     19   /* V3: 987 ms on air                    */
#define FUSED_PACKET_SIZE_V2  20   /* age_ds + rocket_id                   */
#define FUSED_PACKET_SIZE_V1  19   /* legacy: age_ds, no rocket_id; told   */
                                   /* apart from V3 by flags bit0          */

typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FUSED
    int32_t  latitude;       // deg * 10^7
    int32_t  longitude;      // deg * 10^7
    uint16_t alt_qm;         // (alt_m + 500) * 4, quarter-meters
    int16_t  v_n_cms;        // North velocity, cm/s
    int16_t  v_e_cms;        // East  velocity, cm/s
    int16_t  v_d_cms;        // Down  velocity, cm/s
    uint8_t  flags;          // FUSED_FLAG_* (bit0 = FUSED_FLAG_V3 marker)
    uint8_t  rocket_id;      // ROCKET_ID of this airframe (0..254)
} FusedPosPacket_t;

/* LAUNCH_T0: 6 bytes. Sent once, the instant launch detection trips, so a
 * flight record always starts with a crisp T0 mark regardless of what the
 * packet cadence happened to be doing that second. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_LAUNCH_T0
    uint8_t  rocket_id;
    uint16_t uptime_s;       // TX uptime at the moment T0 was declared
    uint16_t age_ds;         // age of the newest GPS fix at T0 (deciseconds)
} LaunchT0Packet_t;
#define LAUNCH_T0_PACKET_SIZE   6

/* IMU inertial trace: 18 bytes (V2), interleaved with fused packets during
 * LAUNCH/POST_LAUNCH at ~1 Hz. Thought experiment that motivated it: a
 * shredded-at-burnout flight like L0016 gives two position packets and
 * nothing else; peak accel + rotation rates are the only measurable
 * evidence of what came apart. V2 dropped temp_c10 (the BNO085 temp path
 * was never wired - a hardwired sentinel): 18 B crosses back under the
 * 987 ms airtime step vs. 1118 ms at 20 B. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_IMU
    uint8_t  rocket_id;
    uint16_t ts_ms;          // ms within the last second (timestamp mod 1000)
    int16_t  accel_x_cg;     // centi-g (1/100 g); BNO085 linear accel
    int16_t  accel_y_cg;
    int16_t  accel_z_cg;
    int16_t  gyro_x_cds;     // centi-deg/s (1/100 deg/s)
    int16_t  gyro_y_cds;
    int16_t  gyro_z_cds;
    int16_t  peak_accel_mg;  // milli-g: highest |a| since previous IMU packet
} ImuTracePacket_t;
#define IMU_TRACE_PACKET_SIZE    18   /* V2                            */
#define IMU_TRACE_PACKET_SIZE_V1 20   /* legacy layout with temp_c10   */

/* FLIGHT_EVENT: 8 bytes. One packet type carries the certified one-shot
 * life-cycle events (apogee, drogue, main, landed) AND the flight-anomaly
 * sequencer's codes; the code byte says which. One-shots are repeated
 * FLIGHT_EVENT_REPEATS times so a single RF-null moment can't erase the
 * record; anomalies re-announce every ANOM_EVENT_REPEAT_MS while the
 * condition holds. Detection lives in flight_events.cpp (thresholds in
 * config.h). value semantics per code:
 *   APOGEE:        fused altitude, whole meters (int16)
 *   DROGUE / MAIN: descent rate, cm/s at declaration
 *   LANDED:        0
 *   anomalies:     descent rate cm/s at announcement (debris-track context) */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_FLIGHT_EVENT
    uint8_t  rocket_id;      // ROCKET_ID of this airframe
    uint8_t  code;           // FLIGHT_EVENT_*
    uint16_t uptime_s;       // TX uptime at TX time (saturates at 65535)
    int16_t  value;          // per-code, see above
} FlightEventPacket_t;
#define FLIGHT_EVENT_PACKET_SIZE    7   /* 594 ms vs 725 with the spare  */
#define FLIGHT_EVENT_PACKET_SIZE_V1 8   /* legacy layout with spare byte */

#define FLIGHT_EVENT_NONE             0x00
#define FLIGHT_EVENT_APOGEE           0x01  /* v_d held downward post-launch   */
#define FLIGHT_EVENT_DROGUE           0x02  /* descent rate entered drogue band */
#define FLIGHT_EVENT_MAIN             0x03  /* descent rate stepped below band  */
#define FLIGHT_EVENT_LANDED           0x04  /* landing latch tripped            */
#define FLIGHT_EVENT_ANOM_BALLISTIC   0x10  /* no drogue: rate > ballistic band */
#define FLIGHT_EVENT_ANOM_TUMBLE      0x11  /* gyro magnitude past 1000 dps     */
#define FLIGHT_EVENT_ANOM_SENSOR_LOSS 0x12  /* BNO085 dead in flight            */
#define FLIGHT_EVENT_ANOM_GPS_OUTAGE  0x13  /* no fix past rescue horizon, airborne */
#define FLIGHT_EVENT_ANOM_REBOOT      0x14  /* reserved: reboot-while-airborne
                                             * (needs flight_log phase resume) */
#define FLIGHT_EVENT_IS_ANOMALY(c)    ((c) >= 0x10)

/* MAXIMA: 12 bytes. The "black box on the air": while airborne, every
 * MAXIMA_TX_INTERVAL_MS the beacon re-announces its running flight maxima,
 * so the LAST packet heard before a total loss still certifies how
 * high/fast/hard the airframe had flown up to that second (L0016 died
 * ~2 s off the rail with nothing but pad data on the card). One final copy
 * goes out at the landing latch. All fields are magnitudes. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;     // PACKET_TYPE_MAXIMA
    uint8_t  rocket_id;
    int16_t  max_alt_m;       // highest fused altitude so far, m MSL
    uint16_t t_maxalt_s;      // TX uptime at that altitude (saturates)
    uint16_t max_speed_cms;   // peak |v| cm/s (covers 655 m/s)
    uint16_t max_accel_cg;    // peak |linear accel| centi-g (covers 655 g)
    uint8_t  max_gyro_dps16;  // peak |gyro| in units of 16 dps (0..4080 dps,
                              // past the BNO085's own +-2000 dps range)
} MaximaPacket_t;
#define MAXIMA_GYRO_DPS_SCALE  16u
#define MAXIMA_PACKET_SIZE     11   /* V2: 725 ms vs 856 at 12 B */
#define MAXIMA_PACKET_SIZE_V1  12   /* legacy: u16 max_gyro_dps  */

/* HELLO: 8 bytes. Sent once at boot, right after the callsign: the wire
 * copy of the beacon's firmware identity (git short hash + dirty mark),
 * so every receiver log names exactly which transmitter firmware flew.
 * The receiver logs its own fw hash in its first row; L0016's forensic
 * readout stalled partly on "which TX firmware was this, what could it
 * even have sent?". 28 hash bits + MSB dirty flag. */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;    // PACKET_TYPE_HELLO
    uint8_t  rocket_id;
    uint32_t fw_hash;        // GIT_HASH_HEX (MSB set when tree was dirty)
} HelloPacket_t;
#define HELLO_PACKET_SIZE    6   /* uptime at boot is always ~2 s - cut */
#define HELLO_PACKET_SIZE_V1 8   /* legacy layout with uptime_s         */

/* ------------------------------------------------------------------
 * Two-way channel (post-landing / diagnostics). Both frames are 6 bytes:
 *   [0] type   [1] rocket_id   [2] cmd_code   [3] seq_hi   [4] seq_lo
 *   [5] param/echo
 * cmd_code: 0x01=PING (receiver answers with ACK echo), 0x02=TUNE_NEXT_CH
 * (move us both to the next frequency - dissent tolerated between sites),
 * spare values reserved for future hardware. */
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

// Helper macros for encoding/decoding
#define GPS_COORD_SCALE         10000000.0f  // Scale factor for lat/lon (10^7)

// Convert decimal degrees to encoded integer
#define ENCODE_COORD(deg)       ((int32_t)((deg) * GPS_COORD_SCALE))

// Convert encoded integer back to decimal degrees
#define DECODE_COORD(val)       ((float)(val) / GPS_COORD_SCALE)

#endif // _PACKET_FORMAT_H_
