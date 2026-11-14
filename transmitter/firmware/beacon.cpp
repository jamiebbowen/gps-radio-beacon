#include "include/beacon.h"
#include "include/fixed_point.h"
#include "include/config.h"
#include "include/mpu_config.h"
#include "include/radio.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "include/nav.h"
#include "include/flight_maxima.h"
#include "include/nmea_fields.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* 255 is the receiver's "unbound" sentinel for the airframe filter; an
 * airframe claiming it would be invisible to that filter. */
static_assert(ROCKET_ID != 255, "ROCKET_ID 255 is reserved (RX unbound sentinel)");

/**
 * The fix gate shared by every position sender (the flight-window sender
 * and the pad audit in firmware.ino): valid, past the "0" pre-fix
 * placeholder, enough satellites, fix quality >= 1, sane altitude.
 * @param log_reason nonzero: print why a fix is rejected
 * @return 1 if the fix may go on the air
 */
uint8_t beacon_validate_fix(const GPSCoordinates_t *coords, uint8_t log_reason)
{
    if (coords == NULL) return 0;
    if (!coords->valid || (coords->lat[0] == '0' && strlen(coords->lat) == 1)) {
        if (log_reason) {
            Serial.print(F("[Beacon] Rejecting GPS - valid="));
            Serial.print(coords->valid);
            Serial.print(F(" lat='"));
            Serial.print(coords->lat);
            Serial.print(F("' lon='"));
            Serial.print(coords->lon);
            Serial.println(F("'"));
        }
        return 0;
    }

    /* Minimum satellites: 4 (3D fix) in flight. Once the landing latch is
     * set, a 3-sat 2D fix still carries recovery-grade horizontal
     * coordinates - and a rocket under canopy with 3 sats would otherwise
     * go position-silent exactly when the crew is walking in (it would
     * heartbeat instead, with only the stale pre-canopy fix surviving).
     * The 0x20 flag marks degraded geometry so the log explains itself. */
    int sat_count = atoi(coords->satellites);
    int min_sats = launch_detect_has_landed() ? 3 : 4;
    if (sat_count < min_sats) {
        if (log_reason) {
            Serial.print(F("[Beacon] Rejecting GPS - insufficient satellites: "));
            Serial.println(sat_count);
        }
        return 0;
    }

    if (coords->fix_quality < 1) {
        if (log_reason) {
            Serial.print(F("[Beacon] Rejecting GPS - no fix, quality: "));
            Serial.println(coords->fix_quality);
        }
        return 0;
    }

    float altitude = atof(coords->altitude);
    if (!isfinite(altitude) || altitude < -500.0f || altitude > 50000.0f) {
        if (log_reason) {
            Serial.print(F("[Beacon] Rejecting GPS - unreasonable altitude: "));
            Serial.println(altitude);
        }
        return 0;
    }
    return 1;
}

/**
 * Transmit GPS data in binary format (13 bytes)
 * @param coords Pointer to GPS coordinates structure
 * @param system_time_seconds Current system time in seconds
 * @param transmit_fast If true, transmit GPS data in fast mode
 * @return 1 if transmission was successful, 0 if coordinates were invalid
 */
uint8_t beacon_transmit_gps_data_binary(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast) {
    (void)system_time_seconds; /* API parity with the ASCII variant; binary packets carry GPS time */
    if (!beacon_validate_fix(coords, 1)) return 0;
    int sat_count = atoi(coords->satellites);
    float altitude = atof(coords->altitude);

    /* Convert + validate BEFORE the radio is powered: atof() parses
     * "nan"/"inf" field text literally, and ENCODE_COORD's float->int32
     * cast of a non-finite value is undefined behavior - a checksum-valid
     * corrupt sentence could otherwise put garbage coordinates on the
     * air. Out-of-range is rejected too: lat +-90, lon +-180. */
    double lat_decimal = gps_nmea_to_decimal(coords->lat, coords->lat_dir);
    double lon_decimal = gps_nmea_to_decimal(coords->lon, coords->lon_dir);
    if (!isfinite(lat_decimal) || !isfinite(lon_decimal) ||
        lat_decimal < -90.0f || lat_decimal > 90.0f ||
        lon_decimal < -180.0f || lon_decimal > 180.0f) {
        Serial.println(F("[Beacon] Rejecting GPS - bad coords"));
        return 0;
    }
    
    Serial.println(F("[Beacon] GPS data valid, building binary packet..."));
    
    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    
    // Build binary packet
    BinaryGPSPacket_t packet;
    packet.packet_type = PACKET_TYPE_GPS;
    packet.latitude = ENCODE_COORD(lat_decimal);
    packet.longitude = ENCODE_COORD(lon_decimal);
    // Clamp altitude to int16 max: the validation above allows up to
    // 50000 m but the wire format only carries -32768..32767 m. Without the
    // clamp, 32768..50000 m would silently wrap negative on the receiver.
    // (No negative clamp: validation already floors altitude at -500 m.)
    packet.altitude = (altitude > 32767.0f) ? 32767 : (int16_t)altitude;
    packet.satellites = (uint8_t)sat_count;
    
    // Pack flags.
    // NOTE: launch_detect_is_launched() is a one-shot (read-and-clear) edge
    // trigger owned by the main-loop state machine in firmware.ino. Calling
    // it here would consume the edge and either steal the LAUNCH transition
    // from the state machine or (more commonly) always read false because
    // the main loop already consumed it. Use the level-based state query
    // instead so every packet after launch carries the launch bit.
    packet.flags = 0;
    if (launch_detect_get_state() == LAUNCH_STATE_CONFIRMED) {
        packet.flags |= FLAG_LAUNCH_DETECTED;
    }
    /* Carried on the raw GPS stream too: the RX shows LANDED from either
     * packet type, and without it the indicator would flap between the
     * FUS stream (has FUSED_FLAG_LANDED) and this stream (cleared it). */
    if (launch_detect_has_landed()) {
        packet.flags |= FLAG_LANDED;
    }
    if (coords->fix_quality >= 1) {
        packet.flags |= FLAG_FIX_QUALITY_GOOD;
    }
    /* Degraded-geometry marker: post-landing 2D positions (min-sats relaxed
     * to 3). Receivers that predate the bit ignore it; logs read sats=3. */
    if (launch_detect_has_landed() && sat_count < 4) {
        packet.flags |= FLAG_LOW_SATS;
    }
    packet.flags |= (coords->fix_quality & FLAG_FIX_TYPE_MASK);
    /* V2 wire byte: lets a co-channel foreign beacon (this same firmware,
     * different airframe) be told apart and dropped at the receiver. */
    packet.rocket_id = (uint8_t)ROCKET_ID;

    // Transmit binary packet
    Serial.print(F("[Beacon] Transmitting binary GPS packet ("));
    Serial.print(sizeof(packet));
    Serial.print(F(" bytes): "));
    Serial.print(lat_decimal, 7);
    Serial.print(F(","));
    Serial.print(lon_decimal, 7);
    Serial.print(F(","));
    Serial.print(altitude, 1);
    Serial.print(F("m,"));
    Serial.print(sat_count);
    Serial.println(F(" sats"));
    
    int result = transmit_packet((uint8_t*)&packet, sizeof(packet));
    
    if (result != 0) {
        Serial.print(F("[Beacon] ✗ Transmission failed, error code: "));
        Serial.println(result);
    }
    
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    
    return (result == 0) ? 1 : 0;
}

/**
 * Transmit a no-fix heartbeat packet (rate-limited)
 *
 * Called from the main loop whenever a beacon TX was requested but the GPS
 * data was rejected (no fix, <4 sats, bad altitude). Without this, a beacon
 * with no fix is silent except for the 5-minute callsign, which makes the
 * receiver's channel scan and the operator wait far too long to learn the
 * rocket is powered and listening for satellites.
 *
 * @param coords Current (invalid/partial) GPS coordinates for sat count
 * @param system_time_seconds Seconds since boot
 * @param transmit_fast If true, radio is already enabled (LAUNCH phase)
 * @return 1 if a heartbeat was transmitted, 0 if rate-limited or TX failed
 */
uint8_t beacon_transmit_heartbeat(const GPSCoordinates_t* coords, uint32_t system_time_seconds,
                                  uint32_t min_interval_s, uint8_t transmit_fast) {
    /* Rate limit: the GPS TX path retries every second when there is no fix;
     * do not turn every retry into an RF transmission. On the pad states the
     * caller passes the (slower) state cadence, which the 1 Hz tick already
     * paces - this limiter is the floor, not the scheduler. */
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now_ms = millis();
    if (last_heartbeat_ms != 0 &&
        (now_ms - last_heartbeat_ms) < min_interval_s * 1000UL) {
        return 0;
    }
    
    HeartbeatPacket_t hb;
    hb.packet_type = PACKET_TYPE_HEARTBEAT;
    hb.rocket_id   = (uint8_t)ROCKET_ID;
    hb.channel     = radio_get_channel();
    hb.satellites  = coords ? nmea_u8_field(coords->satellites, 99) : 0;
    hb.fix_quality = coords ? coords->fix_quality : 0;
    hb.uptime_s    = (system_time_seconds > 65535UL) ? 65535U
                                                     : (uint16_t)system_time_seconds;
    /* Tells the receiver WHY there is no fix: still acquiring (normal) vs
     * GPS UART silent / garbled (wiring, power, baud - go check the rocket
     * before it flies). Sats=0 alone cannot distinguish those. */
    hb.gps_health  = gps_get_health();
    /* V3 heartbeat: carries the boot's SAMD51 reset-cause register, so a
     * in-flight brown-out (supply sag) or watchdog hang shows up as an
     * identity in the receiver log instead of hiding in an uptime gap. */
    extern uint8_t g_boot_rcause;
    hb.reset_info  = (uint8_t)g_boot_rcause;
    
    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    
    Serial.print(F("[Beacon] Transmitting heartbeat: sats="));
    Serial.print(hb.satellites);
    Serial.print(F(" fix="));
    Serial.print(hb.fix_quality);
    Serial.print(F(" up="));
    Serial.println(hb.uptime_s);
    
    int result = transmit_packet((uint8_t*)&hb, sizeof(hb));
    
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    
    if (result == 0) {
        last_heartbeat_ms = now_ms;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Two-way channel: answer commands during RF-quiet gaps               */
/* ------------------------------------------------------------------ */

/* Listen-at-most 40 ms per cadence gap, only outside flight windows.
 * PING echoes; TUNE_NEXT_CH is answered with echo=0xFF (unsupported on
 * beacon channels set by jumper) so the RX side learns the truth. */
void beacon_poll_commands(uint8_t in_flight)
{
    if (in_flight) return;   /* flight windows are TX-dense; don't listen */
    if (!radio_is_transmitting()) {
        uint8_t buf[32] = {0};
        int n = radio_poll_rx(buf, sizeof(buf), 40 /*ms*/);   /* <0 = error */
        if (n >= CMD_ACK_PACKET_SIZE && buf[0] == PACKET_TYPE_CMD &&
            buf[1] == (uint8_t)ROCKET_ID) {
            uint8_t echo;
            if (buf[2] == CMD_PING)        echo = buf[5];
            else if (buf[2] == CMD_TUNE_NEXT_CH) echo = 0xFF; /* unsupported */
            else                           echo = 0xFE;       /* unknown */
            radio_transmit_ack((uint8_t)ROCKET_ID, buf[2], buf[3], buf[4], echo);
            Serial.println(F("[2way] CMD answered"));
        }
    }
}

/**
 * Transmit the one-shot launch T0 packet. Fired by the main loop the instant
 * launch-detect confirms. Contains enough context (uptime, GPS age) that a
 * crash record found on the card unambiguously opens flight forensics.
 */
uint8_t beacon_transmit_launch_t0(uint32_t system_time_seconds, uint8_t transmit_fast) {
    LaunchT0Packet_t t0;
    t0.packet_type = PACKET_TYPE_LAUNCH_T0;
    t0.rocket_id   = (uint8_t)ROCKET_ID;
    t0.uptime_s    = (system_time_seconds > 65535UL) ? 65535U : (uint16_t)system_time_seconds;
    /* Age of the freshest GPS fix in deciseconds at T0 - a pad-side estimate
     * of how trustworthy the first post-shred position is. */
    uint32_t age_ms = gps_get_fix_age_ms();
    t0.age_ds = (age_ms > 25500UL) ? 255U : (uint16_t)(age_ms / 100U);

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    int result = transmit_packet((uint8_t*)&t0, sizeof(t0));
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    return (result == 0) ? 1 : 0;
}

/**
 * Transmit an inertial trace packet. Called at ~1 Hz during the flight
 * window. Feeds from the launch-detect layer's cached linear-accel + gyro.
 * peak/reset kept here so every packet reports peak-since-previous-packet.
 * The encoded packet is kept so the on-chip flight log stores exactly what
 * went on the air (beacon_last_imu_trace(), right after this call).
 */
static ImuTracePacket_t s_last_imu;
void beacon_last_imu_trace(ImuTracePacket_t *out) { if (out) *out = s_last_imu; }

uint8_t beacon_transmit_imu_trace(uint8_t transmit_fast) {
    ImuTracePacket_t p;
    memset(&p, 0, sizeof(p));
    p.packet_type = PACKET_TYPE_IMU;
    p.rocket_id   = (uint8_t)ROCKET_ID;
    /* Millisecond-carrier timestamp (0..999) fused with the SD log's
     * seconds-level Timestamp column on the receiver side; bit 15 tells
     * the receiver these are V3 units. */
    p.ts_ms       = (uint16_t)((millis() % 1000UL) | IMU_TS_V3_FLAG);

    float ax, ay, az;
    launch_detect_get_accel_xyz(&ax, &ay, &az);
    float gx, gy, gz;
    launch_detect_get_gyro_rads(&gx, &gy, &gz);

    /* BNO085 linear accel (m/s^2) -> centi-g */
    p.accel_x_cg = sat_i16(ax / MS2_PER_G * 100.0f);
    p.accel_y_cg = sat_i16(ay / MS2_PER_G * 100.0f);
    p.accel_z_cg = sat_i16(az / MS2_PER_G * 100.0f);
    /* Calibrated gyro (rad/s) -> deci-deg/s. +-3276.7 deg/s covers the
     * BNO085's +-2000 deg/s range (and the 1000 deg/s tumble alarm);
     * anything beyond saturates instead of wrapping. */
    p.gyro_x_dds = sat_i16(gx * DEG_PER_RAD * 10.0f);
    p.gyro_y_dds = sat_i16(gy * DEG_PER_RAD * 10.0f);
    p.gyro_z_dds = sat_i16(gz * DEG_PER_RAD * 10.0f);

    /* Peak linear accel (m/s^2) since the last IMU packet -> milli-g */
    static float peak_ms2 = 0.0f;
    float curr_ms2 = launch_detect_get_current_accel();
    if (curr_ms2 > peak_ms2) peak_ms2 = curr_ms2;
    p.peak_accel_mg = sat_i16(peak_ms2 / MS2_PER_G * 1000.0f);
    peak_ms2 = 0.0f;
    s_last_imu = p;

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    int result = transmit_packet((uint8_t*)&p, sizeof(p));
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    return (result == 0) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* Certified flight events (APOGEE / DROGUE / MAIN / LANDED + anomaly   */
/* codes): one-shot edges that must survive a single RF-null moment.    */
/* ------------------------------------------------------------------ */

#define FLIGHT_EVENT_Q_DEPTH 8
static struct {
    uint8_t  code;
    int16_t  value;
    uint8_t  left;          /* transmissions remaining (0 = free slot)  */
    uint32_t due_ms;        /* next allowed TX time                     */
} event_q[FLIGHT_EVENT_Q_DEPTH];

void beacon_queue_flight_event(uint8_t code, int16_t value, uint8_t repeats)
{
    for (uint8_t i = 0; i < FLIGHT_EVENT_Q_DEPTH; i++) {
        if (event_q[i].left == 0) {
            event_q[i].code   = code;
            event_q[i].value  = value;
            event_q[i].left   = (repeats == 0) ? 1 : repeats;
            event_q[i].due_ms = millis();   /* first copy goes immediately */
            return;
        }
    }
    /* All slots busy: the edge is lost. Loud on USB so a bench run can
     * see it; on the air the anomaly re-announce covers level events. */
    Serial.print(F("[Beacon] Flight event queue full, dropped 0x"));
    Serial.println(code, HEX);
}

uint8_t beacon_transmit_flight_event(uint8_t code, int16_t value,
                                     uint32_t system_time_seconds, uint8_t transmit_fast) {
    FlightEventPacket_t p;
    p.packet_type = PACKET_TYPE_FLIGHT_EVENT;
    p.rocket_id   = (uint8_t)ROCKET_ID;
    p.code        = code;
    p.uptime_s    = (system_time_seconds > 65535UL) ? 65535U
                                                    : (uint16_t)system_time_seconds;
    p.value       = value;

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }

    Serial.print(F("[Beacon] Flight event 0x"));
    Serial.print(code, HEX);
    Serial.print(F(" value="));
    Serial.println(value);

    int result = transmit_packet((uint8_t*)&p, sizeof(p));

    if (result != 0) {
        Serial.print(F("[Beacon] ✗ Event TX failed, code: "));
        Serial.println(result);
    }

    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    return (result == 0) ? 1 : 0;
}

uint8_t beacon_service_flight_events(uint32_t now_ms, uint32_t now_s, uint8_t transmit_fast)
{
    uint8_t sent = 0;
    for (uint8_t i = 0; i < FLIGHT_EVENT_Q_DEPTH; i++) {
        if (event_q[i].left != 0 &&
            (int32_t)(now_ms - event_q[i].due_ms) >= 0) {
            if (beacon_transmit_flight_event(event_q[i].code, event_q[i].value,
                                             now_s, transmit_fast)) {
                event_q[i].left--;
                event_q[i].due_ms = now_ms + FLIGHT_EVENT_SPACING_MS;
                sent = 1;
            } else {
                /* TX failed: back off one spacing and retry; the repeat
                 * budget (successes only) is preserved. */
                event_q[i].due_ms = now_ms + FLIGHT_EVENT_SPACING_MS;
            }
            break;  /* one packet per service call: radio is half-duplex */
        }
    }
    return sent;
}

/**
 * Boot identity packet: 28 bits of git hash + dirty-tree MSB. Goes out
 * once, right after the boot callsign; the receiver logs it as TXBOOT.
 */
uint8_t beacon_transmit_hello(uint32_t system_time_seconds, uint8_t transmit_fast) {
    (void)system_time_seconds;   /* uptime at boot is ~0 - cut from V2 wire */
    HelloPacket_t p;
    p.packet_type = PACKET_TYPE_HELLO;
    p.rocket_id   = (uint8_t)ROCKET_ID;
    p.fw_hash     = (uint32_t)GIT_HASH_HEX | ((uint32_t)GIT_DIRTY_FLAG << 31);

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    int result = transmit_packet((uint8_t*)&p, sizeof(p));
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    if (result == 0) {
        Serial.print(F("[Beacon] HELLO fw=0x"));
        Serial.println(p.fw_hash, HEX);
    }
    return (result == 0) ? 1 : 0;
}

/**
 * Running maxima recap. While airborne the loop calls this every
 * MAXIMA_TX_INTERVAL_MS; one more copy goes out as the landing latch
 * trips, so the ground's last-heard row is the flight's whole envelope.
 */
uint8_t beacon_transmit_maxima(uint32_t system_time_seconds, uint8_t transmit_fast) {
    MaximaPacket_t p;
    flight_maxima_get(&p);
    p.packet_type = PACKET_TYPE_MAXIMA;
    p.rocket_id   = (uint8_t)ROCKET_ID;

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }

    Serial.print(F("[Beacon] Maxima alt="));
    Serial.print(p.max_alt_m);
    Serial.print(F("m@"));
    Serial.print(p.t_maxalt_s);
    Serial.print(F("s v="));
    Serial.print(p.max_speed_cms / 100.0f, 1);
    Serial.println(F("m/s"));

    int result = transmit_packet((uint8_t*)&p, sizeof(p));
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    (void)system_time_seconds;
    return (result == 0) ? 1 : 0;
}

/**
 * Transmit callsign via LoRa beacon
 * @param transmit_fast If true, transmit callsign in fast mode
 */
void beacon_transmit_callsign(uint8_t transmit_fast) {
    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    
    /* "CALLSIGN-<rocket id> CH<active channel>". No commas, so the receiver
     * parser classifies it as a callsign packet (must stay under its
     * 16-char callsign limit: "KE0MZS-2 CH3" = 12). The channel reflects the
     * backup jumper, letting the operator confirm airframe AND frequency. */
    char callsign_msg[24];
    snprintf(callsign_msg, sizeof(callsign_msg), "%s-%u CH%u",
             BEACON_CALLSIGN, (unsigned)ROCKET_ID, (unsigned)radio_get_channel());
    
    // Transmit callsign
    Serial.print("[Beacon] Transmitting callsign: ");
    Serial.println(callsign_msg);
    transmit_string(callsign_msg);
    
    
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
}

/**
 * Transmit a fused (EKF) position + velocity packet.
 *
 * Pulls the latest fused snapshot from the nav layer, packs it into a
 * FusedPosPacket_t and transmits via the radio.  If the nav layer hasn't
 * anchored yet (no first GPS fix), returns 0 without transmitting.
 */
uint8_t beacon_transmit_fused_data(uint32_t system_time_seconds, uint8_t transmit_fast) {
    (void)system_time_seconds;

    NavFused_t f;
    nav_get_fused(&f);
    if (!f.valid) {
        return 0;
    }
    /* Non-finite guard: ENCODE_COORD and clamp_i16's lroundf are undefined
     * behavior on NaN/Inf, and a poisoned filter must not reach the wire
     * as garbage coordinates. nav_update_from_gps already rejects
     * non-finite fixes; this is the last line of defense. */
    if (!isfinite(f.lat_deg) || !isfinite(f.lon_deg) || !isfinite(f.alt_m) ||
        !isfinite(f.v_n) || !isfinite(f.v_e) || !isfinite(f.v_d)) {
        return 0;
    }

    /* Saturate velocity to int16 cm/s (±327 m/s).  Real rockets in this
     * project's envelope stay well within that. */
    auto clamp_i16 = [](float v) -> int16_t {
        if (v >  327.0f) return  32700;
        if (v < -327.0f) return -32700;
        return (int16_t)lroundf(v * 100.0f);
    };

    FusedPosPacket_t p;
    memset(&p, 0, sizeof(p));
    p.packet_type = PACKET_TYPE_FUSED;
    p.latitude    = ENCODE_COORD(f.lat_deg);
    p.longitude   = ENCODE_COORD(f.lon_deg);
    /* Quarter-meter resolution with a 500 m MSL floor offset. Negative
     * altitudes below the floor clamp to 0 rather than wrap. */
    float alt_qm_f = (f.alt_m + FUSED_ALT_FLOOR_M) * FUSED_ALT_SCALE;
    if (alt_qm_f < 0.0f) alt_qm_f = 0.0f;
    if (alt_qm_f > 65535.0f) alt_qm_f = 65535.0f;
    p.alt_qm      = (uint16_t)lroundf(alt_qm_f);
    p.v_n_cms     = clamp_i16(f.v_n);
    p.v_e_cms     = clamp_i16(f.v_e);
    p.v_d_cms     = clamp_i16(f.v_d);

    /* Bit 0 marks the V3 wire layout (distinguishes 19-byte V3 from the
     * legacy 19-byte V1, which has age_ds where V3 keeps flags). */
    uint8_t flags = FUSED_FLAG_V3;
    if (f.gps_fresh)       flags |= FUSED_FLAG_GPS_FRESH;
    if (f.imu_healthy)     flags |= FUSED_FLAG_IMU_HEALTHY;
    if (f.dead_reckoning)  flags |= FUSED_FLAG_DEAD_RECKONING;
    if (f.sensor_degraded) flags |= FUSED_FLAG_SENSOR_DEGRADED;
    /* Innovation-gate rejections since the previous fused packet - the
     * field-visible hint of nav trouble. Delta, not a latch. */
    static uint32_t last_gps_rejects = 0;
    uint32_t rejects = nav_get_gps_rejects();
    if (rejects != last_gps_rejects) {
        last_gps_rejects = rejects;
        flags |= FUSED_FLAG_GATE_REJECT;
    }
    /* Level-based query - see note in beacon_transmit_gps_data_binary about
     * why launch_detect_is_launched() (one-shot) must NOT be used here. */
    if (launch_detect_get_state() == LAUNCH_STATE_CONFIRMED) {
        flags |= FUSED_FLAG_LAUNCH_DETECTED;
    }
    /* Landing latch: recovery is the workflow that most needs certainty.
     * The RX nav page shows LANDED and the SD log row carries the flag,
     * instead of the operator inferring touchdown from the cadence drop
     * to battery-save. */
    if (launch_detect_has_landed()) flags |= FUSED_FLAG_LANDED;
    p.flags = flags;
    /* V2 wire byte: same airframe ID carried by the raw GPS stream and the
     * heartbeat - the receiver's foreign-beacon filter keys off it. */
    p.rocket_id = (uint8_t)ROCKET_ID;

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }

    /* One-shot diagnostic: sizeof should be 19. If the compiler added any
     * padding despite __attribute__((packed)) the RX parser (which assumes
     * 19 bytes) will reject the packet - loud log helps us notice. */
    static bool size_logged = false;
    if (!size_logged) {
        Serial.print(F("[Beacon] sizeof(FusedPosPacket_t)="));
        Serial.println((int)sizeof(p));
        size_logged = true;
    }

    /* Hex-dump every N-th fused packet so we can correlate the decoded
     * v_n/v_e/v_d values below against the on-wire bytes the RX receives. */
    static uint16_t hexdump_counter = 0;
    if ((hexdump_counter++ % 10) == 0) {
        Serial.print(F("[Beacon] FUS bytes:"));
        const uint8_t *b = (const uint8_t*)&p;
        for (size_t i = 0; i < sizeof(p); i++) {
            Serial.print(' ');
            if (b[i] < 0x10) Serial.print('0');
            Serial.print(b[i], HEX);
        }
        Serial.print(F("  vn_cms=")); Serial.print((int)p.v_n_cms);
        Serial.print(F(" ve_cms=")); Serial.print((int)p.v_e_cms);
        Serial.print(F(" alt_qm=")); Serial.print((unsigned)p.alt_qm);
        Serial.print(F(" flags=0x")); Serial.println(p.flags, HEX);
    }

    int result = transmit_packet((uint8_t*)&p, sizeof(p));

    if (result != 0) {
        Serial.print(F("[Beacon] ✗ Fused TX failed, code: "));
        Serial.println(result);
    }

    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    return (result == 0) ? 1 : 0;
}
