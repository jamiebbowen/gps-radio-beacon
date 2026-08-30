#include "include/beacon.h"
#include "include/config.h"
#include "include/mpu_config.h"
#include "include/radio.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "include/nav.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Checksum function removed - LoRa provides built-in CRC validation

/**
 * Transmit GPS data via LoRa beacon with proper framing and checksum
 * @param coords Pointer to GPS coordinates structure
 * @param system_time_seconds Current system time in seconds
 * @param transmit_fast If true, transmit GPS data in fast mode
 * @return 1 if transmission was successful, 0 if coordinates were invalid
 */
uint8_t beacon_transmit_gps_data(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast) {
    char packet[256];  // LoRa packet buffer (plenty of room)
    int pos = 0;
    
    // Check if we have valid GPS data
    if (!coords->valid || (coords->lat[0] == '0' && strlen(coords->lat) == 1)) {
        Serial.print(F("[Beacon] Rejecting GPS - valid="));
        Serial.print(coords->valid);
        Serial.print(F(" lat='"));
        Serial.print(coords->lat);
        Serial.print(F("' lon='"));
        Serial.print(coords->lon);
        Serial.println(F("'"));
        return 0;
    }
    
    // Require minimum 4 satellites for reliable fix
    int sat_count = atoi(coords->satellites);
    if (sat_count < 4) {
        Serial.print(F("[Beacon] Rejecting GPS - insufficient satellites: "));
        Serial.println(sat_count);
        return 0;
    }
    
    // Check GPS fix quality (GGA field 6)
    // 0=no fix, 1=GPS fix (SPS), 2=DGPS fix, 3=PPS fix, etc.
    if (coords->fix_quality < 1) {
        Serial.print(F("[Beacon] Rejecting GPS - no fix, quality: "));
        Serial.println(coords->fix_quality);
        return 0;
    }
    
    // Sanity check altitude (reasonable range: -500m to 50000m)
    float altitude = atof(coords->altitude);
    if (altitude < -500.0f || altitude > 50000.0f) {
        Serial.print(F("[Beacon] Rejecting GPS - unreasonable altitude: "));
        Serial.println(altitude);
        return 0;
    }
    
    Serial.println(F("[Beacon] GPS data valid, building packet..."));
    
    if (!transmit_fast) {
        // Enable radio
        radio_enable();
        delay(10);
    }

    // Build packet: simple CSV format - LoRa handles framing and CRC
    // Add latitude with sign
    if (coords->lat_dir == 'S') {
        packet[pos++] = '-';
    }
    if (strlen(coords->lat) > 0) {
        for (uint8_t i = 0; coords->lat[i] != '\0'; i++) {
            packet[pos++] = coords->lat[i];
        }
    }
    
    packet[pos++] = ',';
    
    // Add longitude with sign
    if (coords->lon_dir == 'W') {
        packet[pos++] = '-';
    }
    if (strlen(coords->lon) > 0) {
        for (uint8_t i = 0; coords->lon[i] != '\0'; i++) {
            packet[pos++] = coords->lon[i];
        }
    }
    
    packet[pos++] = ',';
    
    // Add altitude
    if (strlen(coords->altitude) > 0) {
        for (uint8_t i = 0; coords->altitude[i] != '\0'; i++) {
            packet[pos++] = coords->altitude[i];
        }
    }

    if (!transmit_fast) {
        packet[pos++] = ',';
        
        // Add satellite count (only in full packets)
        if (strlen(coords->satellites) > 0) {
            for (uint8_t i = 0; coords->satellites[i] != '\0'; i++) {
                packet[pos++] = coords->satellites[i];
            }
        }
    }

#if INCLUDE_CARRIAGE_RETURNS
    // Add newline (only in testing mode)
    packet[pos++] = '\r';
    packet[pos++] = '\n';
#endif
    
    // Null terminate
    packet[pos] = '\0';
    
    // Transmit via LoRa
    Serial.print("[Beacon] Transmitting GPS packet: ");
    Serial.println(packet);
    int result = transmit_string(packet);
    
    if (result != 0) {
        Serial.print(F("[Beacon] ✗ Transmission failed, error code: "));
        Serial.println(result);
    }
    
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
    
    return (result == 0) ? 1 : 0;  // RadioLib returns 0 on success
}

/**
 * Transmit GPS data in binary format (13 bytes)
 * @param coords Pointer to GPS coordinates structure
 * @param system_time_seconds Current system time in seconds
 * @param transmit_fast If true, transmit GPS data in fast mode
 * @return 1 if transmission was successful, 0 if coordinates were invalid
 */
uint8_t beacon_transmit_gps_data_binary(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast) {
    // Perform all the same validation checks as ASCII version
    if (!coords->valid || (coords->lat[0] == '0' && strlen(coords->lat) == 1)) {
        Serial.print(F("[Beacon] Rejecting GPS - valid="));
        Serial.print(coords->valid);
        Serial.print(F(" lat='"));
        Serial.print(coords->lat);
        Serial.print(F("' lon='"));
        Serial.print(coords->lon);
        Serial.println(F("'"));
        return 0;
    }
    
    int sat_count = atoi(coords->satellites);
    if (sat_count < 4) {
        Serial.print(F("[Beacon] Rejecting GPS - insufficient satellites: "));
        Serial.println(sat_count);
        return 0;
    }
    
    if (coords->fix_quality < 1) {
        Serial.print(F("[Beacon] Rejecting GPS - no fix, quality: "));
        Serial.println(coords->fix_quality);
        return 0;
    }
    
    float altitude = atof(coords->altitude);
    if (altitude < -500.0f || altitude > 50000.0f) {
        Serial.print(F("[Beacon] Rejecting GPS - unreasonable altitude: "));
        Serial.println(altitude);
        return 0;
    }
    
    Serial.println(F("[Beacon] GPS data valid, building binary packet..."));
    
    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    
    // Convert NMEA to decimal degrees
    float lat_decimal = gps_nmea_to_decimal(coords->lat, coords->lat_dir);
    float lon_decimal = gps_nmea_to_decimal(coords->lon, coords->lon_dir);
    
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
    if (coords->fix_quality >= 1) {
        packet.flags |= FLAG_FIX_QUALITY_GOOD;
    }
    packet.flags |= (coords->fix_quality & FLAG_FIX_TYPE_MASK);
    
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
uint8_t beacon_transmit_heartbeat(const GPSCoordinates_t* coords, uint32_t system_time_seconds, uint8_t transmit_fast) {
    /* Rate limit: the GPS TX path retries every second when there is no fix;
     * do not turn every retry into an RF transmission. */
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now_ms = millis();
    if (last_heartbeat_ms != 0 &&
        (now_ms - last_heartbeat_ms) < (uint32_t)HEARTBEAT_INTERVAL_SEC * 1000UL) {
        return 0;
    }
    
    HeartbeatPacket_t hb;
    hb.packet_type = PACKET_TYPE_HEARTBEAT;
    hb.rocket_id   = (uint8_t)ROCKET_ID;
    hb.channel     = radio_get_channel();
    hb.satellites  = coords ? (uint8_t)atoi(coords->satellites) : 0;
    hb.fix_quality = coords ? coords->fix_quality : 0;
    hb.uptime_s    = (system_time_seconds > 65535UL) ? 65535U
                                                     : (uint16_t)system_time_seconds;
    /* Tells the receiver WHY there is no fix: still acquiring (normal) vs
     * GPS UART silent / garbled (wiring, power, baud - go check the rocket
     * before it flies). Sats=0 alone cannot distinguish those. */
    hb.gps_health  = gps_get_health();
    
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
    
#if INCLUDE_CARRIAGE_RETURNS
    transmit_string("\r\n");
#endif
    
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
    p.altitude_cm = (int32_t)lroundf(f.alt_m * 100.0f);
    p.v_n_cms     = clamp_i16(f.v_n);
    p.v_e_cms     = clamp_i16(f.v_e);
    p.v_d_cms     = clamp_i16(f.v_d);
    p.age_ds      = f.age_ds;

    uint8_t flags = 0;
    if (f.gps_fresh)      flags |= FUSED_FLAG_GPS_FRESH;
    if (f.imu_healthy)    flags |= FUSED_FLAG_IMU_HEALTHY;
    if (f.dead_reckoning) flags |= FUSED_FLAG_DEAD_RECKONING;
    /* Level-based query - see note in beacon_transmit_gps_data_binary about
     * why launch_detect_is_launched() (one-shot) must NOT be used here. */
    if (launch_detect_get_state() == LAUNCH_STATE_CONFIRMED) {
        flags |= FUSED_FLAG_LAUNCH_DETECTED;
    }
    p.flags = flags;

    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }

    /* One-shot diagnostic: sizeof should be 21. If the compiler added any
     * padding despite __attribute__((packed)) the RX parser (which assumes
     * 21 bytes) will reject the packet - loud log helps us notice. */
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
        Serial.print(F(" alt_cm=")); Serial.print((long)p.altitude_cm);
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
