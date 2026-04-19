#include "include/beacon.h"
#include "include/config.h"
#include "include/mpu_config.h"
#include "include/radio.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
    packet.altitude = (int16_t)altitude;
    packet.satellites = (uint8_t)sat_count;
    
    // Pack flags
    packet.flags = 0;
    if (launch_detect_is_launched()) {
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
 * Transmit callsign via LoRa beacon
 * @param transmit_fast If true, transmit callsign in fast mode
 */
void beacon_transmit_callsign(uint8_t transmit_fast) {
    if (!transmit_fast) {
        radio_enable();
        delay(10);
    }
    
    // Transmit callsign
    Serial.print("[Beacon] Transmitting callsign: ");
    Serial.println(BEACON_CALLSIGN);
    transmit_string(BEACON_CALLSIGN);
    
#if INCLUDE_CARRIAGE_RETURNS
    transmit_string("\r\n");
#endif
    
    if (!transmit_fast) {
        delay(1);
        radio_disable();
    }
}
