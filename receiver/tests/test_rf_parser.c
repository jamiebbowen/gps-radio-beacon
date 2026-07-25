/**
 * @file test_rf_parser.c
 * @brief Host-side unit tests for the receiver RF packet parser.
 *
 * Exercises all three wire formats accepted from the transmitter:
 *   1. ASCII CSV packets  (RF_Parser_ParseAsciiPacket)
 *   2. Binary GPS packets (RF_Parser_ParseBinaryPacket, 13 bytes)
 *   3. Fused EKF packets  (RF_Parser_ParseFusedPacket, 21 bytes)
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "rf_parser.h"
#include "packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Packet-building helpers                                             */
/* ------------------------------------------------------------------ */

static void put_i32_le(uint8_t *buf, int32_t v) {
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_i16_le(uint8_t *buf, int16_t v) {
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

/** Build a 13-byte binary GPS packet as the transmitter would. */
static void build_gps_packet(uint8_t *buf, double lat_deg, double lon_deg,
                             int16_t alt_m, uint8_t sats, uint8_t flags) {
    buf[0] = PACKET_TYPE_GPS;
    put_i32_le(&buf[1], (int32_t)(lat_deg * 10000000.0));
    put_i32_le(&buf[5], (int32_t)(lon_deg * 10000000.0));
    put_i16_le(&buf[9], alt_m);
    buf[11] = sats;
    buf[12] = flags;
}

/** Build a 21-byte fused EKF packet as the transmitter would. */
static void build_fused_packet(uint8_t *buf, double lat_deg, double lon_deg,
                               int32_t alt_cm, int16_t vn_cms, int16_t ve_cms,
                               int16_t vd_cms, uint8_t age_ds, uint8_t flags) {
    buf[0] = PACKET_TYPE_FUSED;
    put_i32_le(&buf[1], (int32_t)(lat_deg * 10000000.0));
    put_i32_le(&buf[5], (int32_t)(lon_deg * 10000000.0));
    put_i32_le(&buf[9], alt_cm);
    put_i16_le(&buf[13], vn_cms);
    put_i16_le(&buf[15], ve_cms);
    put_i16_le(&buf[17], vd_cms);
    buf[19] = age_ds;
    buf[20] = flags;
}

/* ------------------------------------------------------------------ */
/* ASCII parser tests                                                  */
/* ------------------------------------------------------------------ */

TEST(test_ascii_null_and_empty_rejected) {
    RF_Parser_Reset();
    uint32_t nulls = 0;

    CHECK(RF_Parser_ParseAsciiPacket(NULL) == RF_PARSER_ERROR);
    CHECK(RF_Parser_ParseAsciiPacket("") == RF_PARSER_ERROR);

    RF_Parser_GetDetailedFailures(&nulls, NULL, NULL, NULL, NULL);
    CHECK(nulls == 2);
}

TEST(test_ascii_fast_packet) {
    RF_Parser_Reset();
    /* Fast packet: lat,lon,alt - transmitted continuously during LAUNCH */
    CHECK(RF_Parser_ParseAsciiPacket("3953.40284,-10407.38970,1234.5") == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    /* 39°53.40284' = 39.8900473°, 104°07.38970' = 104.1231617° (west) */
    CHECK_NEAR(gps.latitude, 39.8900473, 1e-4);
    CHECK_NEAR(gps.longitude, -104.1231617, 1e-4);
    CHECK_NEAR(gps.altitude, 1234.5, 1e-3);
    CHECK(gps.satellites == 0);        /* unknown in fast packets */
    CHECK(gps.fix == 1);               /* assumed valid */
    CHECK(gps.launch_detected == 1);   /* fast packets imply launch */
}

TEST(test_ascii_full_packet) {
    RF_Parser_Reset();
    /* Full packet: lat,lon,alt,sats - transmitted in non-launch states */
    CHECK(RF_Parser_ParseAsciiPacket("3953.40284,10407.38970,1671.7,12") == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK_NEAR(gps.latitude, 39.8900473, 1e-4);
    CHECK_NEAR(gps.longitude, 104.1231617, 1e-4);
    CHECK(gps.satellites == 12);
    CHECK(gps.launch_detected == 0);   /* full packets imply pre/post-launch */
}

TEST(test_ascii_southern_western_hemisphere) {
    RF_Parser_Reset();
    CHECK(RF_Parser_ParseAsciiPacket("-3345.55000,-5830.25000,10.0") == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    /* Buenos Aires-ish: 33°45.55'S, 58°30.25'W */
    CHECK_NEAR(gps.latitude, -(33.0 + 45.55 / 60.0), 1e-4);
    CHECK_NEAR(gps.longitude, -(58.0 + 30.25 / 60.0), 1e-4);
}

TEST(test_ascii_callsign_packet) {
    RF_Parser_Reset();
    CHECK(RF_Parser_ParseAsciiPacket("KD0ABC") == RF_PARSER_OK);

    GPS_Data gps;
    char callsign[RF_PARSER_MAX_CALLSIGN_LEN] = {0};
    CHECK(RF_Parser_GetParsedData(&gps, callsign, sizeof(callsign), NULL) == 1);
    CHECK(strcmp(callsign, "KD0ABC") == 0);
}

TEST(test_ascii_insufficient_fields_rejected) {
    RF_Parser_Reset();
    uint32_t insufficient = 0;

    /* Two fields only (has a comma so it isn't treated as a callsign) */
    CHECK(RF_Parser_ParseAsciiPacket("3953.40284,1234") == RF_PARSER_ERROR);

    RF_Parser_GetDetailedFailures(NULL, &insufficient, NULL, NULL, NULL);
    CHECK(insufficient == 1);
}

TEST(test_ascii_invalid_coordinates_rejected) {
    RF_Parser_Reset();

    /* Latitude > 90° (9100.0 NMEA = 91°) */
    CHECK(RF_Parser_ParseAsciiPacket("9100.00000,-10407.38970,100") == RF_PARSER_ERROR);
    /* Longitude > 180° */
    CHECK(RF_Parser_ParseAsciiPacket("3953.40284,-18100.00000,100") == RF_PARSER_ERROR);
    /* Zero coordinates ("Null Island" guard) */
    CHECK(RF_Parser_ParseAsciiPacket("0000.00000,0000.00000,100") == RF_PARSER_ERROR);
    /* Minutes field >= 60 is not valid NMEA */
    CHECK(RF_Parser_ParseAsciiPacket("3999.99999,-10407.38970,100") == RF_PARSER_ERROR);
    /* Coordinate string too short to be DDMM.MM */
    CHECK(RF_Parser_ParseAsciiPacket("123.45,-10407.38970,100") == RF_PARSER_ERROR);

    uint32_t ok = 0, attempts = 0, fails = 0;
    RF_Parser_GetDiagnostics(&attempts, &ok, &fails);
    CHECK(attempts == 5);
    CHECK(ok == 0);
    CHECK(fails == 5);
}

TEST(test_ascii_last_packet_bookkeeping) {
    RF_Parser_Reset();
    char buf[RF_PARSER_BUFFER_SIZE];

    /* A failed parse should still be visible as the last RAW packet... */
    CHECK(RF_Parser_ParseAsciiPacket("garbage,with,commas") == RF_PARSER_ERROR);
    CHECK(RF_Parser_GetLastRawPacket(buf, sizeof(buf)) > 0);
    CHECK(strcmp(buf, "garbage,with,commas") == 0);

    /* ...but must NOT become the last VALID packet. */
    CHECK(RF_Parser_ParseAsciiPacket("3953.40284,-10407.38970,100") == RF_PARSER_OK);
    CHECK(RF_Parser_GetLastValidPacket(buf, sizeof(buf)) > 0);
    CHECK(strcmp(buf, "3953.40284,-10407.38970,100") == 0);

    /* Truncation must stay within the caller's buffer and null-terminate. */
    char tiny[8];
    uint16_t n = RF_Parser_GetLastValidPacket(tiny, sizeof(tiny));
    CHECK(n == sizeof(tiny) - 1);
    CHECK(tiny[sizeof(tiny) - 1] == '\0');
}

/* ------------------------------------------------------------------ */
/* Binary GPS parser tests                                             */
/* ------------------------------------------------------------------ */

TEST(test_binary_valid_packet) {
    RF_Parser_Reset();
    uint8_t pkt[13];
    build_gps_packet(pkt, 39.8900750, -105.1155100, 1671, 12,
                     FLAG_LAUNCH_DETECTED | FLAG_FIX_QUALITY_GOOD | 0x01);

    CHECK(RF_Parser_ParseBinaryPacket(pkt, sizeof(pkt)) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK_NEAR(gps.latitude, 39.8900750, 1e-5);
    CHECK_NEAR(gps.longitude, -105.1155100, 1e-5);
    CHECK_NEAR(gps.altitude, 1671.0, 1e-3);
    CHECK(gps.satellites == 12);
    CHECK(gps.launch_detected == 1);
    CHECK(gps.fix == 1);
    CHECK(gps.is_fused == 0);
}

TEST(test_binary_negative_altitude) {
    RF_Parser_Reset();
    uint8_t pkt[13];
    build_gps_packet(pkt, 51.5000000, -0.1200000, -100, 8, 0x41);

    CHECK(RF_Parser_ParseBinaryPacket(pkt, sizeof(pkt)) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK_NEAR(gps.altitude, -100.0, 1e-3);
    CHECK(gps.launch_detected == 0);
}

TEST(test_binary_malformed_rejected) {
    RF_Parser_Reset();
    uint8_t pkt[13];
    build_gps_packet(pkt, 39.89, -105.11, 100, 8, 0);

    CHECK(RF_Parser_ParseBinaryPacket(NULL, 13) == RF_PARSER_ERROR);  /* NULL   */
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 12) == RF_PARSER_ERROR);   /* short  */
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 14) == RF_PARSER_ERROR);   /* long   */

    pkt[0] = PACKET_TYPE_TELEMETRY;                                   /* type   */
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 13) == RF_PARSER_ERROR);

    build_gps_packet(pkt, 95.0, -105.11, 100, 8, 0);                  /* lat>90 */
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 13) == RF_PARSER_ERROR);

    build_gps_packet(pkt, 39.89, -190.0, 100, 8, 0);                  /* lon    */
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 13) == RF_PARSER_ERROR);
}

/* ------------------------------------------------------------------ */
/* Fused packet parser tests                                           */
/* ------------------------------------------------------------------ */

TEST(test_fused_valid_packet) {
    RF_Parser_Reset();
    uint8_t pkt[FUSED_PACKET_SIZE];
    build_fused_packet(pkt, 39.8900750, -105.1155100,
                       167170,           /* 1671.70 m           */
                       1234, -567, -89,  /* vN/vE/vD in cm/s    */
                       7,
                       FUSED_FLAG_LAUNCH_DETECTED | FUSED_FLAG_GPS_FRESH
                       | FUSED_FLAG_IMU_HEALTHY);

    CHECK(RF_Parser_ParseFusedPacket(pkt, sizeof(pkt)) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK_NEAR(gps.latitude, 39.8900750, 1e-5);
    CHECK_NEAR(gps.longitude, -105.1155100, 1e-5);
    CHECK_NEAR(gps.altitude, 1671.70, 1e-2);
    CHECK_NEAR(gps.v_north, 12.34, 1e-3);
    CHECK_NEAR(gps.v_east, -5.67, 1e-3);
    CHECK_NEAR(gps.v_down, -0.89, 1e-3);
    CHECK(gps.is_fused == 1);
    CHECK(gps.fused_gps_fresh == 1);
    CHECK(gps.fused_imu_healthy == 1);
    CHECK(gps.fused_dr == 0);
    CHECK(gps.fused_age_ds == 7);
    CHECK(gps.launch_detected == 1);
}

TEST(test_fused_dead_reckoning_flag) {
    RF_Parser_Reset();
    uint8_t pkt[FUSED_PACKET_SIZE];
    build_fused_packet(pkt, 39.89, -105.11, 100000, 0, 0, 0, 255,
                       FUSED_FLAG_DEAD_RECKONING);

    CHECK(RF_Parser_ParseFusedPacket(pkt, sizeof(pkt)) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK(gps.fused_dr == 1);
    CHECK(gps.fused_gps_fresh == 0);
    CHECK(gps.fused_age_ds == 255);
}

TEST(test_fused_malformed_rejected) {
    RF_Parser_Reset();
    uint8_t pkt[FUSED_PACKET_SIZE];
    build_fused_packet(pkt, 39.89, -105.11, 0, 0, 0, 0, 0, 0);

    CHECK(RF_Parser_ParseFusedPacket(NULL, FUSED_PACKET_SIZE) == RF_PARSER_ERROR);
    CHECK(RF_Parser_ParseFusedPacket(pkt, FUSED_PACKET_SIZE - 1) == RF_PARSER_ERROR);

    pkt[0] = PACKET_TYPE_GPS;
    CHECK(RF_Parser_ParseFusedPacket(pkt, FUSED_PACKET_SIZE) == RF_PARSER_ERROR);

    build_fused_packet(pkt, -95.0, -105.11, 0, 0, 0, 0, 0, 0);
    CHECK(RF_Parser_ParseFusedPacket(pkt, FUSED_PACKET_SIZE) == RF_PARSER_ERROR);
}

/* ------------------------------------------------------------------ */
/* Cross-packet interaction tests                                      */
/* ------------------------------------------------------------------ */

TEST(test_fused_does_not_clobber_gps_fix) {
    /* `fix` drives the "3D"/"NoFix" UI label and is only meaningful for raw
     * GPS packets; fused packets must leave it alone (regression guard for
     * the display-flapping bug documented in rf_parser.c). */
    RF_Parser_Reset();
    uint8_t gps_pkt[13], fused_pkt[FUSED_PACKET_SIZE];
    build_gps_packet(gps_pkt, 39.89, -105.11, 100, 10, 0x43);  /* fix = 3 */
    build_fused_packet(fused_pkt, 39.90, -105.12, 10000, 100, 0, 0, 3,
                       FUSED_FLAG_GPS_FRESH);

    CHECK(RF_Parser_ParseBinaryPacket(gps_pkt, 13) == RF_PARSER_OK);
    CHECK(RF_Parser_ParseFusedPacket(fused_pkt, FUSED_PACKET_SIZE) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK(gps.fix == 3);  /* still the raw-GPS value */
}

TEST(test_gps_does_not_clobber_fused_velocity) {
    /* Velocities come only from fused packets and must persist across raw
     * GPS packets so the nav display doesn't flicker to 0.0 m/s. */
    RF_Parser_Reset();
    uint8_t gps_pkt[13], fused_pkt[FUSED_PACKET_SIZE];
    build_fused_packet(fused_pkt, 39.90, -105.12, 10000, 1500, -250, 40, 3, 0);
    build_gps_packet(gps_pkt, 39.89, -105.11, 100, 10, 0x41);

    CHECK(RF_Parser_ParseFusedPacket(fused_pkt, FUSED_PACKET_SIZE) == RF_PARSER_OK);
    CHECK(RF_Parser_ParseBinaryPacket(gps_pkt, 13) == RF_PARSER_OK);

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 1);
    CHECK(gps.is_fused == 0);            /* correctly marked as raw GPS */
    CHECK_NEAR(gps.v_north, 15.0, 1e-3); /* but velocity persisted      */
    CHECK_NEAR(gps.v_east, -2.5, 1e-3);
    CHECK_NEAR(gps.v_down, 0.4, 1e-3);
}

TEST(test_reset_clears_state) {
    uint8_t pkt[13];
    build_gps_packet(pkt, 39.89, -105.11, 100, 10, 0x41);
    CHECK(RF_Parser_ParseBinaryPacket(pkt, 13) == RF_PARSER_OK);

    RF_Parser_Reset();

    GPS_Data gps;
    CHECK(RF_Parser_GetParsedData(&gps, NULL, 0, NULL) == 0);  /* no data */

    uint32_t attempts = 1, ok = 1, fails = 1;
    RF_Parser_GetDiagnostics(&attempts, &ok, &fails);
    CHECK(attempts == 0 && ok == 0 && fails == 0);
}

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_ascii_null_and_empty_rejected();
    run_test_ascii_fast_packet();
    run_test_ascii_full_packet();
    run_test_ascii_southern_western_hemisphere();
    run_test_ascii_callsign_packet();
    run_test_ascii_insufficient_fields_rejected();
    run_test_ascii_invalid_coordinates_rejected();
    run_test_ascii_last_packet_bookkeeping();
    run_test_binary_valid_packet();
    run_test_binary_negative_altitude();
    run_test_binary_malformed_rejected();
    run_test_fused_valid_packet();
    run_test_fused_dead_reckoning_flag();
    run_test_fused_malformed_rejected();
    run_test_fused_does_not_clobber_gps_fix();
    run_test_gps_does_not_clobber_fused_velocity();
    run_test_reset_clears_state();

    return TEST_SUMMARY();
}
