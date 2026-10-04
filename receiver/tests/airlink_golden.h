/*
 * airlink_golden.h - byte-exact reference packets for TX<->RX airlink tests.
 *
 * One canonical fused packet (V3, 19 bytes), generated from the production
 * TX encoders:
 *   lat = 39.8899999 deg (enc 398900000)    alt = 1655.43 m (8622 quarter-m)
 *   lon = -104.885168 deg (enc -1048851712)
 *   v_n = 12.34 m/s   v_e = -3.21 m/s   v_d = -55.0 m/s
 *   flags = FUSED_FLAG_GPS_FRESH | FUSED_FLAG_IMU_HEALTHY | FUSED_FLAG_V3
 *   rocket_id = 0 (the ROCKET_ID default test builds link against)
 *
 * V3 dropped the age_ds byte: the receiver derives staleness from
 * GPS_FRESH + inter-arrival time, and V2's 20th byte had pushed the
 * packet over the 987->1118 ms LoRa airtime step at SF10/BW62.5k.
 *
 * Both sides test against these same bytes:
 *   transmitter/tests/test_beacon.cpp  - production encode must produce them
 *   receiver/tests/test_rf_parser.c    - production decode must read them back
 * Any wire-format drift breaks exactly one of those suites on purpose.
 */
#ifndef AIRLINK_GOLDEN_H
#define AIRLINK_GOLDEN_H

#include <stdint.h>

#define AIRLINK_FUSED_LAT_DEG   39.89
#define AIRLINK_FUSED_LON_DEG   (-104.8851678)
#define AIRLINK_FUSED_ALT_M     1655.43f
#define AIRLINK_FUSED_VN_CMS    1234
#define AIRLINK_FUSED_VE_CMS    (-321)
#define AIRLINK_FUSED_VD_CMS    (-5500)
#define AIRLINK_FUSED_ROCKET_ID 0           /* ROCKET_ID default in test builds */

static const uint8_t AIRLINK_FUSED_GOLDEN[19] = {
    0x04,                     /* PACKET_TYPE_FUSED            */
    0x20, 0xBB, 0xC6, 0x17,   /* latitude  = 398900000        */
    0x00, 0xCB, 0x7B, 0xC1,   /* longitude = -1048851712      */
    0xAE, 0x21,               /* alt_qm    = 8622 (1655.5 m)  */
    0xD2, 0x04,               /* v_n_cms   = 1234             */
    0xBF, 0xFE,               /* v_e_cms   = -321             */
    0x84, 0xEA,               /* v_d_cms   = -5500            */
    0x61,                     /* GPS_FRESH | IMU_HEALTHY | V3 */
    AIRLINK_FUSED_ROCKET_ID   /* rocket_id                    */
};

/* Same coverage for the raw GPS stream (14-byte V2). Encoded from the
 * production beacon path fed with NMEA lat "3953.40000"N / lon
 * "10453.11007"W, alt 1655.4 m, 8 sats, fix quality 1 (pad state: no
 * launch, no landed bits). Lat/lon are the same point as the fused
 * golden, so both streams pin one canonical position. */
#define AIRLINK_GPS_NMEA_LAT   "3953.40000"
#define AIRLINK_GPS_NMEA_LON   "10453.11007"
#define AIRLINK_GPS_LAT_DEG    39.89
#define AIRLINK_GPS_LON_DEG    (-104.8851678)
#define AIRLINK_GPS_ALT_M      1655        /* 1655.4 m truncated            */
#define AIRLINK_GPS_SATS       8
#define AIRLINK_GPS_FLAGS      0x41        /* FIX_QUALITY_GOOD | fix type 1 */
#define AIRLINK_GPS_ROCKET_ID  AIRLINK_FUSED_ROCKET_ID

static const uint8_t AIRLINK_GPS_GOLDEN[14] = {
    0x01,                     /* PACKET_TYPE_GPS              */
    0x20, 0xBB, 0xC6, 0x17,   /* latitude  = 398900000        */
    0x00, 0xCB, 0x7B, 0xC1,   /* longitude = -1048851712      */
    0x77, 0x06,               /* altitude  = 1655             */
    AIRLINK_GPS_SATS,
    AIRLINK_GPS_FLAGS,
    AIRLINK_GPS_ROCKET_ID
};

#endif /* AIRLINK_GOLDEN_H */
