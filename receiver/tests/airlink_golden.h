/*
 * airlink_golden.h - byte-exact reference packets for TX<->RX airlink tests.
 *
 * One canonical fused packet, generated from the production TX encoders:
 *   lat = 39.8899999 deg (enc 398900000)    alt = 1655.43 m (8622 quarter-m)
 *   lon = -104.885168 deg (enc -1048851712) age = 7 ds
 *   v_n = 12.34 m/s   v_e = -3.21 m/s   v_d = -55.0 m/s
 *   flags = FUSED_FLAG_GPS_FRESH | FUSED_FLAG_IMU_HEALTHY (0x60)
 *   rocket_id = 0 (the ROCKET_ID default test builds link against)
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
#define AIRLINK_FUSED_AGE_DS    7
#define AIRLINK_FUSED_ROCKET_ID 0           /* ROCKET_ID default in test builds */

static const uint8_t AIRLINK_FUSED_GOLDEN[20] = {
    0x04,                     /* PACKET_TYPE_FUSED            */
    0x20, 0xBB, 0xC6, 0x17,   /* latitude  = 398900000        */
    0x00, 0xCB, 0x7B, 0xC1,   /* longitude = -1048851712      */
    0xAE, 0x21,               /* alt_qm    = 8622 (1655.5 m)  */
    0xD2, 0x04,               /* v_n_cms   = 1234             */
    0xBF, 0xFE,               /* v_e_cms   = -321             */
    0x84, 0xEA,               /* v_d_cms   = -5500            */
    0x07,                     /* age_ds                      */
    0x60,                     /* GPS_FRESH | IMU_HEALTHY      */
    AIRLINK_FUSED_ROCKET_ID   /* V2 rocket_id byte            */
};

#endif /* AIRLINK_GOLDEN_H */
