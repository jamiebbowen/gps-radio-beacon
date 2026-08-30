/**
 * @file test_beacon.cpp
 * @brief Host-side unit tests for the transmitter beacon module.
 *
 * Fakes the radio (capturing every transmitted packet/string), GPS layer,
 * launch detector and nav/EKF layer to exercise all four wire formats the
 * beacon produces:
 *   - ASCII CSV GPS packets   (full + fast, hemisphere signs)
 *   - Binary GPS packets      (13 bytes, altitude clamping, flag bits)
 *   - Heartbeats              (rate limiting, uptime saturation)
 *   - Callsign strings        ("KE0MZS-<id> CH<n>")
 *   - Fused EKF packets       (21 bytes, velocity clamping, flag bits)
 * plus every GPS-rejection path (no fix, <4 sats, bad altitude).
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include "include/gps.h"
#include "include/radio.h"
#include "include/nav.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino core fakes                                                  */
/* ------------------------------------------------------------------ */

FakeSerial Serial;

static unsigned long now_ms = 10000;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }

/* ------------------------------------------------------------------ */
/* Radio fakes: capture everything that would go over the air          */
/* ------------------------------------------------------------------ */

static uint8_t  tx_buf[256];
static size_t   tx_len          = 0;
static uint32_t tx_count        = 0;
static int      tx_result       = 0;      /* 0 = RadioLib success */
static uint32_t radio_enables   = 0;
static uint32_t radio_disables  = 0;
static uint8_t  fake_channel    = 2;

void radio_enable(void)  { radio_enables++; }
void radio_disable(void) { radio_disables++; }
uint8_t radio_get_channel(void) { return fake_channel; }

int transmit_packet(const uint8_t *data, size_t length)
{
    if (length <= sizeof(tx_buf)) {
        memcpy(tx_buf, data, length);
        tx_len = length;
    }
    tx_count++;
    return tx_result;
}

int transmit_string(const char *str)
{
    return transmit_packet((const uint8_t *)str, strlen(str) + 1);
}

/* ------------------------------------------------------------------ */
/* GPS / launch / nav fakes                                            */
/* ------------------------------------------------------------------ */

static uint8_t fake_gps_health = 0x01;    /* HB_GPS_ACQUIRING-ish */
uint8_t gps_get_health(void) { return fake_gps_health; }

float gps_nmea_to_decimal(const char *nmea_coord, char direction)
{
    /* Reference conversion: ddmm.mmmm -> decimal degrees */
    double v = atof(nmea_coord);
    int deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double out = deg + min / 60.0;
    if (direction == 'S' || direction == 'W') out = -out;
    return (float)out;
}

static launch_state_t fake_launch_state = LAUNCH_STATE_IDLE;
launch_state_t launch_detect_get_state(void) { return fake_launch_state; }

static NavFused_t fake_fused;
void nav_get_fused(NavFused_t *out) { *out = fake_fused; }

/* Include the module under test AFTER the fakes (it only needs their
 * declarations from the headers; definitions resolve at link within
 * this translation unit). */
#include "../firmware/beacon.cpp"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void reset_tx(void)
{
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_len = 0;
    tx_count = 0;
    tx_result = 0;
    radio_enables = 0;
    radio_disables = 0;
    Serial.log_len = 0;
    Serial.log[0] = '\0';
}

/** A canonical valid full fix: 39°53.40284'N, 104°53.11007'W */
static GPSCoordinates_t valid_coords(void)
{
    GPSCoordinates_t c;
    memset(&c, 0, sizeof(c));
    strcpy(c.lat, "3953.40284");
    strcpy(c.lon, "10453.11007");
    c.lat_dir = 'N';
    c.lon_dir = 'W';
    c.valid = 1;
    c.fix_quality = 1;
    strcpy(c.satellites, "8");
    strcpy(c.altitude, "1655.4");
    return c;
}

static int32_t get_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ------------------------------------------------------------------ */
/* ASCII GPS packet tests                                              */
/* ------------------------------------------------------------------ */

TEST(test_ascii_full_packet)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();

    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 1);
    CHECK(tx_count == 1);
    /* West longitude gets a '-' sign; full packet carries sats */
    CHECK(strcmp((const char *)tx_buf, "3953.40284,-10453.11007,1655.4,8") == 0);
    /* Slow path power-cycles the radio around the TX */
    CHECK(radio_enables == 1 && radio_disables == 1);
}

TEST(test_ascii_fast_packet_and_south)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();
    c.lat_dir = 'S';
    c.lon_dir = 'E';

    CHECK(beacon_transmit_gps_data(&c, 100, 1) == 1);
    /* Fast packet: no sats field, radio stays enabled (LAUNCH phase) */
    CHECK(strcmp((const char *)tx_buf, "-3953.40284,10453.11007,1655.4") == 0);
    CHECK(radio_enables == 0 && radio_disables == 0);
}

TEST(test_ascii_rejection_paths)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();

    c.valid = 0;
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    c = valid_coords();
    strcpy(c.lat, "0");                       /* the "0" placeholder fix */
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    c = valid_coords();
    strcpy(c.satellites, "3");                /* < 4 sats */
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    c = valid_coords();
    c.fix_quality = 0;                        /* no fix */
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    c = valid_coords();
    strcpy(c.altitude, "60000");              /* > 50 km */
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    c = valid_coords();
    strcpy(c.altitude, "-600");               /* < -500 m */
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);

    CHECK(tx_count == 0);                     /* nothing ever hit the air */
}

TEST(test_ascii_tx_failure_reported)
{
    reset_tx();
    tx_result = -707;                         /* RadioLib error */
    GPSCoordinates_t c = valid_coords();
    CHECK(beacon_transmit_gps_data(&c, 100, 0) == 0);
    CHECK(strstr(Serial.log, "-707") != NULL);
    tx_result = 0;
}

/* ------------------------------------------------------------------ */
/* Binary GPS packet tests                                             */
/* ------------------------------------------------------------------ */

TEST(test_binary_packet_fields)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();

    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 1);
    CHECK(tx_len == 13);
    CHECK(tx_buf[0] == PACKET_TYPE_GPS);

    /* lat 39.8900473, lon -104.8851678 (deg * 1e7) */
    int32_t lat = get_i32_le(&tx_buf[1]);
    int32_t lon = get_i32_le(&tx_buf[5]);
    CHECK(lat > 398900000 && lat < 398901000);
    CHECK(lon < -1048851000 && lon > -1048852000);

    CHECK(get_i16_le(&tx_buf[9]) == 1655);    /* altitude, whole meters */
    CHECK(tx_buf[11] == 8);                   /* sats */
    /* On the pad: no launch bit, good-fix bit + fix type 1 */
    CHECK(tx_buf[12] == (FLAG_FIX_QUALITY_GOOD | 0x01));
}

TEST(test_binary_altitude_clamps)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();
    strcpy(c.altitude, "40000");              /* valid (> -50 km bound) but > int16 */
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);
    CHECK(get_i16_le(&tx_buf[9]) == 32767);   /* clamped, not wrapped */

    reset_tx();
    c = valid_coords();
    strcpy(c.altitude, "-40000");             /* below the -500 m sanity floor */
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 0);  /* rejected outright */
}

TEST(test_binary_launch_flag)
{
    reset_tx();
    fake_launch_state = LAUNCH_STATE_CONFIRMED;
    GPSCoordinates_t c = valid_coords();
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);
    CHECK((tx_buf[12] & FLAG_LAUNCH_DETECTED) != 0);
    fake_launch_state = LAUNCH_STATE_IDLE;
}

TEST(test_binary_rejection_and_tx_failure)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();

    c.valid = 0;
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    c = valid_coords();
    strcpy(c.lat, "0");
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    c = valid_coords();
    strcpy(c.satellites, "2");
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    c = valid_coords();
    c.fix_quality = 0;
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    c = valid_coords();
    strcpy(c.altitude, "99999");
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    CHECK(tx_count == 0);

    tx_result = -2;
    c = valid_coords();
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 0) == 0);
    tx_result = 0;
}

/* ------------------------------------------------------------------ */
/* Heartbeat tests                                                     */
/* ------------------------------------------------------------------ */

TEST(test_heartbeat_contents_and_rate_limit)
{
    reset_tx();
    GPSCoordinates_t c = valid_coords();
    strcpy(c.satellites, "2");                /* acquiring */
    c.fix_quality = 0;
    fake_gps_health = 0x21;                   /* health nibble + resets */

    CHECK(beacon_transmit_heartbeat(&c, 42, 0) == 1);
    CHECK(tx_len == sizeof(HeartbeatPacket_t));
    const HeartbeatPacket_t *hb = (const HeartbeatPacket_t *)tx_buf;
    CHECK(hb->packet_type == PACKET_TYPE_HEARTBEAT);
    CHECK(hb->rocket_id   == ROCKET_ID);
    CHECK(hb->channel     == fake_channel);
    CHECK(hb->satellites  == 2);
    CHECK(hb->fix_quality == 0);
    CHECK(hb->uptime_s    == 42);
    CHECK(hb->gps_health  == 0x21);

    /* Within HEARTBEAT_INTERVAL_SEC: rate limited, no TX */
    uint32_t count_before = tx_count;
    now_ms += 1000;
    CHECK(beacon_transmit_heartbeat(&c, 43, 0) == 0);
    CHECK(tx_count == count_before);

    /* After the interval: transmits again */
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL;
    CHECK(beacon_transmit_heartbeat(&c, 48, 0) == 1);
    CHECK(tx_count == count_before + 1);
}

TEST(test_heartbeat_null_coords_and_saturation)
{
    reset_tx();
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL + 1;

    /* NULL coords: sats/fix default to 0; uptime saturates at 65535 */
    CHECK(beacon_transmit_heartbeat(NULL, 100000UL, 1) == 1);
    const HeartbeatPacket_t *hb = (const HeartbeatPacket_t *)tx_buf;
    CHECK(hb->satellites == 0 && hb->fix_quality == 0);
    CHECK(hb->uptime_s == 65535);
    CHECK(radio_enables == 0);                /* fast mode: no power cycle */
}

TEST(test_heartbeat_tx_failure_does_not_consume_slot)
{
    reset_tx();
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL + 1;

    tx_result = -1;
    CHECK(beacon_transmit_heartbeat(NULL, 1, 0) == 0);
    tx_result = 0;

    /* Failed TX must not update the rate limiter: the next attempt goes
     * out immediately instead of waiting another full interval */
    CHECK(beacon_transmit_heartbeat(NULL, 2, 0) == 1);
}

/* ------------------------------------------------------------------ */
/* Callsign tests                                                      */
/* ------------------------------------------------------------------ */

TEST(test_callsign_format)
{
    reset_tx();
    fake_channel = 3;
    beacon_transmit_callsign(0);
    CHECK(strcmp((const char *)tx_buf, BEACON_CALLSIGN "-0 CH3") == 0);
    CHECK(strlen((const char *)tx_buf) < 16);  /* RX callsign field limit */
    CHECK(strchr((const char *)tx_buf, ',') == NULL);  /* parsed as callsign */
    CHECK(radio_enables == 1 && radio_disables == 1);

    reset_tx();
    beacon_transmit_callsign(1);               /* fast: radio left alone */
    CHECK(radio_enables == 0 && radio_disables == 0);
    fake_channel = 2;
}

/* ------------------------------------------------------------------ */
/* Fused packet tests                                                  */
/* ------------------------------------------------------------------ */

TEST(test_fused_not_anchored_no_tx)
{
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));   /* valid = false */
    CHECK(beacon_transmit_fused_data(100, 0) == 0);
    CHECK(tx_count == 0);
}

TEST(test_fused_packet_fields)
{
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid       = true;
    fake_fused.lat_deg     = 39.8900473f;
    fake_fused.lon_deg     = -104.8851678f;
    fake_fused.alt_m       = 1655.43f;
    fake_fused.v_n         = 12.34f;
    fake_fused.v_e         = -3.21f;
    fake_fused.v_d         = -55.0f;
    fake_fused.age_ds      = 7;
    fake_fused.gps_fresh   = true;
    fake_fused.imu_healthy = true;

    CHECK(beacon_transmit_fused_data(100, 0) == 1);
    CHECK(tx_len == 21);
    CHECK(tx_buf[0] == PACKET_TYPE_FUSED);
    CHECK(get_i32_le(&tx_buf[1]) > 398900000);
    CHECK(get_i32_le(&tx_buf[5]) < -1048851000);
    CHECK(get_i32_le(&tx_buf[9]) == 165543);       /* alt in cm */
    CHECK(get_i16_le(&tx_buf[13]) == 1234);        /* vN cm/s */
    CHECK(get_i16_le(&tx_buf[15]) == -321);        /* vE cm/s */
    CHECK(get_i16_le(&tx_buf[17]) == -5500);       /* vD cm/s */
    CHECK(tx_buf[19] == 7);                        /* age ds */
    CHECK(tx_buf[20] == (FUSED_FLAG_GPS_FRESH | FUSED_FLAG_IMU_HEALTHY));
}

TEST(test_fused_velocity_clamps_and_flags)
{
    reset_tx();
    fake_fused.v_n = 500.0f;                       /* > +327 m/s */
    fake_fused.v_e = -500.0f;                      /* < -327 m/s */
    fake_fused.gps_fresh = false;
    fake_fused.dead_reckoning = true;
    fake_launch_state = LAUNCH_STATE_CONFIRMED;

    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    CHECK(get_i16_le(&tx_buf[13]) == 32700);
    CHECK(get_i16_le(&tx_buf[15]) == -32700);
    CHECK((tx_buf[20] & FUSED_FLAG_DEAD_RECKONING) != 0);
    CHECK((tx_buf[20] & FUSED_FLAG_LAUNCH_DETECTED) != 0);
    CHECK((tx_buf[20] & FUSED_FLAG_GPS_FRESH) == 0);
    fake_launch_state = LAUNCH_STATE_IDLE;
}

TEST(test_fused_hexdump_cadence_and_tx_failure)
{
    reset_tx();
    /* The hex dump fires every 10th packet; run enough to hit both the
     * dump and no-dump branches of the counter */
    for (int i = 0; i < 11; i++) {
        CHECK(beacon_transmit_fused_data(100 + i, 1) == 1);
    }

    tx_result = -3;
    CHECK(beacon_transmit_fused_data(200, 0) == 0);
    CHECK(strstr(Serial.log, "Fused TX failed") != NULL);
    tx_result = 0;
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_ascii_full_packet();
    run_test_ascii_fast_packet_and_south();
    run_test_ascii_rejection_paths();
    run_test_ascii_tx_failure_reported();
    run_test_binary_packet_fields();
    run_test_binary_altitude_clamps();
    run_test_binary_launch_flag();
    run_test_binary_rejection_and_tx_failure();
    run_test_heartbeat_contents_and_rate_limit();
    run_test_heartbeat_null_coords_and_saturation();
    run_test_callsign_format();
    run_test_heartbeat_tx_failure_does_not_consume_slot();
    run_test_fused_not_anchored_no_tx();
    run_test_fused_packet_fields();
    run_test_fused_velocity_clamps_and_flags();
    run_test_fused_hexdump_cadence_and_tx_failure();

    return TEST_SUMMARY();
}
