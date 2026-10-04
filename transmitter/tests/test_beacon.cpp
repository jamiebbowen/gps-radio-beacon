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
 *   - Fused EKF packets       (19 bytes, velocity clamping, flag bits)
 *   - Flight-event packets    (8 bytes: repeat queue, spacing, backoff)
 * plus every GPS-rejection path (no fix, <4 sats, bad altitude).
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>
#include "include/gps.h"
#include "include/radio.h"
#include "include/nav.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "test_harness.h"
#include "airlink_golden.h"

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
bool radio_is_transmitting(void) { return false; }
int  radio_poll_rx(uint8_t *out, size_t max_len, uint16_t timeout_ms)
{
    (void)out; (void)max_len; (void)timeout_ms;
    return 0;   /* always nothing on the wire in this harness */
}
int  radio_transmit_ack(uint8_t rocket_id, uint8_t cmd_code,
                        uint8_t seq_hi, uint8_t seq_lo, uint8_t echo)
{
    (void)rocket_id; (void)cmd_code; (void)seq_hi; (void)seq_lo; (void)echo;
    return 0;
}

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
static uint32_t fake_fix_age = 250;    /* plausible fix age for T0 test */
uint32_t gps_get_fix_age_ms(void) { return fake_fix_age; }

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

static bool fake_landed = false;
bool launch_detect_has_landed(void) { return fake_landed; }

static NavFused_t fake_fused;
void nav_get_fused(NavFused_t *out) { *out = fake_fused; }

/* launch_detect IMU accessors (in-flight IMU trace packet pulls these) */
static float fake_ax = 1.25f, fake_ay = 0.f, fake_az = -9.8f;
void launch_detect_get_accel_xyz(float *x, float *y, float *z)
{ if (x) *x = fake_ax; if (y) *y = fake_ay; if (z) *z = fake_az; }
static float fake_gx = 0.5f, fake_gy = 0.f, fake_gz = -0.75f;
void launch_detect_get_gyro_rads(float *x, float *y, float *z)
{ if (x) *x = fake_gx; if (y) *y = fake_gy; if (z) *z = fake_gz; }
static float fake_total_accel = 1.25f;
float launch_detect_get_current_accel(void) { return fake_total_accel; }

static uint32_t fake_gps_rejects = 0;
uint32_t nav_get_gps_rejects(void) { return fake_gps_rejects; }

/* beacon.cpp's V3 heartbeat reports the boot's reset cause; on real
 * hardware firmware.ino captures RSTC->RCAUSE into this global. */
volatile uint8_t g_boot_rcause = 0;

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
    CHECK(tx_len == GPS_PACKET_SIZE);
    CHECK(tx_len == 14);
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
    CHECK(tx_buf[13] == (uint8_t)ROCKET_ID);  /* V2 airframe ID */
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
    CHECK((tx_buf[12] & FLAG_LANDED) == 0);
    fake_launch_state = LAUNCH_STATE_IDLE;
}

TEST(test_landed_flag_both_packet_types)
{
    /* The landing latch rides both streams so the RX indicator can't
     * flap as GPS/FUS interleave (rx-side counterpart test lives in
     * test_rf_parser.c). */
    reset_tx();
    fake_landed = true;
    GPSCoordinates_t c = valid_coords();
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);
    CHECK((tx_buf[12] & FLAG_LANDED) != 0);

    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.lat_deg = 39.89;  fake_fused.lon_deg = -105.11;
    fake_fused.alt_m   = 1600.0; fake_fused.valid    = true;
    fake_fused.gps_fresh = true;
    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    CHECK(tx_buf[0] == PACKET_TYPE_FUSED);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_LANDED) != 0);
    fake_landed = false;
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

    CHECK(beacon_transmit_heartbeat(&c, 42, HEARTBEAT_INTERVAL_SEC, 0) == 1);
    CHECK(tx_len == sizeof(HeartbeatPacket_t));
    CHECK(sizeof(HeartbeatPacket_t) == HEARTBEAT_PACKET_SIZE);  /* =9 */
    const HeartbeatPacket_t *hb = (const HeartbeatPacket_t *)tx_buf;
    CHECK(hb->packet_type == PACKET_TYPE_HEARTBEAT);
    CHECK(hb->rocket_id   == ROCKET_ID);
    CHECK(hb->channel     == fake_channel);
    CHECK(hb->satellites  == 2);
    CHECK(hb->fix_quality == 0);
    CHECK(hb->uptime_s    == 42);
    CHECK(hb->gps_health  == 0x21);
    /* V3: reset cause travels with the heartbeat (WDT boot = 0x20) */
    g_boot_rcause = HB_RESET_WDT;
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL + 1;
    CHECK(beacon_transmit_heartbeat(&c, 50, HEARTBEAT_INTERVAL_SEC, 0) == 1);
    hb = (const HeartbeatPacket_t *)tx_buf;
    CHECK(hb->reset_info  == HB_RESET_WDT);
    g_boot_rcause = 0;

    /* Within the caller's interval: rate limited, no TX */
    uint32_t count_before = tx_count;
    now_ms += 1000;
    CHECK(beacon_transmit_heartbeat(&c, 43, HEARTBEAT_INTERVAL_SEC, 0) == 0);
    CHECK(tx_count == count_before);

    /* After the interval: transmits again */
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL;
    CHECK(beacon_transmit_heartbeat(&c, 48, HEARTBEAT_INTERVAL_SEC, 0) == 1);
    CHECK(tx_count == count_before + 1);

    /* The interval is the caller's, not the module's: a pad-cadence caller
     * (60 s) is still rate-limited 30 s in, a tighter caller is not. */
    count_before = tx_count;
    now_ms += 30 * 1000UL;
    CHECK(beacon_transmit_heartbeat(&c, 78, PRELAUNCH_HEARTBEAT_INTERVAL_SEC, 0) == 0);
    CHECK(tx_count == count_before);
}

TEST(test_heartbeat_null_coords_and_saturation)
{
    reset_tx();
    now_ms += HEARTBEAT_INTERVAL_SEC * 1000UL + 1;

    /* NULL coords: sats/fix default to 0; uptime saturates at 65535 */
    CHECK(beacon_transmit_heartbeat(NULL, 100000UL, HEARTBEAT_INTERVAL_SEC, 1) == 1);
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
    CHECK(beacon_transmit_heartbeat(NULL, 1, HEARTBEAT_INTERVAL_SEC, 0) == 0);
    tx_result = 0;

    /* Failed TX must not update the rate limiter: the next attempt goes
     * out immediately instead of waiting another full interval */
    CHECK(beacon_transmit_heartbeat(NULL, 2, HEARTBEAT_INTERVAL_SEC, 0) == 1);
}

TEST(test_launch_t0_packet_contents)
{
    reset_tx();
    fake_fix_age = 2500;                              /* 2.5 s fix age */
    CHECK(beacon_transmit_launch_t0(123, 0) == 1);
    CHECK(tx_len == sizeof(LaunchT0Packet_t));
    CHECK(sizeof(LaunchT0Packet_t) == LAUNCH_T0_PACKET_SIZE);  /* =6 */
    const LaunchT0Packet_t *t0 = (const LaunchT0Packet_t *)tx_buf;
    CHECK(t0->packet_type == PACKET_TYPE_LAUNCH_T0);
    CHECK(t0->rocket_id   == ROCKET_ID);
    CHECK(t0->uptime_s    == 123);
    CHECK(t0->age_ds      == 25);                     /* 2.5 s in deciseconds */
}

/* ------------------------------------------------------------------ */
/* Certified flight-event packets + retransmit queue                   */
/* ------------------------------------------------------------------ */

TEST(test_flight_event_packet_and_repeats)
{
    reset_tx();
    CHECK(sizeof(FlightEventPacket_t) == FLIGHT_EVENT_PACKET_SIZE);  /* =8 */
    beacon_queue_flight_event(FLIGHT_EVENT_APOGEE, 2234, FLIGHT_EVENT_REPEATS);

    /* First copy goes out immediately, on the first service pass */
    CHECK(beacon_service_flight_events(now_ms, 42, 1) == 1);
    CHECK(tx_count == 1);
    const FlightEventPacket_t *ev = (const FlightEventPacket_t *)tx_buf;
    CHECK(ev->packet_type == PACKET_TYPE_FLIGHT_EVENT);
    CHECK(ev->rocket_id   == ROCKET_ID);
    CHECK(ev->code        == FLIGHT_EVENT_APOGEE);
    CHECK(ev->spare       == 0);
    CHECK(ev->uptime_s    == 42);
    CHECK(ev->value       == 2234);

    /* Copies 2..N gated by FLIGHT_EVENT_SPACING_MS, then the slot drains */
    CHECK(beacon_service_flight_events(now_ms + FLIGHT_EVENT_SPACING_MS - 1, 43, 1) == 0);
    CHECK(tx_count == 1);
    CHECK(beacon_service_flight_events(now_ms + FLIGHT_EVENT_SPACING_MS, 43, 1) == 1);
    CHECK(beacon_service_flight_events(now_ms + 2 * FLIGHT_EVENT_SPACING_MS, 44, 1) == 1);
    CHECK(beacon_service_flight_events(now_ms + 3 * FLIGHT_EVENT_SPACING_MS, 45, 1) == 0);
    CHECK(tx_count == 3);                            /* exactly REPEATS copies */
    CHECK(radio_enables == 0);                       /* fast mode: no power cycle */
}

TEST(test_flight_event_tx_failure_backs_off)
{
    reset_tx();
    tx_result = -2;
    beacon_queue_flight_event(FLIGHT_EVENT_ANOM_BALLISTIC, 4000, 2);
    CHECK(beacon_service_flight_events(now_ms, 100, 1) == 0);
    CHECK(tx_count == 1);                            /* the attempt happened */
    tx_result = 0;

    /* Retry postponed by one spacing, not a busy re-attempt */
    CHECK(beacon_service_flight_events(now_ms + 1, 100, 1) == 0);
    CHECK(tx_count == 1);
    CHECK(beacon_service_flight_events(now_ms + FLIGHT_EVENT_SPACING_MS, 100, 1) == 1);
    CHECK(tx_count == 2);

    /* Drain the last repeat so the static queue enters the next test empty */
    CHECK(beacon_service_flight_events(now_ms + 2 * FLIGHT_EVENT_SPACING_MS, 101, 1) == 1);
    CHECK(tx_count == 3);
}

TEST(test_flight_event_queue_overflow_counted)
{
    reset_tx();
    uint32_t drops0 = beacon_flight_event_dropped();
    for (int i = 0; i < 8; i++) {
        beacon_queue_flight_event(FLIGHT_EVENT_MAIN, (int16_t)i, 3);
    }
    beacon_queue_flight_event(FLIGHT_EVENT_MAIN, 99, 3);   /* 9th: full */
    CHECK(beacon_flight_event_dropped() == drops0 + 1);

    /* Everything queued still drains at the spacing cadence: 8 x 3 copies */
    uint32_t sent = 0;
    for (uint32_t t = 0; t < 40; t++) {
        sent += beacon_service_flight_events(now_ms + t * FLIGHT_EVENT_SPACING_MS,
                                             (uint32_t)t, 1);
    }
    CHECK(sent == 24);
}

TEST(test_imu_trace_packet_contents)
{
    reset_tx();
    fake_ax = 0.55f; fake_ay = -1.25f; fake_az = 0.0f;
    fake_gx = 0.01f; fake_gy = -0.02f; fake_gz = 0.0f;
    fake_total_accel = 1.35f;

    CHECK(beacon_transmit_imu_trace(0) == 1);
    CHECK(tx_len == sizeof(ImuTracePacket_t));
    CHECK(sizeof(ImuTracePacket_t) == IMU_TRACE_PACKET_SIZE);  /* =16 */
    const ImuTracePacket_t *im = (const ImuTracePacket_t *)tx_buf;
    CHECK(im->packet_type == PACKET_TYPE_IMU);
    CHECK(im->rocket_id   == ROCKET_ID);
    CHECK(im->accel_x_cg  == 55);
    CHECK(im->accel_y_cg  == -125);
    CHECK(im->accel_z_cg  == 0);
    CHECK(im->gyro_x_cds  == 57);                     /* 0.01 rad/s */
    CHECK(im->gyro_y_cds  == -115);                   /* -0.02 rad/s */
    CHECK(im->peak_accel_mg == 1350);

    /* Peak resets after transmit; next one captures fresh only */
    fake_total_accel = 0.5f;
    now_ms += 1000;
    CHECK(beacon_transmit_imu_trace(0) == 1);
    im = (const ImuTracePacket_t *)tx_buf;
    CHECK(im->peak_accel_mg == 500);
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
    CHECK(tx_len == FUSED_PACKET_SIZE);
    CHECK(tx_len == 20);
    CHECK(tx_buf[0] == PACKET_TYPE_FUSED);
    CHECK(get_i32_le(&tx_buf[1]) > 398900000);
    CHECK(get_i32_le(&tx_buf[5]) < -1048851000);
    /* alt: 1655.43 m -> (1655.43 + 500) * 4 = 8621.72 -> 8622 quarter-m */
    CHECK((uint16_t)(tx_buf[9] | (tx_buf[10] << 8)) == 8622);
    CHECK(get_i16_le(&tx_buf[11]) == 1234);        /* vN cm/s */
    CHECK(get_i16_le(&tx_buf[13]) == -321);        /* vE cm/s */
    CHECK(get_i16_le(&tx_buf[15]) == -5500);       /* vD cm/s */
    CHECK(tx_buf[17] == 7);                        /* age ds */
    CHECK(tx_buf[18] == (FUSED_FLAG_GPS_FRESH | FUSED_FLAG_IMU_HEALTHY));
    CHECK(tx_buf[19] == (uint8_t)ROCKET_ID);       /* V2 airframe ID */
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
    CHECK(get_i16_le(&tx_buf[11]) == 32700);
    CHECK(get_i16_le(&tx_buf[13]) == -32700);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_DEAD_RECKONING) != 0);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_LAUNCH_DETECTED) != 0);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_GPS_FRESH) == 0);
    fake_launch_state = LAUNCH_STATE_IDLE;
}

TEST(test_fused_sensor_degraded_flag)
{
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid           = true;
    fake_fused.lat_deg         = 39.89f;
    fake_fused.lon_deg         = -105.11f;
    fake_fused.alt_m           = 1655.0f;
    fake_fused.sensor_degraded = true;

    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    /* Exact byte: ONLY bit 2 may be set. SENSOR_DEGRADED is the wire-format
     * pin for the IMU-dead signal - the receiver decodes it at the same
     * offset (see its mirror test in receiver/tests/test_rf_parser.c). */
    CHECK(tx_buf[offsetof(FusedPosPacket_t, flags)] == FUSED_FLAG_SENSOR_DEGRADED);
}

TEST(test_fused_wire_format_pin)
{
    /* The receiver carries its own copy of packet_format.h; a drift between
     * the two files breaks the link silently. These literal CHECKs (and the
     * identical block in receiver/tests/test_rf_parser.c) make a drift a
     * red test on whichever side changed without updating the other. */
    CHECK(sizeof(FusedPosPacket_t) == 20);
    CHECK(FUSED_PACKET_SIZE == 20 && FUSED_PACKET_SIZE_V1 == 19);
    CHECK(offsetof(FusedPosPacket_t, age_ds)    == 17);
    CHECK(offsetof(FusedPosPacket_t, flags)     == 18);
    CHECK(offsetof(FusedPosPacket_t, rocket_id) == 19);

    CHECK(sizeof(BinaryGPSPacket_t) == 14);
    CHECK(GPS_PACKET_SIZE == 14 && GPS_PACKET_SIZE_V1 == 13);
    CHECK(offsetof(BinaryGPSPacket_t, flags)     == 12);
    CHECK(offsetof(BinaryGPSPacket_t, rocket_id) == 13);

    CHECK(FUSED_FLAG_LAUNCH_DETECTED == 0x80);
    CHECK(FUSED_FLAG_GPS_FRESH       == 0x40);
    CHECK(FUSED_FLAG_IMU_HEALTHY     == 0x20);
    CHECK(FUSED_FLAG_DEAD_RECKONING  == 0x10);
    CHECK(FUSED_FLAG_LANDED          == 0x08);
    CHECK(FUSED_FLAG_SENSOR_DEGRADED == 0x04);
    CHECK(FUSED_FLAG_GATE_REJECT     == 0x02);
    CHECK(FUSED_FLAG_RESERVED_MASK   == 0x01);

    CHECK(FLAG_LOW_SATS              == 0x20);

    /* FLIGHT_EVENT (apogee/drogue/main/landed + anomaly codes) */
    CHECK(sizeof(FlightEventPacket_t) == 8);
    CHECK(FLIGHT_EVENT_PACKET_SIZE == 8);
    CHECK(PACKET_TYPE_FLIGHT_EVENT == 0x0A);
    CHECK(offsetof(FlightEventPacket_t, code)     == 2);
    CHECK(offsetof(FlightEventPacket_t, uptime_s) == 4);
    CHECK(offsetof(FlightEventPacket_t, value)    == 6);
    CHECK(FLIGHT_EVENT_APOGEE           == 0x01);
    CHECK(FLIGHT_EVENT_DROGUE           == 0x02);
    CHECK(FLIGHT_EVENT_MAIN             == 0x03);
    CHECK(FLIGHT_EVENT_LANDED           == 0x04);
    CHECK(FLIGHT_EVENT_ANOM_BALLISTIC   == 0x10);
    CHECK(FLIGHT_EVENT_ANOM_TUMBLE      == 0x11);
    CHECK(FLIGHT_EVENT_ANOM_SENSOR_LOSS == 0x12);
    CHECK(FLIGHT_EVENT_ANOM_GPS_OUTAGE  == 0x13);
    CHECK(FLIGHT_EVENT_ANOM_REBOOT      == 0x14);
}

TEST(test_fused_gate_reject_delta)
{
    /* GATE_REJECT (bit 1) is a per-packet delta: it must appear exactly on
     * packets where nav_get_gps_rejects grew, then clear again. */
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid = true;
    fake_fused.lat_deg = 39.89f; fake_fused.lon_deg = -105.11f;

    fake_gps_rejects = 0;
    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_GATE_REJECT) == 0);

    fake_gps_rejects = 3;                     /* gate rejected some fixes */
    CHECK(beacon_transmit_fused_data(101, 1) == 1);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_GATE_REJECT) != 0);

    CHECK(beacon_transmit_fused_data(102, 1) == 1);
    CHECK((tx_buf[offsetof(FusedPosPacket_t, flags)] & FUSED_FLAG_GATE_REJECT) == 0);
    /* Settle the tracker so no later test sees a stale delta */
    fake_gps_rejects = 0;
    CHECK(beacon_transmit_fused_data(103, 1) == 1);
}

TEST(test_3sat_2d_fix_post_landing_only)
{
    /* Recovery rule: after the landing latch, a 3-sat 2D fix still flies
     * (canopy landings) tagged FLAG_LOW_SATS; before landing, 3 sats is
     * still a rejection - noisy flight positions must not ship. */
    reset_tx();
    fake_landed = false;
    GPSCoordinates_t c = valid_coords();
    strcpy(c.satellites, "3");
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 0);   /* pre-landing */
    CHECK(tx_count == 0);

    reset_tx();
    fake_landed = true;
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);   /* post-landing */
    CHECK(tx_buf[12] & FLAG_LOW_SATS);
    CHECK(tx_buf[12] & FLAG_LANDED);
    CHECK(tx_buf[11] == 3);                                    /* sats field honest */

    /* LOW_SATS must never mark healthy geometry */
    reset_tx();
    fake_landed = true;
    c = valid_coords();                                        /* 8 sats */
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);
    CHECK((tx_buf[12] & FLAG_LOW_SATS) == 0);

    /* Two sats is nonsense even landed: still rejected */
    reset_tx();
    strcpy(c.satellites, "2");
    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 0);
    fake_landed = false;
}

TEST(test_fused_altitude_floor_clamps)
{
    /* Below the -500 m MSL floor the quarter-meter field must clamp to 0,
     * never wrap to ~+16 km and scare the recovery crew. */
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid = true;
    fake_fused.lat_deg = 39.89;  fake_fused.lon_deg = -105.11;
    fake_fused.alt_m   = -800.0;
    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    CHECK((uint16_t)(tx_buf[9] | (tx_buf[10] << 8)) == 0);

    /* Exactly at the floor: encodes 0 as well, decodes back to -500 */
    fake_fused.alt_m = -500.0;
    CHECK(beacon_transmit_fused_data(101, 1) == 1);
    CHECK((uint16_t)(tx_buf[9] | (tx_buf[10] << 8)) == 0);

    /* Ceiling: 15883.75 m is the uint16 quarter-meter roof */
    fake_fused.alt_m = 50000.0;
    CHECK(beacon_transmit_fused_data(102, 1) == 1);
    CHECK((uint16_t)(tx_buf[9] | (tx_buf[10] << 8)) == 65535);
}

TEST(test_airlink_golden_gps_encode)
{
    /* Same tripwire as the fused golden, for the raw GPS stream: the
     * production encoder fed with the golden NMEA input must emit exactly
     * the golden V2 bytes the RX parser test decodes. */
    reset_tx();
    GPSCoordinates_t c = valid_coords();
    strcpy(c.lat, AIRLINK_GPS_NMEA_LAT); c.lat_dir = 'N';
    strcpy(c.lon, AIRLINK_GPS_NMEA_LON); c.lon_dir = 'W';
    strcpy(c.altitude, "1655.4");
    strcpy(c.satellites, "8");
    c.fix_quality = 1;
    fake_launch_state = LAUNCH_STATE_IDLE;
    fake_landed = false;

    CHECK(beacon_transmit_gps_data_binary(&c, 100, 1) == 1);
    CHECK(tx_len == sizeof(AIRLINK_GPS_GOLDEN));
    CHECK(memcmp(tx_buf, AIRLINK_GPS_GOLDEN, sizeof(AIRLINK_GPS_GOLDEN)) == 0);
}

TEST(test_airlink_golden_fused_encode)
{
    /* Byte-exact encode pin: the production beacon encoder must emit
     * exactly the bytes in airlink_golden.h, which the production RX
     * parser decodes on the other side (receiver/tests/test_rf_parser.c).
     * This is the end-to-end "TX bit4 vs RX bit3" tripwire: any drift on
     * either side fails exactly one suite. */
    reset_tx();
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid       = true;
    fake_fused.lat_deg     = AIRLINK_FUSED_LAT_DEG;
    fake_fused.lon_deg     = AIRLINK_FUSED_LON_DEG;
    fake_fused.alt_m       = AIRLINK_FUSED_ALT_M;
    fake_fused.v_n         = AIRLINK_FUSED_VN_CMS / 100.0f;
    fake_fused.v_e         = AIRLINK_FUSED_VE_CMS / 100.0f;
    fake_fused.v_d         = AIRLINK_FUSED_VD_CMS / 100.0f;
    fake_fused.age_ds      = AIRLINK_FUSED_AGE_DS;
    fake_fused.gps_fresh   = true;
    fake_fused.imu_healthy = true;

    CHECK(beacon_transmit_fused_data(100, 1) == 1);
    CHECK(tx_len == sizeof(AIRLINK_FUSED_GOLDEN));
    CHECK(memcmp(tx_buf, AIRLINK_FUSED_GOLDEN, sizeof(AIRLINK_FUSED_GOLDEN)) == 0);
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
    run_test_landed_flag_both_packet_types();
    run_test_binary_rejection_and_tx_failure();
    run_test_heartbeat_contents_and_rate_limit();
    run_test_heartbeat_null_coords_and_saturation();
    run_test_callsign_format();
    run_test_heartbeat_tx_failure_does_not_consume_slot();
    run_test_launch_t0_packet_contents();
    run_test_flight_event_packet_and_repeats();
    run_test_flight_event_tx_failure_backs_off();
    run_test_flight_event_queue_overflow_counted();
    run_test_imu_trace_packet_contents();
    run_test_fused_not_anchored_no_tx();
    run_test_fused_packet_fields();
    run_test_fused_velocity_clamps_and_flags();
    run_test_fused_sensor_degraded_flag();
    run_test_fused_wire_format_pin();
    run_test_fused_gate_reject_delta();
    run_test_3sat_2d_fix_post_landing_only();
    run_test_fused_altitude_floor_clamps();
    run_test_airlink_golden_gps_encode();
    run_test_airlink_golden_fused_encode();
    run_test_fused_hexdump_cadence_and_tx_failure();

    return TEST_SUMMARY();
}
