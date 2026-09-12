/**
 * @file test_flight_profile.cpp
 * @brief Simulated flight through the real nav + launch_detect + beacon
 *        layers chained together - the closest we get to a field day on
 *        the bench.
 *
 * Drives a scripted IMU event stream (push_accel / push_rotvec into the
 * BNO085 stub queue), scripted GPS fixes into nav.cpp, and reads the
 * fused packets beacon.cpp builds. The point is INTERACTION coverage:
 * the spot where individual module tests can't see (pad anchor ->
 * sustained accel launch -> fused flags visible on the wire -> GPS hole
 * -> rescue -> landing latch) all in one continuous run like a flight.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "include/gps.h"
#include "include/radio.h"
#include "include/nav.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / peripheral fakes                                          */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
WireClass Wire;

static unsigned long now_ms = 100000;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

/* BNO08x stub knobs */
bool bno_begin_result  = true;
bool bno_enable_result = true;
int  bno_enable_fail_sensor = 0;
bool bno_was_reset     = false;
int  bno_enable_calls  = 0;
int  bno_begin_calls   = 0;
int  bno_last_report   = -1;
sh2_SensorValue_t bno_events[32];
int bno_event_head = 0;
int bno_event_tail = 0;

/* Radio fakes: capture fused packets */
static uint8_t  tx_buf[32];
static size_t   tx_len = 0;
static int      tx_result = 0;
static uint32_t tx_count = 0;
static uint32_t radio_enables  = 0;
static uint32_t radio_disables = 0;

void radio_enable(void)  { radio_enables++; }
void radio_disable(void) { radio_disables++; }
uint8_t radio_get_channel(void) { return 0; }

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
    return transmit_packet((const uint8_t*)str, strlen(str) + 1);
}

/* GPS fakes (beacon.cpp links against them even if we drive nav directly) */
static GPSCoordinates_t fake_coords;
const GPSCoordinates_t* gps_get_current_coordinates(void) { return &fake_coords; }
uint8_t gps_get_health(void) { return HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0); }
float gps_nmea_to_decimal(const char *n, char d)
{
    double v = atof(n);
    int deg = (int)(v / 100.0);
    double out = deg + (v - deg*100.0) / 60.0;
    if (d == 'S' || d == 'W') out = -out;
    return (float)out;
}

/* ------------------------------------------------------------------ */
/* Real modules wired together (this is the point of the scenario)     */
/* ------------------------------------------------------------------ */

#include "../firmware/launch_detect.cpp"
#include "../firmware/nav.cpp"
#include "../firmware/ekf.cpp"
#include "../firmware/beacon.cpp"

/* ------------------------------------------------------------------ */
/* Scenario scripting helpers                                          */
/* ------------------------------------------------------------------ */

static void push_accel(float x, float y, float z)
{
    sh2_SensorValue_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.sensorId = SH2_LINEAR_ACCELERATION;
    ev.un.linearAcceleration.x = x;
    ev.un.linearAcceleration.y = y;
    ev.un.linearAcceleration.z = z;
    bno_events[bno_event_tail++ % 32] = ev;
}

static void push_rotvec(float w, float i, float j, float k)
{
    sh2_SensorValue_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.sensorId = SH2_ROTATION_VECTOR;
    ev.un.rotationVector.real = w;
    ev.un.rotationVector.i    = i;
    ev.un.rotationVector.j    = j;
    ev.un.rotationVector.k    = k;
    bno_events[bno_event_tail++ % 32] = ev;
}

/* One main-loop-equivalent iteration at +ms: drain sensor events
 * (launch_detect does that internally) and run the EKF predict step. */
static void tick(uint32_t ms)
{
    now_ms += ms;
    launch_detect_update((uint32_t)(now_ms/1000), (uint32_t)(now_ms%1000));
    nav_predict();
}

static void refill_idle_sensors(void)
{
    /* Healthy streaming: rotation vector + quiet accel every 20 ms. The
     * queue drains per tick; push a burst sized to outlast the next tick. */
    push_accel(0.0f, 0.0f, 0.0f);
    push_rotvec(1.0f, 0.0f, 0.0f, 0.0f);
}

static const uint8_t* tick_and_fused_tx(uint32_t ms)   /* returns last flags byte */
{
    refill_idle_sensors();
    tick(ms);
    tx_len = 0;
    beacon_transmit_fused_data((uint32_t)(now_ms/1000), 1);
    if (tx_len < FUSED_PACKET_SIZE) return NULL;
    return &tx_buf[FUSED_PACKET_SIZE - 1];
}

static void feed_gps(float lat, float lon, float alt)
{
    refill_idle_sensors();
    launch_detect_update((uint32_t)(now_ms/1000), (uint32_t)(now_ms%1000));
    nav_update_from_gps(lat, lon, alt, 8, 1);
}

#define PAD_LAT 39.8900004f
#define PAD_LON (-104.8850002f)
#define PAD_ALT 1650.0f

/* Shared per-scenario reset - nav + launch-detect state reset cleanly,
 * simulated clock rewound forward-never-rewind. */
static void scenario_reset(void)
{
    bno_event_head = bno_event_tail = 0;
    launch_detect_init();
    nav_init();
    tx_count = 0;
    tx_len = 0;
}

/* One iteration of "main loop is healthy" while the GPS module reports
 * NO fix (fix_quality 0 - the altitude/speed lockout mode) */
static void tick_no_fix(uint32_t ms)
{
    refill_idle_sensors();
    tick(ms);
}

/* ------------------------------------------------------------------ */

TEST(test_profile_pad_to_landing)
{
    scenario_reset();

    /* ---- PAD: anchor + quiet healthy flag ------------------------- */
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);
    tick(1000);
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);      /* candidate + confirm */
    CHECK(nav_is_valid() == true);

    const uint8_t *fl = tick_and_fused_tx(600);
    CHECK(fl != NULL);
    CHECK((*fl & FUSED_FLAG_GPS_FRESH)   != 0);
    CHECK((*fl & FUSED_FLAG_IMU_HEALTHY) != 0);
    CHECK((*fl & FUSED_FLAG_LAUNCH_DETECTED) == 0);
    CHECK((*fl & FUSED_FLAG_LANDED)      == 0);
    CHECK(nav_get_gps_rejects() == 0);

    /* ---- BOOST: sustained accel beyond threshold -> launch -------- */
    now_ms += 2000;   /* settle */
    for (int i = 0; i < 8; i++) {     /* 8 x 50 ms = 400 ms >> 100 ms */
        push_accel(0, 0, 25.0f);      /* 25 m/s^2 > LAUNCH threshold */
        tick(50);
    }
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(launch_detect_is_launched() == true);   /* one-shot edge */
    CHECK(launch_detect_is_launched() == false);

    fl = tick_and_fused_tx(600);
    CHECK(fl != NULL);
    CHECK((*fl & FUSED_FLAG_LAUNCH_DETECTED) != 0);

    /* ---- MID-FLIGHT GPS HOLE: 20 s of radio/Windshield silence ---- */
    for (int i = 0; i < 40; i++) tick(500);   /* no gps feed */
    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.dead_reckoning == true);
    CHECK(f.gps_fresh == false);

    /* ---- FIXES RETURN AT THE ROCKET'S ACTUAL SPOT: rescue pair ---- *
     * The rescue requires two motion-consistent consecutive fixes; the
     * first armed by the hole, the second accepted. */
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);
    f.gps_fresh = 1;
    now_ms += 1000;
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);
    nav_get_fused(&f);
    CHECK(f.gps_fresh == true);          /* accepted again */
    CHECK(fabsf(f.lat_deg - PAD_LAT) < 1e-4f);
    now_ms += 1000;
    fl = tick_and_fused_tx(600);
    CHECK(fl != NULL);
    CHECK((*fl & FUSED_FLAG_DEAD_RECKONING) == 0);

    /* ---- DESCENT + LAND: quiet accel + stable GPS alt ------------- */
    /* Landing requires quiet accel AND GPS altitude stable LAND_QUIET_S.
     * Feed once per second like firmware.ino does. */
    uint32_t s = (uint32_t)(now_ms/1000);
    bool landed = false;
    for (int i = 0; i < LAND_QUIET_S + 5; i++) {
        feed_gps(PAD_LAT, PAD_LON, PAD_ALT);   /* stable */
        refill_idle_sensors();                 /* total_accel ~0 again */
        landed = landing_detect_update(PAD_ALT, true, ++s) || landed;
        tick(1000);
    }
    CHECK(landed);
    CHECK(launch_detect_has_landed() == true);

    fl = tick_and_fused_tx(600);
    CHECK(fl != NULL);
    CHECK((*fl & FUSED_FLAG_LANDED) != 0);

    /* The recovery phases consumed the radio cleanly throughout */
    CHECK(tx_count > 0);
    CHECK(radio_disables == 0 && radio_enables == 0);   /* fast path used */
}

TEST(test_profile_gps_lockout_at_altitude)
{
    /* Launch-day failure: the GPS module's altitude/speed lockout (or a
     * boost-phase flinch) means NMEA keeps 1 Hz flowing but fix_quality
     * drops to 0 for the whole burn. Design behavior: stop sending raw GPS,
     * fused continues as dead-reckoning, and when fixes come back the
     * rescue pair re-acquires without a permanent wedge. */
    scenario_reset();

    /* Anchored on the pad */
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);
    tick(1000);
    feed_gps(PAD_LAT, PAD_LON, PAD_ALT);
    CHECK(nav_is_valid() == true);

    /* Launch (quickly, via the accel path) */
    now_ms += 2000;
    for (int i = 0; i < 8; i++) { push_accel(0, 0, 25.0f); tick(50); }
    CHECK(launch_detect_is_launched() == true);

    /* Lockout: GPS keeps ticking at 1 Hz but fix is gone for a while */
    refill_idle_sensors();
    tick_no_fix(500);
    NavFused_t f;
    /* Actually update the nav view through the packet path so DR shows */
    for (int i = 0; i < 60; i++) tick_no_fix(500);   /* 30 s without fix */
    nav_get_fused(&f);
    CHECK(f.dead_reckoning == true);
    CHECK(f.gps_fresh == false);

    /* Position telem continues (this is what saves a ballistic story) */
    const uint8_t *fl = tick_and_fused_tx(600);
    CHECK(fl != NULL);
    CHECK((*fl & FUSED_FLAG_DEAD_RECKONING) != 0);
    CHECK((*fl & FUSED_FLAG_GPS_FRESH) == 0);

    /* Fixes return - rescue pair accepted, position snaps to truth */
    feed_gps(PAD_LAT + 0.0002f, PAD_LON, PAD_ALT);
    now_ms += 1000;
    feed_gps(PAD_LAT + 0.0002f, PAD_LON, PAD_ALT);
    nav_get_fused(&f);
    CHECK(f.gps_fresh == true);
    CHECK(fabsf(f.lat_deg - (PAD_LAT + 0.0002f)) < 1e-4f);
    CHECK(f.v_n < 5.0f && f.v_e < 5.0f);   /* no phantom velocity survives */
}

int main(void)
{
    run_test_profile_pad_to_landing();
    run_test_profile_gps_lockout_at_altitude();
    return TEST_SUMMARY();
}
