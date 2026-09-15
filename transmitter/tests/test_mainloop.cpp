/**
 * @file test_mainloop.cpp
 * @brief Host simulation of the REAL firmware.ino loop(): boots the sketch,
 *        then scripts GPS fixes, an IMU launch burst, descent, and landing,
 *        over minutes of simulated time.
 *
 * This is the last untested structural unit on the TX side: the module
 * tests cover the pieces, flight_cadence covers the pacing table, but the
 * composition glue (flag/heartbeat interplay at fix loss, state entries
 * forcing a TX, radio enable/disable choreography, callsign gating) only
 * lives in loop().
 *
 * Wire format checks ride along: every packet type on the air is counted
 * by type byte, so the sim asserts exact rates per phase.
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
#include "include/flight_cadence.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / peripheral fakes                                          */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
WireClass Wire;

static unsigned long sim_ms = 0;
unsigned long millis(void) { return sim_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

/* SAMD51 WDT + RSTC register fakes (firmware.ino pokes them directly).
 * SYNCBUSY reads 0 forever: writes never "synchronize" on the host. */
static struct {
    struct { struct { uint8_t ENABLE; } bit; } CTRLA;
    struct { struct { uint32_t ENABLE; uint32_t CLEAR; } bit; } SYNCBUSY;
    struct { struct { uint8_t PER; } bit; } CONFIG;
    struct { uint8_t reg; } CLEAR;
} fake_wdt;
#define WDT (&fake_wdt)
#define WDT_CONFIG_PER_CYC16384_Val  0x0B
#define WDT_CLEAR_CLEAR_KEY_Val      0xA5
static struct { struct { uint8_t reg; } RCAUSE; } fake_rstc;
#define RSTC (&fake_rstc)
#define RSTC_RCAUSE_WDT     0x20
#define RSTC_RCAUSE_BODCORE 0x02
#define RSTC_RCAUSE_BODVDD  0x04
#define RSTC_RCAUSE_POR     0x01

/* ------------------------------------------------------------------ */
/* Radio fakes: count every packet by type                             */
/* ------------------------------------------------------------------ */

static uint32_t tx_by_type[8];    /* indexed by packet_type byte */
static uint32_t tx_callsigns = 0;
static uint32_t tx_other = 0;
static uint32_t radio_enables = 0, radio_disables = 0;
static uint8_t  radio_enabled_flag = 0;
static int      tx_result = 0;

void radio_enable(void)  { radio_enables++; radio_enabled_flag = 1; }
void radio_disable(void) { radio_disables++; radio_enabled_flag = 0; }
void radio_init(void)    {}
uint8_t radio_get_channel(void) { return 0; }

int transmit_packet(const uint8_t *data, size_t length)
{
    uint8_t t = data[0];
    if (t >= PACKET_TYPE_GPS && t <= PACKET_TYPE_HEARTBEAT) {
        tx_by_type[t]++;
    } else if (t >= 0x20 && t <= 0x7E) {
        tx_callsigns++;              /* printable = callsign text */
    } else {
        tx_other++;
    }
    return tx_result;
}
int transmit_string(const char *str)
{
    return transmit_packet((const uint8_t *)str, strlen(str) + 1);
}

/* ------------------------------------------------------------------ */
/* IMU event injection (same pattern as test_flight_profile)           */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/* GPS fakes with a scripted fix stream                                */
/* ------------------------------------------------------------------ */

static GPSCoordinates_t fake_coords;
static uint8_t fake_fix_ok = 1;

void    gps_init(void) {}
uint8_t gps_poll_rx(void) { return 0; }
const GPSCoordinates_t* gps_get_current_coordinates(void) { return &fake_coords; }
uint8_t gps_get_health(void) { return HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0); }
float   gps_nmea_to_decimal(const char *n, char d)
{
    double v = atof(n);
    int deg = (int)(v / 100.0);
    double out = deg + (v - deg * 100.0) / 60.0;
    if (d == 'S' || d == 'W') out = -out;
    return (float)out;
}

static void script_fix(double lat, double lon, double alt, int sats,
                       uint8_t fix_quality)
{
    /* NMEA strings the way the u-blox prints them */
    int latd = (int)fabs(lat);
    double latm = (fabs(lat) - latd) * 60.0;
    snprintf(fake_coords.lat, sizeof(fake_coords.lat), "%02d%08.5f", latd, latm);
    fake_coords.lat_dir = lat < 0 ? 'S' : 'N';
    int lond = (int)fabs(lon);
    double lonm = (fabs(lon) - lond) * 60.0;
    snprintf(fake_coords.lon, sizeof(fake_coords.lon), "%03d%08.5f", lond, lonm);
    fake_coords.lon_dir = lon < 0 ? 'W' : 'E';
    snprintf(fake_coords.altitude, sizeof(fake_coords.altitude), "%.1f", alt);
    snprintf(fake_coords.satellites, sizeof(fake_coords.satellites), "%d", sats);
    fake_coords.fix_quality = fix_quality;
    fake_coords.valid = fake_fix_ok && fix_quality >= 1;
}

void uart_init(void) {}

/* ------------------------------------------------------------------ */
/* Real modules + the sketch itself                                    */
/* ------------------------------------------------------------------ */

#include "../firmware/launch_detect.cpp"
#include "../firmware/nav.cpp"
#include "../firmware/ekf.cpp"
#include "../firmware/beacon.cpp"
#include "../firmware/firmware.ino"   /* the sketch under test - AFTER fakes */

/* ------------------------------------------------------------------ */
/* Sim driver                                                          */
/* ------------------------------------------------------------------ */

#define SIM_LAT 39.8900004
#define SIM_LON (-104.8850002)
#define SIM_ALT 1650.0

static void sim_run_burst(uint32_t duration_ms, float burst_z);

/* Advance the sim clock, calling loop() every 10 ms like a fast MCU.
 * Feeds the nav layer 1 Hz from the scripted fix exactly as gps.cpp
 * would, and keeps the IMU streaming so the health flags stay up.
 * burst_z != 0 scripts a motor burn: ONLY the burst accel is queued
 * (interleaving idle accel would break sustained-threshold detection). */
static void sim_run(uint32_t duration_ms)
{
    sim_run_burst(duration_ms, 0.0f);
}

static void sim_run_burst(uint32_t duration_ms, float burst_z)
{
    uint32_t end = sim_ms + duration_ms;
    uint32_t last_gps_feed = 0;
    uint32_t last_burst_ms = 0;
    while (sim_ms < end) {
        sim_ms += 10;
        if (burst_z != 0.0f) {
            if (sim_ms - last_burst_ms >= 50) {
                last_burst_ms = sim_ms;
                push_accel(0.0f, 0.0f, burst_z);
            }
        } else {
            push_accel(0.0f, 0.0f, 0.0f);
        }
        push_rotvec(1.0f, 0.0f, 0.0f, 0.0f);
        if (fake_fix_ok && sim_ms / 1000 != last_gps_feed) {
            last_gps_feed = (uint32_t)(sim_ms / 1000);
            nav_update_from_gps(SIM_LAT, SIM_LON, (float)SIM_ALT, 8, 1);
        }
        loop();
    }
}

static uint32_t type_count(uint8_t type) { return tx_by_type[type]; }

/* ------------------------------------------------------------------ */

int main(void)
{
    Test_SetTickAutoAdvance(0);

    /* ---------- Boot: pad phase, fix healthy ---------- */
    fake_fix_ok = 1;
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 9, 1);
    setup();
    CHECK(tx_callsigns == 1);                     /* boot ID (FCC) */
    CHECK(beacon_state == BEACON_STATE_PRE_LAUNCH);

    sim_run(40000);                               /* 40 s on the pad */
    /* Raw GPS every 5 s: about 8-9 (boot TX at 0 + each 5 s tick) */
    CHECK(type_count(PACKET_TYPE_GPS) >= 7 && type_count(PACKET_TYPE_GPS) <= 10);
    /* No fix problems -> zero heartbeats */
    CHECK(type_count(PACKET_TYPE_HEARTBEAT) == 0);
    /* Fused idle cadence 5 s once nav anchors */
    CHECK(type_count(PACKET_TYPE_FUSED) >= 6);
    /* Pad TX uses radio enable/disable around each non-fast packet */
    CHECK(radio_enables > 0 && radio_disables == radio_enables);
    CHECK(radio_enabled_flag == 0);

    /* ---------- GPS dies at the pad (cable knocked loose) ---------- */
    fake_fix_ok = 0;
    fake_coords.valid = 0;
    uint32_t hb0 = type_count(PACKET_TYPE_HEARTBEAT);
    sim_run(40000);
    /* Loop asks for a beacon every 5 s; with no fix that becomes
     * heartbeats, rate-limited by HEARTBEAT_INTERVAL_SEC - never silence */
    uint32_t hbs = type_count(PACKET_TYPE_HEARTBEAT) - hb0;
    CHECK(hbs >= 6 && hbs <= 10);
    CHECK(type_count(PACKET_TYPE_GPS) <= 10);     /* no new position TX */

    /* ---------- Fix returns, then LAUNCH (sustained accel burst) ----- */
    fake_fix_ok = 1;
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 9, 1);
    sim_run(8000);                                /* re-anchor cleanly */
    CHECK(nav_is_valid() == true);

    uint32_t enables_before = radio_enables;
    sim_run_burst(500, 25.0f);                    /* 500 ms of boost */
    /* The loop must have taken the launch transition to LAUNCH + fast TX */
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(beacon_state == BEACON_STATE_LAUNCH);
    CHECK(transmit_fast_flag == 1);
    CHECK(radio_enables == enables_before + 1);   /* radio keyed once */
    CHECK(radio_enabled_flag == 1);

    /* LAUNCH exits to POST_LAUNCH after POST_LAUNCH_DURATION_SEC */
    sim_run(3000);
    CHECK(beacon_state == BEACON_STATE_POST_LAUNCH);
    CHECK(transmit_fast_flag == 0);
    CHECK(radio_enabled_flag == 0);               /* radio released */

    /* ---------- Descent under chute: fused + paced raw ------------- */
    uint32_t fus0 = type_count(PACKET_TYPE_FUSED);
    uint32_t gps0 = type_count(PACKET_TYPE_GPS);
    sim_run(30000);                               /* 30 s of recovery window */
    uint32_t fus = type_count(PACKET_TYPE_FUSED) - fus0;
    uint32_t gps = type_count(PACKET_TYPE_GPS) - gps0;
    CHECK(fus == 20);                             /* 1500 ms cadence exactly */
    CHECK((gps == 15 || gps == 16));              /* 2 s paced raw stream */

    /* ---------- Touchdown: quiet IMU + stable alt -> battery save ---- */
    sim_run((LAND_QUIET_S + 20) * 1000UL);
    CHECK(launch_detect_has_landed() == true);
    CHECK(beacon_state == BEACON_STATE_BATTERY_SAVE);

    uint32_t fus1 = type_count(PACKET_TYPE_FUSED);
    uint32_t gps1 = type_count(PACKET_TYPE_GPS);
    sim_run(540000);                            /* 9 min on the ground */
    uint32_t fus_save = type_count(PACKET_TYPE_FUSED) - fus1;
    CHECK(fus_save >= 106 && fus_save <= 110);             /* 5 s idle cadence */
    uint32_t gps_save = type_count(PACKET_TYPE_GPS) - gps1;
    CHECK(gps_save >= 8 && gps_save <= 11);                /* ~60 s spacing */

    /* ---------- FCC ID cadence across the whole sim ---------------- */
    CHECK(tx_callsigns == 3);                     /* boot + 300 s + 600 s */
    CHECK(tx_other == 0);                         /* nothing unaccounted */

    /* ---------- Watchdog never saw >16 s of silence ----------------- */
    /* (loop() feeds it every pass; the sim asserting loop-liveness is
     *  the point of the whole file) */
    CHECK(fake_wdt.CLEAR.reg == WDT_CLEAR_CLEAR_KEY_Val);

    printf("mainloop sim: GPS=%lu FUS=%lu HB=%lu CALL=%lu\n",
           (unsigned long)type_count(PACKET_TYPE_GPS),
           (unsigned long)type_count(PACKET_TYPE_FUSED),
           (unsigned long)type_count(PACKET_TYPE_HEARTBEAT),
           (unsigned long)tx_callsigns);

    return TEST_SUMMARY();
}
