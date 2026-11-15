/**
 * @file test_airlink_roundtrip.cpp
 * @brief Airlink property sweep: PRODUCTION TX encoder (beacon.cpp) wired
 *        straight into the PRODUCTION RX parser (rf_parser.c - receiver
 *        source compiled as C and linked in) over thousands of scripted
 *        points covering the full value envelope of both binary formats.
 *
 * The golden vectors pin two exact points; this sweeps the space BETWEEN
 * them: arbitrary lat/lon, fused altitude from below the -500 m floor to
 * the uint16 ceiling, velocity clamp edges, every fused flag combination,
 * raw-GPS clamps and rejections. It catches systematic encode/decode
 * asymmetry (scale, sign, clamp order) that pinned points cannot.
 *
 * Note on scaffolding: this file defines its own micro CHECK macros
 * instead of including receiver/tests/test_harness.h because HAL_GetTick
 * must have C linkage to satisfy the separately-compiled C parser object
 * (the shared harness defines it in a C++ TU, which would not link).
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>
#include "include/gps.h"        /* TX-side: GPSCoordinates_t */
#include "include/radio.h"
#include "include/nav.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"   /* TX copy defines the true wire shape */

/* RX parser under test - its own header has extern "C" guards; the
 * implementation is linked from receiver/firmware/src/rf_parser.c. */
#include "rf_parser.h"          /* receiver/firmware/inc, with GPS_Data */

/* ------------------------------------------------------------------ */
/* Micro harness (see header note)                                     */
/* ------------------------------------------------------------------ */

static int tests_run = 0, checks_failed = 0;
static uint32_t tick_ms = 100000;
extern "C" uint32_t HAL_GetTick(void) { return tick_ms; }

#define CHECK(cond) do {                                                \
    tests_run++;                                                        \
    if (!(cond)) {                                                      \
        checks_failed++;                                                \
        printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                   \
} while (0)
#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))
#define SUMMARY() \
    (printf("%d checks, %d failed\n", tests_run, checks_failed), checks_failed ? 1 : 0)

/* xorshift32 - reproducible sweep */
static uint32_t rng_state = 0x9E3779B9u;
static uint32_t rng(void)
{
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return rng_state = x;
}
static float frand(float lo, float hi)
{
    return lo + (hi - lo) * ((float)(rng() & 0xFFFFFF) / 16777216.0f);
}

/* ------------------------------------------------------------------ */
/* Arduino core fakes                                                  */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
static unsigned long fake_millis_v = 10000;
unsigned long millis(void) { return fake_millis_v; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }

/* Radio fakes: capture what would go over the air */
static uint8_t  tx_buf[256];
static size_t   tx_len = 0;
static uint32_t tx_count = 0;
void radio_enable(void) {}
void radio_disable(void) {}
uint8_t radio_get_channel(void) { return 2; }
int transmit_packet(const uint8_t *data, size_t length)
{
    if (length <= sizeof(tx_buf)) { memcpy(tx_buf, data, length); tx_len = length; }
    tx_count++;
    return 0;
}
int transmit_string(const char *str)
{
    return transmit_packet((const uint8_t *)str, strlen(str) + 1);
}

/* GPS / launch / nav fakes */
uint8_t gps_get_health(void) { return HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0); }
double gps_nmea_to_decimal(const char *nmea_coord, char direction)
{
    double v = atof(nmea_coord);
    int deg = (int)(v / 100.0);
    double out = deg + (v - deg * 100.0) / 60.0;
    if (direction == 'S' || direction == 'W') out = -out;
    return out;
}

static launch_state_t fake_launch_state = LAUNCH_STATE_IDLE;
launch_state_t launch_detect_get_state(void) { return fake_launch_state; }
uint32_t gps_get_fix_age_ms(void) { return 250; }
void launch_detect_get_gyro_rads(float *x, float *y, float *z) { if (x) *x=0; if (y) *y=0; if (z) *z=0; }
void launch_detect_get_accel_xyz(float *x, float *y, float *z) { if (x) *x=0; if (y) *y=0; if (z) *z=0; }
float launch_detect_take_peak_accel(imu_peak_consumer_t who) { (void)who; return 0.0f; }

/* Two-way scaffolding stubs */
bool radio_is_transmitting(void) { return false; }
int  radio_poll_rx(uint8_t *out, size_t max_len, uint16_t timeout_ms)
{ (void)out; (void)max_len; (void)timeout_ms; return 0; }
int  radio_transmit_ack(uint8_t rocket_id, uint8_t cmd_code,
                        uint8_t seq_hi, uint8_t seq_lo, uint8_t echo)
{ (void)rocket_id; (void)cmd_code; (void)seq_hi; (void)seq_lo; (void)echo; return 0; }

static bool fake_landed = false;
bool launch_detect_has_landed(void) { return fake_landed; }

static NavFused_t fake_fused;
void nav_get_fused(NavFused_t *out) { *out = fake_fused; }
static uint32_t fake_gps_rejects = 0;
uint32_t nav_get_gps_rejects(void) { return fake_gps_rejects; }

/* beacon.cpp's V3 heartbeat carries the TX boot's RSTC_RCAUSE; firmware.ino
 * owns the global. Host builds stand it up here. */
uint8_t g_boot_rcause = 0;

/* The real altimeter inputs feed the relay; their flash records go nowhere. */
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
void flight_log_altimeter(uint32_t ms, uint8_t input, uint8_t status,
                          int32_t alt_ft, int32_t apogee_ft)
{ (void)ms; (void)input; (void)status; (void)alt_ft; (void)apogee_ft; }
#include "../firmware/altimeter.cpp"
#include "../firmware/altimeter_input.cpp"

#include "../firmware/beacon.cpp"

/* What the TC3 sampler sees for `text` on PORT A bit `pa_bit` (8N1, idle
 * high around it) while the other jacks in `idle` idle high too. */
static void alt_wave(uint32_t idle, uint8_t pa_bit, const char *text)
{
    for (int k = 0; k < 8; k++) altimeter_sample(idle);
    for (const char *c = text; *c; c++) {
        uint16_t frame = (uint16_t)(((uint8_t)*c << 1) | 0x200);
        for (int b = 0; b < 10; b++)
            for (unsigned k = 0; k < ALT_RX_OVERSAMPLE; k++)
                altimeter_sample((idle & ~(1u << pa_bit)) |
                                 ((uint32_t)((frame >> b) & 1u) << pa_bit));
    }
    for (int k = 0; k < 8; k++) altimeter_sample(idle);
}

/* Altimeters riding the maxima recap: built by the TX from what its inputs
 * decoded, expanded back to input order by the RX parser - none
 * recognised (the plain recap), then two jacks, then all three. */
static void altimeter_roundtrip(void)
{
    altimeter_init();
    MaximaAltPacket_t rx;
    tx_len = 0;
    CHECK(beacon_transmit_maxima(0, 0) == 1);
    CHECK(tx_len == MAXIMA_PACKET_SIZE);
    CHECK(RF_Parser_DecodeMaxima(tx_buf, (uint16_t)tx_len, &rx) == 1);
    CHECK(rx.alt_inputs == 0);

    uint32_t idle = (1u << ALT_J5_PA_BIT) | (1u << ALT_J6_PA_BIT);
    alt_wave(idle, ALT_J5_PA_BIT, "0012.5,+01200,+0410,+072,dma\r"
                                  "0013.0,+01380,+0300,+071,dma\r"
                                  "0018.0,+02510,+0001,+070,Dma\r");
    alt_wave(idle, ALT_J6_PA_BIT, "<01000>%02520>~12---->");
    altimeter_poll(millis());
    tx_len = 0;
    CHECK(beacon_transmit_maxima(0, 0) == 1);
    CHECK(tx_len == MAXIMA_ALT_PACKET_SIZE(2));
    CHECK(RF_Parser_DecodeMaxima(tx_buf, (uint16_t)tx_len, &rx) == 1);
    CHECK(rx.mx.packet_type == PACKET_TYPE_MAXIMA && rx.mx.rocket_id == (uint8_t)ROCKET_ID);
    CHECK(rx.alt_inputs == 0x06 && rx.alt[0].status == 0);          /* J2 not on air */
    CHECK(rx.alt[1].status == (ALT_ST_FMT_RRC3 | ALT_ST_DROGUE));
    CHECK(rx.alt[1].apogee_4ft == 628);
    CHECK(rx.alt[2].status == (ALT_ST_FMT_EGGTIMER | ALT_ST_DROGUE | ALT_ST_MAIN));
    CHECK(rx.alt[2].apogee_4ft == 630);
    CHECK(RF_Parser_DecodeMaxima(tx_buf, (uint16_t)(tx_len - 1), &rx) == 0);

    idle |= 1u << ALT_J2_PA_BIT;
    alt_wave(idle, ALT_J2_PA_BIT, "1180\r\n1190\r\n1200\r\n");
    altimeter_poll(millis());
    tx_len = 0;
    CHECK(beacon_transmit_maxima(0, 0) == 1);
    CHECK(tx_len == MAXIMA_ALT_PACKET_SIZE(3));                    /* 21 B, 1.12 s */
    CHECK(RF_Parser_DecodeMaxima(tx_buf, (uint16_t)tx_len, &rx) == 1);
    CHECK(rx.alt_inputs == 0x07 && rx.alt[0].status == ALT_ST_FMT_STRATOLOGGER);
    CHECK(rx.alt[0].apogee_4ft == 300 && rx.alt[2].apogee_4ft == 630);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Encode a decimal degree into the NMEA ddmm.mmmmm fields the beacon
 * consumes. Produces exactly what a u-blox would print for the same fix. */
static void fmt_nmea_lat(char *out, size_t n, double deg)
{
    int d = (int)fabs(deg);
    double mm = (fabs(deg) - d) * 60.0;
    snprintf(out, n, "%02d%08.5f", d, mm);
}
static void fmt_nmea_lon(char *out, size_t n, double deg)
{
    int d = (int)fabs(deg);
    double mm = (fabs(deg) - d) * 60.0;
    snprintf(out, n, "%03d%08.5f", d, mm);
}

static void check_fused_row(double lat, double lon, float alt, float vn,
                            float ve, float vd, uint8_t age_ds,
                            uint8_t dr, uint8_t fresh, uint8_t healthy,
                            uint8_t degraded)
{
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid = true;
    fake_fused.lat_deg = (float)lat;  fake_fused.lon_deg = (float)lon;
    fake_fused.alt_m = alt;
    fake_fused.v_n = vn; fake_fused.v_e = ve; fake_fused.v_d = vd;
    fake_fused.dead_reckoning = dr;
    fake_fused.gps_fresh = fresh;
    fake_fused.imu_healthy = healthy;
    fake_fused.sensor_degraded = degraded;

    CHECK(beacon_transmit_fused_data(42, 1) == 1);
    CHECK(tx_len == FUSED_PACKET_SIZE);

    /* A stale-fused packet must not override a fresh-ish raw fix; age the
     * tick past RF_FUSED_STALE_OVERRIDE_MS before every decode so this
     * sweep exercises capacity, not the override path (pinned elsewhere). */
    tick_ms += 12000;
    RF_Parser_Init();   /* full parser reset (incl. raw-GPS freshness) */
    CHECK(RF_Parser_ParseFusedPacket(tx_buf, (uint16_t)tx_len) == RF_PARSER_OK);

    GPS_Data g;
    CHECK(RF_Parser_GetParsedData(&g, NULL, 0, NULL) == 1);
    CHECK(g.is_fused == 1);

    /* Position through float scale + int32: 2 counts * 1e-7 deg slack */
    CHECK_NEAR(g.latitude,  (float)lat, 5e-5);
    CHECK_NEAR(g.longitude, (float)lon, 5e-5);

    /* Altitude: floor -500, roof 15883.75, quarter-m grid */
    double exp_alt = alt;
    if (exp_alt < -500.0) exp_alt = -500.0;
    if (exp_alt > 15883.75) exp_alt = 15883.75;
    CHECK_NEAR(g.altitude, exp_alt, 0.3);

    /* Velocities clamp to +-327 m/s, centimeter grid */
    float ev[3] = {vn, ve, vd};
    float gv[3] = {g.v_north, g.v_east, g.v_down};
    for (int i = 0; i < 3; i++) {
        float e = ev[i];
        if (e > 327.0f) e = 327.0f;
        if (e < -327.0f) e = -327.0f;
        CHECK_NEAR(gv[i], e, 0.02);
    }

    /* V3 carries no age byte on the wire: the parser synthesizes it from
     * GPS_FRESH (0 fresh / 255 stale), so the sweep checks the synthesis
     * rule, not round-trip equality. */
    CHECK(g.fused_age_ds == (fresh ? 0 : 255));
    (void)age_ds;
    CHECK(g.fused_dr == dr);
    CHECK(g.fused_gps_fresh == fresh);
    CHECK(g.fused_imu_healthy == healthy);
    CHECK(g.fused_sensor_degraded == degraded);
    CHECK(g.fused_landed == (fake_landed ? 1 : 0));
}

int main(void)
{
    RF_Parser_Init();
    altimeter_roundtrip();

    /* ---------- Fused sweep: envelope edges + random interior ---------- */
    const double lats_edge[] = { -89.9, -45.0, 0.0, 39.89, 45.0, 89.9 };
    const double lons_edge[] = { -180.0, -105.11, 0.0, 179.9, 180.0 };
    const float  alts_edge[] = { -1000.0f, -500.1f, -500.0f, -499.9f, 0.0f,
                                 1655.4f, 15883.7f, 15883.8f, 20000.0f };
    for (size_t i = 0; i < sizeof(lats_edge)/sizeof(lats_edge[0]); i++) {
        for (size_t j = 0; j < sizeof(lons_edge)/sizeof(lons_edge[0]); j += 2) {
            check_fused_row(lats_edge[i], lons_edge[j],
                            alts_edge[(i + j) % 9], 0, 0, 0,
                            (uint8_t)(i + j), 0, 1, 1, 0);
        }
    }
    for (int i = 0; i < 800; i++) {
        fake_landed = (rng() & 0x40) != 0;
        check_fused_row(frand(-89.9, 89.9), frand(-180.0, 180.0),
                        frand(-700.0f, 17000.0f),
                        frand(-400.0f, 400.0f), frand(-400.0f, 400.0f),
                        frand(-400.0f, 400.0f),
                        (uint8_t)(rng() % 300),     /* includes saturate=255 */
                        rng() & 1, rng() & 1, rng() & 1, rng() & 1);
    }
    fake_landed = false;
    /* rocket_id is the fused packet's last byte (index 18): index 19 read
     * whatever a longer, earlier packet had left in the capture buffer */
    CHECK(tx_buf[FUSED_PACKET_SIZE - 1] == (uint8_t)ROCKET_ID);

    /* ---------- Raw GPS sweep ---------- */
    for (int i = 0; i < 600; i++) {
        double lat = frand(-89.9, 89.9);
        double lon = frand(-180.0, 180.0);
        float  alt = frand(-400.0f, 6000.0f);
        int    sats = 4 + (int)(rng() % 28);
        uint8_t fq  = (uint8_t)(1 + rng() % 5);
        fake_launch_state = (rng() & 1) ? LAUNCH_STATE_CONFIRMED : LAUNCH_STATE_IDLE;
        fake_landed = (rng() & 0x20) != 0;

        GPSCoordinates_t c;
        memset(&c, 0, sizeof(c));
        fmt_nmea_lat(c.lat, sizeof(c.lat), lat);
        fmt_nmea_lon(c.lon, sizeof(c.lon), lon);
        c.lat_dir = lat < 0 ? 'S' : 'N';
        c.lon_dir = lon < 0 ? 'W' : 'E';
        snprintf(c.altitude, sizeof(c.altitude), "%.1f", (double)alt);
        snprintf(c.satellites, sizeof(c.satellites), "%d", sats);
        c.fix_quality = fq;
        c.valid = 1;

        CHECK(beacon_transmit_gps_data_binary(&c, 42, 1) == 1);
        CHECK(tx_len == GPS_PACKET_SIZE);
        CHECK(RF_Parser_ParseBinaryPacket(tx_buf, (uint16_t)tx_len) == RF_PARSER_OK);

        GPS_Data g;
        CHECK(RF_Parser_GetParsedData(&g, NULL, 0, NULL) == 1);
        CHECK_NEAR(g.latitude,  lat, 3e-5);   /* NMEA text grid + float scale */
        CHECK_NEAR(g.longitude, lon, 3e-5);
        /* The wire value is the PRINTED string (one decimal), then
         * truncated to whole meters at encode time */
        float alt_on_wire = (float)atof(c.altitude);
        int exp_alt = (alt_on_wire > 32767.0f) ? 32767 : (int)alt_on_wire;
        CHECK(g.altitude == (float)exp_alt);
        CHECK(g.satellites == sats);
        CHECK(g.fix == (fq & FLAG_FIX_TYPE_MASK));
        CHECK(g.launch_detected ==
              (fake_launch_state == LAUNCH_STATE_CONFIRMED ? 1 : 0));
        CHECK(g.fused_landed == (fake_landed ? 1 : 0));
        CHECK(g.is_fused == 0);
    }

    /* ---------- Raw GPS structural rejections must stay rejections ---- */
    /* State hygiene: the sweeps randomize launch/landing state per
     * iteration; restore pad semantics or the 3-sat case legitimately
     * passes the post-landing 2D gate (caught by CI fresh builds). */
    fake_launch_state = LAUNCH_STATE_IDLE;
    fake_landed = false;
    struct { const char *alt; const char *sats; uint8_t fq; } bad[] = {
        { "-600", "8", 1 },   /* below altitude sanity floor */
        { "60000", "8", 1 },  /* above sanity ceiling        */
        { "1655",  "3", 1 },  /* below 4 sats (pre-landing)  */
        { "1655",  "8", 0 },  /* no fix                      */
    };
    for (size_t i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
        GPSCoordinates_t c;
        memset(&c, 0, sizeof(c));
        strcpy(c.lat, "3953.40000"); strcpy(c.lon, "10453.11007");
        c.lat_dir = 'N'; c.lon_dir = 'W';
        strcpy(c.altitude, bad[i].alt);
        strcpy(c.satellites, bad[i].sats);
        c.fix_quality = bad[i].fq;
        c.valid = 1;
        CHECK(beacon_transmit_gps_data_binary(&c, 42, 1) == 0);
    }

    /* And nothing from the sweeps ever confused the parser: land one more
     * clean pair and read it back to prove global state stayed coherent. */
    memset(&fake_fused, 0, sizeof(fake_fused));
    fake_fused.valid = true; fake_fused.lat_deg = 39.89f;
    fake_fused.lon_deg = -105.11f; fake_fused.alt_m = 1655.0f;
    CHECK(beacon_transmit_fused_data(42, 1) == 1);
    tick_ms += 12000;
    RF_Parser_Init();   /* full parser reset (incl. raw-GPS freshness) */
    CHECK(RF_Parser_ParseFusedPacket(tx_buf, (uint16_t)tx_len) == RF_PARSER_OK);
    GPS_Data g;
    CHECK(RF_Parser_GetParsedData(&g, NULL, 0, NULL) == 1);
    CHECK_NEAR(g.latitude, 39.89, 5e-5);

    return SUMMARY();
}
