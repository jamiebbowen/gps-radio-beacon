/**
 * @file test_navigation_mode.c
 * @brief Host tests for the navigation display page (recovery-critical UI).
 *
 * navigation_mode.c is compiled unmodified with the display and RF layers
 * capture-faked: every Display_DrawTextRowCol lands in a buffer the tests
 * inspect. Covers the row semantics the recovery walk depends on:
 *   - pad: ready-to-fly verdict gating (L/T/R/C failure flags)
 *   - landed: LANDED beats "READY TO FLY", drag MOVED! warning,
 *     walk closing-rate (CLS/AWY), anchor re-arm after latch clears
 *   - heartbeat-only pad screen (sat acquisition verdicts), scanning, dead RF
 *
 * Build & run:  make -C receiver/tests
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "display_modes/navigation_mode.h"
#include "compass.h"       /* Compass_Data */
#include "rf_receiver.h"   /* HeartbeatPacket_t, HB_GPS_* */
#include "packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Display capture fake                                                */
/* ------------------------------------------------------------------ */

#define CAP_ROWS 8
static char  cap_row_text[CAP_ROWS][40];
static float cap_arrow_angle = -1.0f;

void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
    if (row >= CAP_ROWS) return;
    if (col != 0) {
        /* mimic row overlay: append at column offset when the row is fresh */
        size_t l = strlen(cap_row_text[row]);
        if (l == 0) while (l < (size_t)col) cap_row_text[row][l++] = ' ';
        strncpy(cap_row_text[row] + l, text, sizeof(cap_row_text[row]) - 1 - l);
        cap_row_text[row][sizeof(cap_row_text[row]) - 1] = '\0';
        return;
    }
    strncpy(cap_row_text[row], text, sizeof(cap_row_text[row]) - 1);
    cap_row_text[row][sizeof(cap_row_text[row]) - 1] = '\0';
}
void Display_DrawText(uint8_t x, uint8_t y, const char *text)
{
    (void)x; (void)y; (void)text;
}
void Display_DrawDirectionIndicator(uint8_t x, uint8_t y, float angle)
{
    (void)x; (void)y; cap_arrow_angle = angle;
}
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    (void)x0; (void)y0; (void)r; (void)color;
}

static void cap_clear(void)
{
    memset(cap_row_text, 0, sizeof(cap_row_text));
    cap_arrow_angle = -1.0f;
}

/* ------------------------------------------------------------------ */
/* RF layer knobs (the page reads these via the public API)            */
/* ------------------------------------------------------------------ */

static int16_t  fake_rssi = -90;
static int8_t   fake_snr  = 6;
static uint8_t  fake_noise_alert = 0;
static int16_t  fake_noise_floor = -118;
static uint8_t  fake_floor_valid = 0;
static uint8_t  fake_scanning    = 0;
static uint8_t  fake_channel     = 0;
static uint8_t  fake_hb_heard    = 0;
static HeartbeatPacket_t fake_hb;
static uint32_t fake_hb_age_ms   = 0;

uint8_t RF_Receiver_GetLastHeartbeat(HeartbeatPacket_t *hb, uint32_t *age_ms)
{
    if (!fake_hb_heard) return 0;
    if (hb) memcpy(hb, &fake_hb, sizeof(*hb));
    if (age_ms) *age_ms = fake_hb_age_ms;
    return 1;
}
uint8_t RF_Receiver_NoiseAlert(void) { return fake_noise_alert; }
uint8_t RF_Receiver_GetNoiseFloor(int16_t *nf)
{
    if (!fake_floor_valid) return 0;
    *nf = fake_noise_floor;
    return 1;
}
void RF_Receiver_GetSignalQuality(int16_t *rssi, int8_t *snr)
{
    *rssi = fake_rssi; *snr = fake_snr;
}
uint8_t RF_Receiver_IsScanning(void) { return fake_scanning; }
uint8_t RF_Receiver_GetChannel(void) { return fake_channel; }

/* globals.c contract */
uint8_t rf_initialized = 1;
Compass_Data compass_data;

/* ------------------------------------------------------------------ */
/* Scenario driver                                                     */
/* ------------------------------------------------------------------ */

static GPS_Data local_g, remote_g;
static uint8_t draw_has_remote = 1;   /* has_valid_remote_gps for draw() */

static void reset_scenario(void)
{
    cap_clear();
    memset(&local_g, 0, sizeof(local_g));
    memset(&remote_g, 0, sizeof(remote_g));
    local_g.latitude    = 39.8900f;
    local_g.longitude   = -105.1150f;
    local_g.satellites  = 9;
    local_g.fix         = 1;
    memset(&compass_data, 0, sizeof(compass_data));
    compass_data.heading_valid  = 1;
    compass_data.heading_stale  = 0;
    compass_data.orientation_valid = 1;
    fake_noise_alert = 0;  fake_floor_valid = 0; fake_scanning = 0;
    fake_hb_heard    = 0;  fake_hb_age_ms  = 0;  fake_channel   = 0;
    fake_rssi = -90;       fake_snr = 8;
    rf_initialized = 1;
    draw_has_remote = 1;
    memset(&fake_hb, 0, sizeof(fake_hb));
    remote_g.latitude  = 39.8905f;      /* pad rocket a few dozen metres off */
    remote_g.longitude = -105.1155f;
    remote_g.fix = 3;  remote_g.satellites = 11;
}

/* Draw with the fake tick set */
static void draw(uint32_t now_ms, uint32_t last_pkt_ms, uint32_t pkt_count)
{
    Test_SetTick(now_ms);
    cap_clear();
    DisplayMode_Navigation(1, draw_has_remote, &local_g, &remote_g, 45.0f,
                           last_pkt_ms, pkt_count);
    if (getenv("NAVDBG")) {
        for (int r = 0; r < CAP_ROWS; r++)
            fprintf(stderr, "[%d] '%s'\n", r, cap_row_text[r]);
        fprintf(stderr, "  arrow=%.1f\n---\n", (double)cap_arrow_angle);
    }
}

#define ROW_IS(row, text)  CHECK(strcmp(cap_row_text[row], text) == 0)
#define ROW_HAS(row, sub)  CHECK(strstr(cap_row_text[row], sub) != NULL)

/* ------------------------------------------------------------------ */

TEST(test_nav_pad_ready_to_fly_verdict)
{
    reset_scenario();
    draw(100000, 100000, 42);          /* fresh link */

    ROW_IS(0, "Dist: 70m");
    ROW_IS(2, "L:Fix B:3D");
    ROW_HAS(3, "S:11");
    ROW_IS(4, "Excellent");
    ROW_IS(5, "READY TO FLY");
    ROW_HAS(6, "RSSI -90");
    ROW_HAS(7, "Pkts:42 GPS");
    CHECK(cap_arrow_angle >= 0.0f);    /* direction arrow up */
}

TEST(test_nav_not_ready_names_failed_checks)
{
    reset_scenario();
    remote_g.fix        = 0;           /* TX no fix */
    remote_g.satellites = 1;
    compass_data.heading_valid = 0;    /* compass uncalibrated */
    draw(100000, 100000 - 60000, 5);   /* stale link too */

    ROW_HAS(5, "NOT RDY:");
    ROW_HAS(5, "L");                   /* stale link */
    ROW_HAS(5, "T");                   /* TX fix     */
    ROW_HAS(5, "C");                   /* compass    */
}

TEST(test_nav_landed_shows_landed_not_rtf)
{
    reset_scenario();
    remote_g.fused_landed = 1;
    draw(100000, 100000, 50);          /* first landed packet = anchor */
    ROW_HAS(7, "LANDED");
    ROW_IS(5, "LANDED");               /* never 'READY TO FLY' on the ground */
}

TEST(test_nav_landed_drag_warning)
{
    reset_scenario();
    remote_g.fused_landed = 1;
    draw(100000, 100000, 10);          /* touchdown anchor (remote pos) */
    ROW_IS(5, "LANDED");

    /* Wind drags it ~120 m south between packets */
    remote_g.latitude -= 120.0f / 111320.0f;
    draw(100500, 100500, 11);
    ROW_HAS(5, "MOVED +");
    ROW_HAS(5, "m!");
}

TEST(test_nav_walk_closing_rate)
{
    reset_scenario();
    /* Clear any landed anchor from earlier tests, then land at the site.
     * The trend tracker pairs the last two >=20 s distance samples, so
     * let one update land AT the touchdown site before the approach leg. */
    draw(200000, 200000, 1);           /* latch clear -> anchor re-arm */
    remote_g.fused_landed = 1;
    remote_g.latitude = 39.8990f;      /* ~1.0 km north of us */
    draw(200500, 200500, 2);           /* touch down */
    draw(221500, 221500, 3);           /* 21 s on site: ref pinned here */
    ROW_HAS(7, "LANDED");

    /* 25 s later, ~280 m closer */
    local_g.latitude = 39.8965f;
    draw(246500, 221500, 4);           /* NOTE: position packet is stale */
    ROW_HAS(5, "CLS");
    ROW_HAS(5, "m/m");
}

TEST(test_nav_landed_anchor_rearms_after_latch_clear)
{
    reset_scenario();
    remote_g.fused_landed = 1;
    draw(250500, 250500, 1);
    /* Anchor from the previous landing scenario is still latched: the
     * position shift must read as MOVED (correct carryover semantics). */
    ROW_HAS(5, "MOVED");

    /* Latch clears (power cycle): anchor re-arms on the next landed row */
    remote_g.fused_landed = 0;
    draw(251000, 251000, 2);
    ROW_IS(5, "READY TO FLY");         /* pad verdict returns */
    remote_g.fused_landed = 1;
    draw(251500, 251500, 3);
    ROW_IS(5, "LANDED");               /* fresh anchor here, no MOVED */
}

TEST(test_nav_heartbeat_only_screen)
{
    reset_scenario();
    draw_has_remote = 0;               /* no position packet ever */
    fake_hb_heard = 1;
    fake_hb.packet_type = PACKET_TYPE_HEARTBEAT;
    fake_hb.rocket_id = 3;
    fake_hb.channel   = 2;
    fake_hb.satellites = 0;
    fake_hb.uptime_s  = 95;
    fake_hb.gps_health = HB_GPS_HEALTH(HB_GPS_ACQUIRING, 2);
    fake_hb_age_ms = 4200;
    draw(260000, 0, 0);
    ROW_IS(1, "NOT READY YET");
    ROW_HAS(2, "Beacon HB 4s ago");
    ROW_HAS(3, "R3 CH2 sats:0");
    /* 0 sats at 90+ s uptime = TX-side hardware verdict, not patience;
     * and the reset counter must SURVIVE the row budget (22 cols). */
    ROW_HAS(6, "0 sats-chk TXant");
    ROW_HAS(6, "rst:2");
}

TEST(test_nav_scanning_and_dead_radio)
{
    reset_scenario();
    fake_scanning = 1;
    draw(270000, 0, 0);
    ROW_HAS(4, "Scanning");

    reset_scenario();
    rf_initialized = 0;
    draw(270500, 0, 0);
    ROW_HAS(4, "RF DEAD");
}

TEST(test_nav_noise_alert_overrides_freshness_row)
{
    reset_scenario();
    remote_g.satellites = 9; remote_g.fix = 1;
    fake_noise_alert = 1; fake_floor_valid = 1; fake_noise_floor = -85;
    draw(280000, 280000, 7);
    ROW_HAS(1, "RF NOISE!");
    ROW_HAS(1, "-85");
}

int main(void)
{
    run_test_nav_pad_ready_to_fly_verdict();
    run_test_nav_not_ready_names_failed_checks();
    run_test_nav_landed_shows_landed_not_rtf();
    run_test_nav_landed_drag_warning();
    run_test_nav_walk_closing_rate();
    run_test_nav_landed_anchor_rearms_after_latch_clear();
    run_test_nav_heartbeat_only_screen();
    run_test_nav_scanning_and_dead_radio();
    run_test_nav_noise_alert_overrides_freshness_row();

    return TEST_SUMMARY();
}
