/**
 * test_eink_summary.c - what the e-ink summary page says (eink_summary.c)
 *
 * Composed inline in main.c until round 5 (no host build); round 4 moved
 * its link age onto the beacon-heard tick. Pinned here.
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/eink_summary.c"
#include "../firmware/src/beacon_track.c"
#include "test_harness.h"

static EinkSummary_Line_t lines[EINK_SUMMARY_MAX_LINES];

static const EinkSummary_Line_t *find(uint8_t n, const char *prefix)
{
    for (uint8_t i = 0; i < n; i++)
        if (strncmp(lines[i].text, prefix, strlen(prefix)) == 0) return &lines[i];
    return NULL;
}

TEST(test_never_heard_no_beacon)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    EinkSummary_In_t in = { .channel = 3, .rssi = -120, .now_ms = 600000,
                            .heard_tick = 0, .distance_km = -1.0f,
                            .vdd_mv = 3900, .beacon = &bt };
    uint8_t n = EinkSummary_Compose(&in, lines);
    char ch[48];
    snprintf(ch, sizeof(ch), "CH3 %.2fMHz  rssi -120dBm", LORA_CHANNEL_FREQ_MHZ(3));
    CHECK(n == 5);
    CHECK(strcmp(lines[0].text, "GPS BEACON RX") == 0 && lines[0].size == 2);
    CHECK(strcmp(lines[1].text, ch) == 0);
    CHECK(strcmp(lines[2].text, "age 0s  pkts 0") == 0);
    CHECK(find(n, "NO BEACON YET") != NULL);
    const EinkSummary_Line_t *lost = find(n, "LINK LOST");
    CHECK(lost != NULL && lost->color == EINK_RED && lost->size == 2);
    CHECK(find(n, "BAT") == NULL);
}

TEST(test_heard_beacon_with_position)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data g;
    memset(&g, 0, sizeof(g));
    g.latitude = 39.89f; g.longitude = -105.11f; g.altitude = 1650.0f;
    (void)BeaconTrack_OnPacket(&bt, &g, 20000);
    EinkSummary_In_t in = { .channel = 0, .rssi = -85, .now_ms = 630000,
                            .heard_tick = 600000, .distance_km = 1.234f,
                            .vdd_mv = 3900, .beacon = &bt };
    uint8_t n = EinkSummary_Compose(&in, lines);
    CHECK(n == 7);
    CHECK(strcmp(lines[2].text, "age 30s  pkts 1") == 0);
    CHECK(find(n, "lat 39.89000") != NULL);
    CHECK(find(n, "lon -105.11000") != NULL);
    CHECK(find(n, "alt 1650m  dist 1.23km") != NULL);
    const EinkSummary_Line_t *bat = find(n, "BAT 3.90V  up 630s");
    CHECK(bat != NULL && bat->color == EINK_BLACK);
    CHECK(find(n, "LINK LOST") == NULL);

    /* Distance unknown: no alt/dist line */
    in.distance_km = -1.0f;
    n = EinkSummary_Compose(&in, lines);
    CHECK(n == 6 && find(n, "alt ") == NULL);

    /* Stale but remembered (has_last_good only): still shown */
    bt.has_valid = 0;
    n = EinkSummary_Compose(&in, lines);
    CHECK(find(n, "lat 39.89000") != NULL && find(n, "NO BEACON YET") == NULL);
}

TEST(test_link_lost_boundary)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    EinkSummary_In_t in = { .channel = 1, .rssi = -90, .now_ms = 1000000,
                            .heard_tick = 1000000 - EINK_SUMMARY_LOST_S * 1000u,
                            .distance_km = -1.0f, .vdd_mv = 3700, .beacon = &bt };
    uint8_t n = EinkSummary_Compose(&in, lines);
    CHECK(find(n, "LINK LOST") == NULL && find(n, "BAT 3.70V") != NULL);  /* 180 s */
    in.heard_tick -= 1000;
    n = EinkSummary_Compose(&in, lines);
    CHECK(find(n, "LINK LOST") != NULL && find(n, "BAT") == NULL);        /* 181 s */
}

int main(void)
{
    run_test_never_heard_no_beacon();
    run_test_heard_beacon_with_position();
    run_test_link_lost_boundary();
    return TEST_SUMMARY();
}
