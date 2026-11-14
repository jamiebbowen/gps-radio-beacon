/**
 * test_beacon_track.c - the receiver's beacon state (beacon_track.c)
 *
 * These rules lived inline in main.c, which has no host build: the round-4
 * changes (held dead-reckoned packets, the heartbeat-paced pad link) were
 * covered only by the firmware compile. Pinned here.
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/beacon_track.c"
#include "../firmware/src/preflight_eval.c"
#include "test_harness.h"

static GPS_Data pkt(float lat, float lon, uint8_t held)
{
    GPS_Data g;
    memset(&g, 0, sizeof(g));
    g.latitude = lat;
    g.longitude = lon;
    g.altitude = 1650.0f;
    g.fix = 1;
    g.satellites = 9;
    g.fused_position_held = held;
    return g;
}

TEST(test_init_is_empty)
{
    BeaconTrack_t bt;
    memset(&bt, 0x5A, sizeof(bt));
    BeaconTrack_Init(&bt);
    CHECK(bt.has_valid == 0 && bt.has_last_good == 0);
    CHECK(bt.pos_tick == 0 && bt.packet_count == 0);
    CHECK(strcmp(bt.current.fix_str, "0") == 0);
    CHECK(strcmp(bt.current.satellites_str, "0") == 0);
}

TEST(test_usable_packet_becomes_current_and_last_good)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data g = pkt(39.89f, -105.11f, 0);
    CHECK(BeaconTrack_OnPacket(&bt, &g, 20000) ==
          (BEACON_TRACK_NAV_ROW | BEACON_TRACK_SAVE));
    CHECK(bt.has_valid == 1 && bt.has_last_good == 1);
    CHECK(bt.pos_tick == 20000 && bt.packet_count == 1);
    CHECK_NEAR(bt.current.latitude, 39.89, 1e-5);
    CHECK_NEAR(bt.last_good.longitude, -105.11, 1e-5);
}

TEST(test_unusable_coordinates_invalidate_but_count)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data good = pkt(39.89f, -105.11f, 0);
    (void)BeaconTrack_OnPacket(&bt, &good, 20000);

    GPS_Data zero = pkt(0.0f, 0.0f, 0);
    CHECK(BeaconTrack_OnPacket(&bt, &zero, 21000) == 0);   /* no NAV row */
    CHECK(bt.has_valid == 0 && bt.has_last_good == 1);
    CHECK_NEAR(bt.current.latitude, 39.89, 1e-5);          /* kept */
    CHECK(bt.pos_tick == 21000 && bt.packet_count == 2);

    GPS_Data lat_out = pkt(91.0f, 10.0f, 0);
    GPS_Data lon_out = pkt(10.0f, -181.0f, 0);
    CHECK(BeaconTrack_OnPacket(&bt, &lat_out, 22000) == 0);
    CHECK(BeaconTrack_OnPacket(&bt, &lon_out, 23000) == 0);
    CHECK(bt.has_valid == 0);
}

TEST(test_held_packet_updates_flags_not_age_or_save)
{
    /* RX-16: a dead-reckoned fused packet is held by the parser. It still
     * reaches the pages (DR / landed flags) and the NAV log, but neither
     * refreshes the position age nor is ever written to BEACON.TXT. */
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data real = pkt(39.89f, -105.11f, 0);
    (void)BeaconTrack_OnPacket(&bt, &real, 20000);
    GPS_Data held = pkt(39.89f, -105.11f, 1);
    held.is_fused = 1;
    held.fused_dr = 1;
    CHECK(BeaconTrack_OnPacket(&bt, &held, 45000) == BEACON_TRACK_NAV_ROW);
    CHECK(bt.pos_tick == 20000);
    CHECK(bt.current.fused_dr == 1 && bt.current.is_fused == 1);
    CHECK(bt.packet_count == 2);
}

TEST(test_beacon_file_rate_limit)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data g = pkt(39.89f, -105.11f, 0);
    CHECK(BeaconTrack_OnPacket(&bt, &g, 20000) & BEACON_TRACK_SAVE);
    CHECK(!(BeaconTrack_OnPacket(&bt, &g, 25000) & BEACON_TRACK_SAVE));
    CHECK(!(BeaconTrack_OnPacket(&bt, &g, 29999) & BEACON_TRACK_SAVE));
    CHECK(BeaconTrack_OnPacket(&bt, &g, 30000) & BEACON_TRACK_SAVE);
}

TEST(test_staleness_after_three_minutes)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    CHECK(BeaconTrack_Age(&bt, 999999) == 0);              /* nothing yet */
    GPS_Data g = pkt(39.89f, -105.11f, 0);
    (void)BeaconTrack_OnPacket(&bt, &g, 20000);
    CHECK(BeaconTrack_Age(&bt, 20000 + BEACON_TRACK_STALE_MS) == 0);
    CHECK(bt.has_valid == 1);
    CHECK(BeaconTrack_Age(&bt, 20001 + BEACON_TRACK_STALE_MS) == 1);
    CHECK(bt.has_valid == 0 && bt.has_last_good == 1);     /* display fallback */

    /* A held packet does not revive a stale position (main.c runs the
     * packet before the age check in the same loop) ... */
    GPS_Data held = pkt(39.89f, -105.11f, 1);
    (void)BeaconTrack_OnPacket(&bt, &held, 30000 + BEACON_TRACK_STALE_MS);
    CHECK(BeaconTrack_Age(&bt, 30000 + BEACON_TRACK_STALE_MS) == 1);
    CHECK(bt.has_valid == 0);
    /* ... a real one does */
    (void)BeaconTrack_OnPacket(&bt, &g, 31000 + BEACON_TRACK_STALE_MS);
    CHECK(BeaconTrack_Age(&bt, 31000 + BEACON_TRACK_STALE_MS) == 0);
    CHECK(bt.has_valid == 1);
}

TEST(test_new_context_restore_and_forget)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    GPS_Data g = pkt(39.89f, -105.11f, 0);
    (void)BeaconTrack_OnPacket(&bt, &g, 20000);
    BeaconTrack_NewContext(&bt);                           /* manual pick */
    CHECK(bt.has_valid == 0 && bt.has_last_good == 0 && bt.pos_tick == 0);
    CHECK(BeaconTrack_Age(&bt, 9999999) == 0);

    /* Boot: BEACON.TXT restores a position with no age ("Last: SD") */
    BeaconTrack_Init(&bt);
    BeaconTrack_Restore(&bt, 39.5f, -105.2f, 1655.0f);
    CHECK(bt.has_valid == 1 && bt.has_last_good == 1 && bt.pos_tick == 0);
    CHECK(bt.current.fix == 1 && bt.last_good.fix == 1);
    CHECK_NEAR(bt.current.latitude, 39.5, 1e-5);
    CHECK_NEAR(bt.last_good.altitude, 1655.0, 0.01);
    CHECK(BeaconTrack_Age(&bt, 9999999) == 0);             /* never ages out */

    BeaconTrack_Forget(&bt);                               /* implausibly far */
    CHECK(bt.has_valid == 0 && bt.has_last_good == 0);
    CHECK(bt.current.fix == 0 && bt.last_good.fix == 0);
}

TEST(test_preflight_link_inputs)
{
    /* RX-14: the pad link is the beacon-heard tick (heartbeats count); the
     * position report is the newest APPLIED position */
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    Preflight_Inputs_t in;
    memset(&in, 0, sizeof(in));
    in.rx_vdd_mv = 4100;                                   /* untouched */
    BeaconTrack_FillPreflight(&bt, 50000, 0, 0, 0, 0, 0, &in);
    CHECK(in.link_heard == 0 && in.link_age_s == 0);
    CHECK(in.pos_valid == 0 && in.pos_age_s == 0 && in.hb_valid == 0);
    CHECK(in.rx_vdd_mv == 4100);

    GPS_Data g = pkt(39.89f, -105.11f, 0);
    (void)BeaconTrack_OnPacket(&bt, &g, 20000);
    BeaconTrack_FillPreflight(&bt, 620000, 600000, 1, 15000, 1, 7, &in);
    CHECK(in.link_heard == 1 && in.link_age_s == 20);
    CHECK(in.pos_valid == 1 && in.pos_age_s == 600);
    CHECK(in.pos_fix == 1 && in.pos_sats == 9);
    CHECK(in.hb_valid == 1 && in.hb_age_s == 15);
    CHECK(in.hb_fix == 1 && in.hb_sats == 7);

    /* ...and the evaluation on top: the heartbeat-paced pad is READY */
    in.rx_fix_ok = 1;
    in.compass_ok = 1;
    in.testing_suspect = 0;
    Preflight_Result_t r;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY && r.tx_sats == 7);
}

int main(void)
{
    run_test_init_is_empty();
    run_test_usable_packet_becomes_current_and_last_good();
    run_test_unusable_coordinates_invalidate_but_count();
    run_test_held_packet_updates_flags_not_age_or_save();
    run_test_beacon_file_rate_limit();
    run_test_staleness_after_three_minutes();
    run_test_new_context_restore_and_forget();
    run_test_preflight_link_inputs();
    return TEST_SUMMARY();
}
