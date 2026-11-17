/**
 * Host tests for log_policy.c - main.c's SD breadcrumb decisions (link
 * edges, contact loss, the RFSTATS row). main.c has no host build, and
 * the host tools parse these rows, so their wording and timing are pinned
 * here.
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/log_policy.c"
#include "test_harness.h"

TEST(test_link_edge_hysteresis)
{
    LogPolicy_LinkEdge_t st = {0};
    char m[48];
    CHECK(LogPolicy_LinkEdge(&st, -70, 9, m, sizeof(m)) == 0);   /* strong */
    CHECK(LogPolicy_LinkEdge(&st, -84, 5, m, sizeof(m)) == 0);   /* above edge */
    CHECK(LogPolicy_LinkEdge(&st, -85, 3, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "LINK WEAK rssi=-85dBm snr=3") == 0);
    /* flutter inside the -85..-80 band logs nothing */
    CHECK(LogPolicy_LinkEdge(&st, -82, 4, m, sizeof(m)) == 0);
    CHECK(LogPolicy_LinkEdge(&st, -90, 1, m, sizeof(m)) == 0);
    CHECK(LogPolicy_LinkEdge(&st, -81, 4, m, sizeof(m)) == 0);
    CHECK(LogPolicy_LinkEdge(&st, -80, -2, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "LINK OK rssi=-80dBm snr=-2") == 0);
    CHECK(LogPolicy_LinkEdge(&st, -79, 6, m, sizeof(m)) == 0);
}

TEST(test_contact_lost_once_and_regained_with_full_gap)
{
    LogPolicy_Contact_t st = {0};
    char m[48];
    /* nothing heard yet: nothing to lose */
    CHECK(LogPolicy_Contact(&st, 0, 900000, m, sizeof(m)) == 0);
    /* packets flowing */
    CHECK(LogPolicy_Contact(&st, 100000, 101000, m, sizeof(m)) == 0);
    CHECK(LogPolicy_Contact(&st, 100000, 400000, m, sizeof(m)) == 0); /* 300 s: not yet */
    /* past 5 min of silence: one LOST row, then quiet */
    CHECK(LogPolicy_Contact(&st, 100000, 400001, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "RF LOST (>5min silence)") == 0);
    CHECK(LogPolicy_Contact(&st, 100000, 700000, m, sizeof(m)) == 0);
    /* a packet at 1000 s ends a 900 s silence; the row reports all of it
     * (it used to count from the LOST row - 300 s short) */
    CHECK(LogPolicy_Contact(&st, 1000000, 1000050, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "RF REGAINED after 900s") == 0);
    CHECK(LogPolicy_Contact(&st, 1000000, 1001000, m, sizeof(m)) == 0);
    /* re-armed: the next long silence logs again */
    CHECK(LogPolicy_Contact(&st, 1000000, 1300001, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "RF LOST (>5min silence)") == 0);
    /* tick wraparound inside a silence still measures the gap */
    LogPolicy_Contact_t w = {0};
    CHECK(LogPolicy_Contact(&w, 0xFFFF0000u, 0xFFFF0000u + 300001u, m, sizeof(m)) == 1);
    CHECK(LogPolicy_Contact(&w, 0x00100000u, 0x00100010u, m, sizeof(m)) == 1);
    CHECK(strcmp(m, "RF REGAINED after 1114s") == 0);
}

TEST(test_contact_heartbeat_paced_pad_never_lost)
{
    /* main.c feeds the beacon-heard tick: on the pad that is one heartbeat
     * a minute (a position only every 10 minutes), which is contact, not a
     * series of 5-minute blackouts. */
    LogPolicy_Contact_t st = {0};
    char m[48];
    uint32_t heard = 10000;
    for (uint32_t now = 10000; now < 3600000u; now += 1000) {
        if (now - heard >= 60000u) heard = now;          /* next heartbeat */
        CHECK(LogPolicy_Contact(&st, heard, now, m, sizeof(m)) == 0);
    }
    CHECK(st.lost == 0);
}

TEST(test_rfstats_row_format)
{
    LogPolicy_RfStats_t s;
    memset(&s, 0, sizeof(s));
    s.pkts = 120; s.irqs = 125; s.crc = 3; s.wedges = 1;
    s.nf_valid = 1; s.nf_dbm = -118;
    s.vdd_mv = 3712; s.loop_ms = 41; s.sd_ms = 12; s.syncs = 7; s.spidiv = 16;
    s.rf_ms = 9; s.gps_ms = 2; s.disp_ms = 30; s.cmp_ms = 1;
    s.iters = 5400; s.sderr = 0;
    s.nfl_valid = 1; s.nfl_dbm = -112;
    char m[200];
    LogPolicy_FormatRfStats(&s, m, sizeof(m));
    CHECK(strcmp(m, "RFSTATS pkts=120 irq=125 crc=3 wedges=1 nf=-118dBm vdd=3712mV "
                    "loop=41ms sd=12ms syncs=7 spidiv=16 rf=9ms gps=2ms disp=30ms "
                    "cmp=1ms iters=5400 sderr=0 nfl=-112dBm") == 0);
    /* no alert-floor estimate yet; no live-link floor: n/a, suffix absent */
    s.nf_valid = 0; s.nfl_valid = 0;
    LogPolicy_FormatRfStats(&s, m, sizeof(m));
    CHECK(strstr(m, " nf=n/a vdd=3712mV ") != NULL);
    CHECK(strstr(m, "nfl=") == NULL);
    CHECK(m[strlen(m) - 1] == '0');           /* ends at sderr=0 */
}

TEST(test_heartbeat_row_format)
{
    LogPolicy_Heartbeat_t h;
    memset(&h, 0, sizeof(h));
    h.rocket_id = 3; h.channel = 0; h.satellites = 9; h.fix_quality = 1;
    h.uptime_s = 120; h.rssi_dbm = -80; h.gps_state = 3; h.gps_resets = 0;
    h.reset_name = "POR";
    char m[128];
    LogPolicy_FormatHeartbeat(&h, m, sizeof(m));
    /* LOG_CONTRACT heartbeat: tools/test_log_summary.py parses this exact row */
    CHECK(strcmp(m, "HEARTBEAT id=3 CH0 sats=9 fix=1 up=120s rssi=-80 gps=acq rst=0 rc=POR") == 0);

    /* another rocket's heartbeat is marked, so the tools can skip it */
    h.foreign = 1; h.gps_state = 1; h.gps_resets = 2; h.reset_name = "WDT";
    LogPolicy_FormatHeartbeat(&h, m, sizeof(m));
    CHECK(strcmp(m, "HEARTBEAT FOREIGN id=3 CH0 sats=9 fix=1 up=120s rssi=-80 gps=SILENT rst=2 rc=WDT") == 0);

    /* every known GPS state has a name; anything newer prints '?' */
    static const char *names[] = { "gps=unk ", "gps=SILENT ", "gps=GARBLED ", "gps=acq " };
    h.foreign = 0;
    for (uint8_t st = 0; st < 4; st++) {
        h.gps_state = st;
        LogPolicy_FormatHeartbeat(&h, m, sizeof(m));
        CHECK(strstr(m, names[st]) != NULL);
    }
    h.gps_state = 9;
    LogPolicy_FormatHeartbeat(&h, m, sizeof(m));
    CHECK(strstr(m, "gps=? ") != NULL);
    h.gps_state = 4;                       /* the first past the table */
    LogPolicy_FormatHeartbeat(&h, m, sizeof(m));
    CHECK(strstr(m, "gps=? ") != NULL);
}

int main(void)
{
    run_test_link_edge_hysteresis();
    run_test_contact_lost_once_and_regained_with_full_gap();
    run_test_contact_heartbeat_paced_pad_never_lost();
    run_test_rfstats_row_format();
    run_test_heartbeat_row_format();
    return TEST_SUMMARY();
}
