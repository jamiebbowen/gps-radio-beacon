/**
 * test_preflight_eval.c - the shared go/no-go evaluation
 *
 * Both READY TO FLY displays render from this one result, so its truth
 * table (checks + veto priority) is the contract pinned here.
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/preflight_eval.c"
#include "test_harness.h"

static Preflight_Inputs_t all_green(void)
{
    Preflight_Inputs_t in;
    memset(&in, 0, sizeof(in));
    in.link_heard = 1;
    in.link_age_s = 5;
    in.pos_valid = 1;
    in.pos_age_s = 5;
    in.pos_fix = 1;
    in.pos_sats = 9;
    in.rx_fix_ok = 1;
    in.compass_ok = 1;
    in.rx_vdd_mv = 3900;
    in.testing_suspect = 0;
    return in;
}

/* The pad as the beacon runs it in PRE_LAUNCH: a heartbeat a minute, a
 * position packet only every 10 minutes. */
static Preflight_Inputs_t pad_heartbeat_paced(uint32_t hb_age_s)
{
    Preflight_Inputs_t in = all_green();
    in.link_age_s = hb_age_s;               /* the heartbeat is the newest */
    in.pos_age_s = 500;
    in.hb_valid = 1;
    in.hb_age_s = hb_age_s;
    in.hb_fix = 1;
    in.hb_sats = 9;
    return in;
}

TEST(test_all_green_is_ready)
{
    Preflight_Inputs_t in = all_green();
    Preflight_Result_t r;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY);
    CHECK(r.link_ok && r.tx_gps_ok && r.rx_gps_ok && r.compass_ok);
    CHECK(r.low_power == 0);
    CHECK(r.tx_fix == 1 && r.tx_sats == 9);

    in.rx_vdd_mv = 0;                       /* not measured: no veto */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY);
}

TEST(test_each_failing_check_blocks)
{
    Preflight_Result_t r;
    Preflight_Inputs_t in;

    in = all_green(); in.link_heard = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.link_ok == 0);

    in = all_green(); in.pos_sats = 3;     /* < 4 sats */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.tx_gps_ok == 0);

    in = all_green(); in.pos_fix = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.tx_gps_ok == 0);

    in = all_green(); in.rx_fix_ok = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.rx_gps_ok == 0);

    in = all_green(); in.compass_ok = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.compass_ok == 0);
}

TEST(test_heartbeat_paced_pad_link)
{
    /* The beacon's pad cadence must be able to hold READY: one heartbeat
     * a minute keeps the link fresh, a position every 10 minutes alone
     * never could (the old 35 s position-packet gate). */
    Preflight_Result_t r;
    Preflight_Inputs_t in = pad_heartbeat_paced(70);
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY && r.link_ok == 1);

    in = pad_heartbeat_paced(PF_LINK_FRESH_S);         /* boundary holds */
    Preflight_Evaluate(&in, &r);
    CHECK(r.link_ok == 1);

    in = pad_heartbeat_paced(80);                      /* a minute + lost */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.link_ok == 0);

    in = pad_heartbeat_paced(70); in.link_heard = 0;   /* never heard */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.link_ok == 0);
}

TEST(test_tx_gps_from_the_fresher_report)
{
    Preflight_Result_t r;
    Preflight_Inputs_t in;

    /* Pad: the 500 s-old position is stale, the 20 s-old heartbeat says
     * the beacon's GPS is fine - that is what the check reports */
    in = pad_heartbeat_paced(20); in.pos_sats = 2;
    Preflight_Evaluate(&in, &r);
    CHECK(r.tx_gps_ok == 1 && r.tx_sats == 9 && r.tx_fix == 1);

    /* ...and a fresh heartbeat that has lost the fix vetoes the old
     * position's good one */
    in = pad_heartbeat_paced(20); in.hb_fix = 0; in.hb_sats = 3;
    Preflight_Evaluate(&in, &r);
    CHECK(r.tx_gps_ok == 0 && r.tx_sats == 3 && r.tx_fix == 0);
    CHECK(r.verdict == PF_NOT_READY);

    /* An older heartbeat yields to a newer position packet */
    in = all_green();
    in.hb_valid = 1; in.hb_age_s = 300; in.hb_fix = 0; in.hb_sats = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.tx_gps_ok == 1 && r.tx_sats == 9);

    /* Heartbeats only (no position packet yet) */
    in = pad_heartbeat_paced(10); in.pos_valid = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.tx_gps_ok == 1 && r.verdict == PF_READY);

    /* Neither report: nothing to judge the beacon's GPS by */
    in = all_green(); in.pos_valid = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.tx_gps_ok == 0 && r.tx_fix == 0 && r.tx_sats == 0);
}

TEST(test_vetoes_and_their_priority)
{
    Preflight_Result_t r;
    Preflight_Inputs_t in;

    /* Test build vetoes an otherwise ready pad */
    in = all_green(); in.testing_suspect = 1;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_VETO_TEST_BUILD);

    /* ...but only judged while the link is up */
    in.link_heard = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY);
    in = all_green(); in.testing_suspect = 1; in.link_age_s = PF_LINK_FRESH_S + 1;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY);

    /* Low pack vetoes too; boundary is strict */
    in = all_green(); in.rx_vdd_mv = PF_LOW_POWER_MV - 1;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_VETO_LOW_POWER && r.low_power == 1);
    in.rx_vdd_mv = PF_LOW_POWER_MV;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY && r.low_power == 0);

    /* Both vetoes: the test build outranks the battery */
    in = all_green(); in.testing_suspect = 1; in.rx_vdd_mv = 3000;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_VETO_TEST_BUILD);

    /* Low power outranks failing checks */
    in = all_green(); in.compass_ok = 0; in.rx_vdd_mv = 3000;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_VETO_LOW_POWER);
}

TEST(test_null_safety)
{
    Preflight_Result_t r;
    memset(&r, 0xAB, sizeof(r));
    Preflight_Evaluate(NULL, &r);
    CHECK(r.verdict == PF_NOT_READY && r.link_ok == 0);
    Preflight_Inputs_t in = all_green();
    Preflight_Evaluate(&in, NULL);          /* must not crash */
}

int main(void)
{
    run_test_all_green_is_ready();
    run_test_each_failing_check_blocks();
    run_test_heartbeat_paced_pad_link();
    run_test_tx_gps_from_the_fresher_report();
    run_test_vetoes_and_their_priority();
    run_test_null_safety();
    return TEST_SUMMARY();
}
