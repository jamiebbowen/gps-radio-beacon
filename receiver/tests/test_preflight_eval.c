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
    in.link_ok = 1;
    in.tx_fix = 1;
    in.tx_sats = 9;
    in.rx_fix_ok = 1;
    in.compass_ok = 1;
    in.rx_vdd_mv = 3900;
    in.testing_suspect = 0;
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

    in.rx_vdd_mv = 0;                       /* not measured: no veto */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_READY);
}

TEST(test_each_failing_check_blocks)
{
    Preflight_Result_t r;
    Preflight_Inputs_t in;

    in = all_green(); in.link_ok = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.link_ok == 0);

    in = all_green(); in.tx_sats = 3;      /* < 4 sats */
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.tx_gps_ok == 0);

    in = all_green(); in.tx_fix = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.tx_gps_ok == 0);

    in = all_green(); in.rx_fix_ok = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.rx_gps_ok == 0);

    in = all_green(); in.compass_ok = 0;
    Preflight_Evaluate(&in, &r);
    CHECK(r.verdict == PF_NOT_READY && r.compass_ok == 0);
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
    in.link_ok = 0;
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
    run_test_vetoes_and_their_priority();
    run_test_null_safety();
    return TEST_SUMMARY();
}
