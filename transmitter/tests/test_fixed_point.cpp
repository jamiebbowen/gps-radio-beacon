/**
 * test_fixed_point.cpp - saturating float -> fixed-point helpers
 *
 * Pins the edges every wire/flash encoder relies on: round-to-nearest,
 * saturation at the type limits (never wrap), and NaN/Inf handling.
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../firmware/include/fixed_point.h"
#include "test_harness.h"

TEST(test_sat_i16_rounds_in_range)
{
    CHECK(sat_i16(0.0f) == 0);
    CHECK(sat_i16(1.4f) == 1);
    CHECK(sat_i16(1.5f) == 2);                  /* half away from zero */
    CHECK(sat_i16(-1.5f) == -2);
    CHECK(sat_i16(32766.6f) == 32767);
    CHECK(sat_i16(-32767.6f) == -32768);
}

TEST(test_sat_i16_saturates_never_wraps)
{
    /* The bug class this exists for: 6 rad/s in centi-deg/s */
    CHECK(sat_i16(6.0f * DEG_PER_RAD * 100.0f) == INT16_MAX);
    CHECK(sat_i16(-6.0f * DEG_PER_RAD * 100.0f) == INT16_MIN);
    CHECK(sat_i16(32767.0f) == INT16_MAX);
    CHECK(sat_i16(-32768.0f) == INT16_MIN);
    CHECK(sat_i16(1e30f) == INT16_MAX);
    CHECK(sat_i16(-1e30f) == INT16_MIN);
    CHECK(sat_i16(INFINITY) == INT16_MAX);
    CHECK(sat_i16(-INFINITY) == INT16_MIN);
    CHECK(sat_i16(NAN) == 0);
}

TEST(test_sat_i32_edges)
{
    CHECK(sat_i32(0.0) == 0);
    CHECK(sat_i32(-104.8851678 * 1e7) == -1048851678);
    CHECK(sat_i32(2147483646.6) == INT32_MAX);
    CHECK(sat_i32(3e9) == INT32_MAX);
    CHECK(sat_i32(-3e9) == INT32_MIN);
    CHECK(sat_i32(INFINITY) == INT32_MAX);
    CHECK(sat_i32(-INFINITY) == INT32_MIN);
    CHECK(sat_i32(NAN) == 0);
}

TEST(test_unit_constants)
{
    CHECK_NEAR(MS2_PER_G, 9.80665, 1e-6);
    CHECK_NEAR(DEG_PER_RAD * (float)M_PI, 180.0, 1e-4);
}

int main(void)
{
    run_test_sat_i16_rounds_in_range();
    run_test_sat_i16_saturates_never_wraps();
    run_test_sat_i32_edges();
    run_test_unit_constants();
    return TEST_SUMMARY();
}
