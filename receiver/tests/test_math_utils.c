/**
 * @file test_math_utils.c
 * @brief Host-side unit tests for receiver GPS math utilities.
 *
 * Covers calculate_distance (haversine), calculate_bearing, and
 * normalize_angle - the functions that drive the receiver's distance
 * readout and navigation arrow.
 */

#include <stdio.h>
#include <math.h>

#include "math_utils.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/math_utils.c"

/* ------------------------------------------------------------------ */
/* normalize_angle                                                     */
/* ------------------------------------------------------------------ */

TEST(test_normalize_angle) {
    CHECK_NEAR(normalize_angle(0.0f), 0.0f, 1e-6);
    CHECK_NEAR(normalize_angle(359.9f), 359.9f, 1e-4);
    CHECK_NEAR(normalize_angle(360.0f), 0.0f, 1e-4);
    CHECK_NEAR(normalize_angle(-90.0f), 270.0f, 1e-4);
    CHECK_NEAR(normalize_angle(725.0f), 5.0f, 1e-3);
    CHECK_NEAR(normalize_angle(-725.0f), 355.0f, 1e-3);
}

/* ------------------------------------------------------------------ */
/* calculate_distance (haversine)                                      */
/* ------------------------------------------------------------------ */

TEST(test_distance_zero_for_same_point) {
    CHECK_NEAR(calculate_distance(39.89f, -105.11f, 39.89f, -105.11f), 0.0f, 0.1);
}

TEST(test_distance_one_degree_latitude) {
    /* 1 degree of latitude = R * pi/180 = ~111,195 m with R = 6,371 km */
    float d = calculate_distance(39.0f, -105.0f, 40.0f, -105.0f);
    CHECK_NEAR(d, 111195.0, 300.0);
}

TEST(test_distance_one_degree_longitude_at_latitude) {
    /* 1 degree of longitude at 39N = 111,195 * cos(39 deg) = ~86,414 m */
    float d = calculate_distance(39.0f, -105.0f, 39.0f, -104.0f);
    CHECK_NEAR(d, 111195.0 * cos(39.0 * M_PI / 180.0), 300.0);
}

TEST(test_distance_symmetry) {
    float d1 = calculate_distance(39.89f, -105.11f, 40.05f, -104.80f);
    float d2 = calculate_distance(40.05f, -104.80f, 39.89f, -105.11f);
    CHECK_NEAR(d1, d2, 1.0);
}

TEST(test_distance_typical_rocket_flight) {
    /* ~2 km downrange drift - representative recovery scenario.
     * 0.018 deg of latitude = ~2001.5 m */
    float d = calculate_distance(39.8900f, -105.1155f, 39.9080f, -105.1155f);
    CHECK_NEAR(d, 2001.5, 20.0);
}

TEST(test_distance_never_nan_at_extremes) {
    /* Antipodal points and float-identical points are where the haversine
     * 'a' can round past 1.0; sqrtf(1-a) then returns NaN. The clamp must
     * hold at both extremes. */
    float d_anti = calculate_distance(39.89f, -105.11f, -39.89f, 74.89f);
    CHECK(d_anti == d_anti);          /* not NaN */
    CHECK_NEAR(d_anti, 20015086.0, 5000.0);   /* ~half earth circumference */

    float d_same = calculate_distance(39.89f, -105.11f, 39.89f, -105.11f);
    CHECK(d_same == d_same);
    CHECK(d_same >= 0.0f && d_same < 0.001f);

    /* Sub-millimetre separation: the classic a > 1 rounding trigger */
    float d_tiny = calculate_distance(39.8900000f, -105.1100000f,
                                      39.8900001f, -105.1100001f);
    CHECK(d_tiny == d_tiny);
    CHECK(d_tiny >= 0.0f && d_tiny < 1.0f);
}

TEST(test_distance_antimeridian_crossing) {
    /* Drift across the dateline: (39.5N, 179.999E) -> (39.5N, 179.999W)
     * is 0.002 deg of longitude - ~172 m at that latitude - not 40,000 km.
     * If a future refactor ever computes raw (lon2 - lon1) without the
     * trig-identity safety of haversine, this is where it breaks. */
    float d = calculate_distance(39.5f, 179.999f, 39.5f, -179.999f);
    CHECK(d == d);                                     /* not NaN */
    CHECK_NEAR(d, 171.7, 5.0);

    /* Larger crossing at the equator: 0.02 deg ~ 2.2 km */
    CHECK_NEAR(calculate_distance(0.0f, 179.99f, 0.0f, -179.99f), 2224.0, 25.0);
}

TEST(test_distance_recovery_walk_up_range) {
    /* Final approach: walking the last few metres to the rocket. The
     * answer must stay sane (no NaN, no zero-collapse, right ballpark).
     * Note the physical floor: float32 coordinates at 40N quantize to
     * ~0.33 m per ulp, and BOTH endpoints jitter, so sub-2 m readings are
     * inherently +/-30%-ish - fine for "it's within arm's reach", which is
     * all this test pins. (Measured: 1.5 m reads ~1.9 m.) An upgrade to
     * double intermediates would help only if the display ever shows
     * sub-metre distances. */
    /* 1.5 m of latitude at 40N = 0.0000135 deg */
    float d = calculate_distance(39.8900000f, -105.1155f,
                                 39.8900135f, -105.1155f);
    CHECK(d == d);
    CHECK(d > 1.0f && d < 2.5f);

    /* Same scale on longitude (~85 km/deg at 40N): 2 m = 0.0000235 deg */
    float d_lon = calculate_distance(39.89f, -105.1155000f,
                                     39.89f, -105.1154765f);
    CHECK(d_lon > 1.2f && d_lon < 3.0f);

    /* A comfortable recovery-display distance must be accurate, though:
     * 50 m downrange at 40N = 0.0004492 deg lat */
    float d_far = calculate_distance(39.8900000f, -105.1155f,
                                     39.8904492f, -105.1155f);
    CHECK_NEAR(d_far, 50.0, 1.5);
}

/* ------------------------------------------------------------------ */
/* calculate_bearing                                                   */
/* ------------------------------------------------------------------ */

TEST(test_bearing_cardinal_directions) {
    /* Due north */
    CHECK_NEAR(calculate_bearing(39.0f, -105.0f, 40.0f, -105.0f), 0.0f, 0.5);
    /* Due south */
    CHECK_NEAR(calculate_bearing(40.0f, -105.0f, 39.0f, -105.0f), 180.0f, 0.5);
    /* Due east (small offset so great-circle curvature stays negligible) */
    CHECK_NEAR(calculate_bearing(39.0f, -105.0f, 39.0f, -104.9f), 90.0f, 0.5);
    /* Due west */
    CHECK_NEAR(calculate_bearing(39.0f, -105.0f, 39.0f, -105.1f), 270.0f, 0.5);
}

TEST(test_bearing_diagonal) {
    /* Equal small offsets north and east at the equator = ~45 degrees */
    CHECK_NEAR(calculate_bearing(0.0f, 0.0f, 0.1f, 0.1f), 45.0f, 0.5);
}

TEST(test_bearing_always_in_range) {
    /* Sweep a ring of targets; result must always be [0, 360) */
    for (int deg = 0; deg < 360; deg += 30) {
        float rad = (float)deg * (float)M_PI / 180.0f;
        float b = calculate_bearing(39.0f, -105.0f,
                                    39.0f + 0.1f * cosf(rad),
                                    -105.0f + 0.1f * sinf(rad));
        CHECK(b >= 0.0f && b < 360.0f);
    }
}

TEST(test_bearing_across_antimeridian) {
    /* Due east across the dateline: 179.99E -> 179.99W is a 0.02 deg hop
     * EAST, so the pointer must read ~90, not ~270. */
    CHECK_NEAR(calculate_bearing(39.5f, 179.99f, 39.5f, -179.99f), 90.0f, 2.0);
    /* And the reverse walk points back west */
    CHECK_NEAR(calculate_bearing(39.5f, -179.99f, 39.5f, 179.99f), 270.0f, 2.0);
}

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_normalize_angle();
    run_test_distance_zero_for_same_point();
    run_test_distance_one_degree_latitude();
    run_test_distance_one_degree_longitude_at_latitude();
    run_test_distance_symmetry();
    run_test_distance_typical_rocket_flight();
    run_test_distance_never_nan_at_extremes();
    run_test_distance_antimeridian_crossing();
    run_test_distance_recovery_walk_up_range();
    run_test_bearing_cardinal_directions();
    run_test_bearing_diagonal();
    run_test_bearing_always_in_range();
    run_test_bearing_across_antimeridian();

    return TEST_SUMMARY();
}
