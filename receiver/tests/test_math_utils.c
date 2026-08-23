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

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_normalize_angle();
    run_test_distance_zero_for_same_point();
    run_test_distance_one_degree_latitude();
    run_test_distance_one_degree_longitude_at_latitude();
    run_test_distance_symmetry();
    run_test_distance_typical_rocket_flight();
    run_test_bearing_cardinal_directions();
    run_test_bearing_diagonal();
    run_test_bearing_always_in_range();

    return TEST_SUMMARY();
}
