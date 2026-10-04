/**
 * @file test_flight_maxima.cpp
 * @brief Pins the running flight maxima tracker (flight_maxima.cpp).
 *
 * These numbers are the "black box on the air": every MAXIMA packet is a
 * cumulative flight record, so tracking bugs are silent forensic lies.
 * The rules that matter: pad handling never counts, an un-anchored EKF
 * never contributes position/velocity, inertial channels stand alone
 * (a dead-GPS boost still records accel/gyro), and wire clamps hold.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../firmware/include/flight_maxima.h"
#include "test_harness.h"

static flight_maxima_input_t in;
static MaximaPacket_t out;

static void reset(void)
{
    flight_maxima_init();
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.airborne = 1;
    in.nav_valid = 1;
}

/* ------------------------------------------------------------------ */

TEST(test_pad_handling_never_counts)
{
    reset();
    in.airborne = 0;                     /* on the rail / in someone's hand */
    in.alt_m = 9999.0f;                  /* fiction                       */
    in.v_d_ms = -80.0f;
    in.accel_ms2 = 300.0f;               /* rail bump                     */
    in.gyro_mag_rads = 2.0f;
    in.uptime_s = 10;

    flight_maxima_feed(&in);
    flight_maxima_get(&out);
    CHECK(out.max_alt_m == 0);           /* nothing recorded              */
    CHECK(out.max_speed_cms == 0);
    CHECK(out.max_accel_cg == 0);
    CHECK(out.max_gyro_dps == 0);
}

TEST(test_maxima_track_peaks_and_time_of_alt)
{
    reset();
    /* Climb profile: alt peaks at t=30 s, speed peaks earlier */
    in.uptime_s = 20; in.alt_m = 1500.0f;
    in.v_n_ms = 10.0f; in.v_e_ms = 0.0f; in.v_d_ms = -100.0f;   /* ~100.5 m/s */
    in.accel_ms2 = 4.0f * 9.80665f;
    in.gyro_mag_rads = 0.0f;
    flight_maxima_feed(&in);

    in.uptime_s = 30; in.alt_m = 2200.0f;
    in.v_n_ms = 0.0f; in.v_e_ms = 0.0f; in.v_d_ms = -30.0f;
    flight_maxima_feed(&in);

    in.uptime_s = 40; in.alt_m = 1800.0f;   /* descending from the peak */
    in.v_d_ms = 15.0f;
    in.gyro_mag_rads = 3.5f;                /* 3.5 rad/s = 200.5 dps */
    flight_maxima_feed(&in);

    flight_maxima_get(&out);
    CHECK(out.max_alt_m == 2200);
    CHECK(out.t_maxalt_s == 30);            /* time of the peak, not of now */
    /* |v| peak = sqrt(10^2 + 100^2) cm/s = 10049.9 */
    CHECK(out.max_speed_cms == 10050);
    CHECK(out.max_accel_cg == 400);         /* 4 g */
    CHECK(out.max_gyro_dps == 201);         /* ~200.5 dps */
}

TEST(test_unanchored_nav_contributes_no_kinematics)
{
    reset();
    in.nav_valid = 0;
    in.alt_m = 5000.0f;                /* EKF fiction pre-anchor */
    in.v_n_ms = 300.0f;
    in.accel_ms2 = 50.0f;              /* ~5.1 g - REAL, still counts */
    in.uptime_s = 5;
    flight_maxima_feed(&in);

    flight_maxima_get(&out);
    CHECK(out.max_alt_m == 0);
    CHECK(out.max_speed_cms == 0);
    CHECK(out.max_accel_cg == 510);    /* inertial stands alone */
}

TEST(test_landing_latch_freezes_the_record)
{
    reset();
    in.alt_m = 1000.0f; in.uptime_s = 7; in.accel_ms2 = 98.0f;  /* ~10 g */
    flight_maxima_feed(&in);
    /* Touchdown impact: airborne latch cleared -> impact spike must NOT
     * overwrite the in-flight peaks (post-topple spikes are not flight) */
    in.airborne = 0;
    in.alt_m = 9000.0f; in.accel_ms2 = 500.0f;
    flight_maxima_feed(&in);

    flight_maxima_get(&out);
    CHECK(out.max_alt_m == 1000);
    CHECK(out.t_maxalt_s == 7);
    CHECK(out.max_accel_cg == 999 || out.max_accel_cg == 1000);
}

TEST(test_wire_range_clamps)
{
    reset();
    in.alt_m = 40000.0f;               /* beyond int16 m */
    in.v_d_ms = 700.0f;                /* beyond uint16 cm/s */
    in.accel_ms2 = 700.0f * 9.80665f;  /* beyond uint16 cg */
    in.uptime_s = 70000;               /* beyond uint16 s */
    flight_maxima_feed(&in);

    flight_maxima_get(&out);
    CHECK(out.max_alt_m == 32767);
    CHECK(out.max_speed_cms == 65535);
    CHECK(out.max_accel_cg == 65535);
    CHECK(out.t_maxalt_s == 65535);
}

int main(void)
{
    run_test_pad_handling_never_counts();
    run_test_maxima_track_peaks_and_time_of_alt();
    run_test_unanchored_nav_contributes_no_kinematics();
    run_test_landing_latch_freezes_the_record();
    run_test_wire_range_clamps();
    return TEST_SUMMARY();
}
