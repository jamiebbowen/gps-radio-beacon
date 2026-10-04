/**
 * @file test_flight_events.cpp
 * @brief Pins the airborne life-cycle + anomaly detector (flight_events.cpp).
 *
 * What is being protected: every detection here is a certified radio event
 * (or the anomaly cadence override) on a link with ~1 packet per second of
 * spare airtime. A false APOGEE early in boost would arm the ballistic
 * grace timer against the wrong T0; a ballistic false fire on the pad
 * would burn the recovery window at emergency duty; a sticky anomaly that
 * never clears would key the PA at ~100% for the whole descent. Each rule
 * below exists to make exactly one of those failure modes a red test.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../firmware/include/flight_events.h"
#include "test_harness.h"

static flight_events_input_t in;
static uint32_t now_ms;
static int16_t ev_value;

static void reset(void)
{
    flight_events_init();
    memset(&in, 0, sizeof(in));
    now_ms = 1000;
    ev_value = -1;
}

/* Feed n times stepping the clock step_ms per pass; returns the first
 * non-NONE event code (0 if none fired across the whole run). */
static uint8_t run_ms(uint32_t total_ms, uint32_t step_ms)
{
    uint8_t ev = FLIGHT_EVENT_NONE;
    for (uint32_t t = 0; t < total_ms; t += step_ms) {
        now_ms += step_ms;
        int16_t v = 0;
        uint8_t e = flight_events_feed(&in, now_ms, &v);
        if (e != FLIGHT_EVENT_NONE && ev == FLIGHT_EVENT_NONE) {
            ev = e;
            ev_value = v;   /* snapshot at fire time; later feeds reset it */
        }
    }
    return ev;
}

static void scenario_airborne(int launched_long_ago)
{
    in.airborne = 1;
    in.nav_valid = 1;
    in.alt_m = 1000.0f;
    in.t_since_launch_s = launched_long_ago ? 30U : 1U;
}

/* ------------------------------------------------------------------ */

TEST(test_no_events_on_the_pad)
{
    reset();
    /* Bizarre-but-possible garbage on the pad must never declare events */
    scenario_airborne(0);
    in.airborne = 0;
    in.v_d_ms = 50.0f;              /* pure fiction pre-launch */
    in.gyro_mag_rads = 30.0f;       /* someone shaking the rail */
    in.imu_degraded = 1;
    in.gps_fix_age_ms = 60000;
    CHECK(run_ms(10000, 100) == FLIGHT_EVENT_NONE);
    CHECK(!flight_events_anomaly_active());
}

TEST(test_apogee_fires_once_after_hold)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = -40.0f;             /* ascending hard: nothing */
    CHECK(run_ms(3000, 100) == FLIGHT_EVENT_NONE);

    /* Noise band crossings must not trip: 1.9 m/s is under the arm level */
    in.v_d_ms = APOGEE_VD_MIN_MS - 0.1f;
    CHECK(run_ms(3000, 100) == FLIGHT_EVENT_NONE);

    /* Sustained real descent past the hold time: APOGEE, once, with alt */
    in.v_d_ms = 20.0f;
    uint8_t ev = run_ms(2000, 100);
    CHECK(ev == FLIGHT_EVENT_APOGEE);
    CHECK(ev_value == 1000);                        /* whole meters */
    CHECK(run_ms(5000, 100) != FLIGHT_EVENT_APOGEE); /* never twice */
}

TEST(test_apogee_rejects_brief_dips_and_early_launch)
{
    reset();
    scenario_airborne(1);
    /* 300 ms above the arm rate then back to ascent: hold not met */
    in.v_d_ms = 5.0f;
    run_ms(300, 100);
    in.v_d_ms = -10.0f;
    CHECK(run_ms(2000, 100) == FLIGHT_EVENT_NONE);

    /* Too soon after launch confirm: the anti-glitch floor rejects it */
    in.t_since_launch_s = APOGEE_MIN_T_SINCE_LAUNCH_S - 1;
    in.v_d_ms = 5.0f;
    CHECK(run_ms(2000, 100) == FLIGHT_EVENT_NONE);
}

TEST(test_drogue_then_main_step_down)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = 3.0f;               /* trip apogee first */
    CHECK(run_ms(2000, 100) == FLIGHT_EVENT_APOGEE);

    /* Drogue band center, sustained: DROGUE with the rate in cm/s */
    in.v_d_ms = 20.0f;
    uint8_t ev = run_ms(3000, 100);
    CHECK(ev == FLIGHT_EVENT_DROGUE);
    CHECK(ev_value == 2000);

    /* Dead-zone rate (neither band) declares nothing new on its own... */
    in.v_d_ms = 9.0f;               /* > MAIN_BAND_MAX, < DROGUE_BAND_MIN */
    run_ms(500, 100);               /* (below MAIN hold anyway) */

    /* ...then the canopy step: 6 m/s is under min(8, 0.5*20): MAIN */
    in.v_d_ms = 6.0f;
    ev = run_ms(3000, 100);
    CHECK(ev == FLIGHT_EVENT_MAIN);
    CHECK(ev_value == 600);
}

TEST(test_main_needs_a_real_prior_descent)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = 3.0f;
    run_ms(2000, 100);              /* apogee */
    /* The whole descent never exceeded 10 m/s: no deployment "detected"
     * out of thin air even though 6 m/s sits below MAIN_BAND_MAX. */
    in.v_d_ms = 6.0f;
    CHECK(run_ms(5000, 100) != FLIGHT_EVENT_MAIN);
}

TEST(test_ballistic_after_grace_and_clear_hysteresis)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = 3.0f;
    run_ms(2000, 100);              /* apogee at ~t+0  */

    /* Inside the grace window a fast descent is NOT yet ballistic */
    in.v_d_ms = 60.0f;
    CHECK(run_ms(ANOM_BALLISTIC_GRACE_S * 1000U - 500, 100) == FLIGHT_EVENT_NONE);
    CHECK(!flight_events_anomaly_active());

    /* Past grace + hold: ballistic entry edge, value = descent cm/s */
    CHECK(run_ms(2000, 100) == FLIGHT_EVENT_ANOM_BALLISTIC);
    CHECK(ev_value == 6000);
    CHECK(flight_events_anomaly_active());
    CHECK(flight_events_anomaly_code() == FLIGHT_EVENT_ANOM_BALLISTIC);

    /* Held condition is a level: no repeated entry edges... */
    CHECK(run_ms(3000, 100) == FLIGHT_EVENT_NONE);
    CHECK(flight_events_anomaly_active());

    /* ...recovery must persist past ANOM_CLEAR_MS before the level folds */
    in.v_d_ms = 4.0f;
    run_ms(ANOM_CLEAR_MS - 1000, 100);
    CHECK(flight_events_anomaly_active());
    run_ms(2000, 100);              /* crosses ANOM_CLEAR_MS of all-clear */
    CHECK(!flight_events_anomaly_active());
}

TEST(test_ballistic_grace_recovers_if_drogue_opens_late)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = 3.0f;
    run_ms(2000, 100);              /* apogee */
    /* Descent stabilizes INTO the drogue band before the grace expires:
     * ballistic must never fire even though the rate once exceeded it. */
    in.v_d_ms = 50.0f;
    run_ms(3000, 100);              /* fast, still inside grace */
    in.v_d_ms = 18.0f;              /* drogue opens late but opens */
    uint8_t ev = run_ms(5000, 100); /* grace + hold would elapse here */
    CHECK(ev != FLIGHT_EVENT_ANOM_BALLISTIC);
    CHECK(!flight_events_anomaly_active());
    CHECK(ev == FLIGHT_EVENT_DROGUE);
}

TEST(test_tumble_and_sensor_and_gps_outage)
{
    reset();
    scenario_airborne(1);

    /* Tumble: needs the hold, not a spike */
    in.gyro_mag_rads = ANOM_TUMBLE_RADS + 1.0f;
    run_ms(ANOM_TUMBLE_HOLD_MS - 100, 100);
    CHECK(!flight_events_anomaly_active());
    CHECK(run_ms(2000, 100) == FLIGHT_EVENT_ANOM_TUMBLE);
    CHECK(flight_events_anomaly_active());

    /* Clear it, start a fresh module for the next cause */
    reset();
    scenario_airborne(1);
    in.imu_degraded = 1;
    CHECK(run_ms(ANOM_SENSOR_HOLD_MS + 1000, 100) == FLIGHT_EVENT_ANOM_SENSOR_LOSS);

    reset();
    scenario_airborne(1);
    /* GPS outage is level-triggered: past the rescue horizon, airborne */
    in.gps_fix_age_ms = ANOM_GPS_OUTAGE_MS + 1;
    CHECK(run_ms(300, 100) == FLIGHT_EVENT_ANOM_GPS_OUTAGE);
}

TEST(test_anomaly_priority_is_worst_first)
{
    reset();
    scenario_airborne(1);
    in.v_d_ms = 3.0f;
    run_ms(2000, 100);              /* apogee */
    /* Ballistic + tumble + GPS hole simultaneously: ballistic reports */
    in.v_d_ms = 80.0f;
    in.gyro_mag_rads = ANOM_TUMBLE_RADS + 5.0f;
    in.gps_fix_age_ms = ANOM_GPS_OUTAGE_MS + 1;
    run_ms(ANOM_BALLISTIC_GRACE_S * 1000U + 2000, 100);
    CHECK(flight_events_anomaly_code() == FLIGHT_EVENT_ANOM_BALLISTIC);
}

TEST(test_kinematics_gated_on_nav_valid)
{
    reset();
    scenario_airborne(1);
    /* Un-anchored EKF: v_d is fiction -> no apogee, no ballistic. But the
     * sensor-plumbing anomalies (tumble/GPS outage) still work. */
    in.nav_valid = 0;
    in.v_d_ms = 50.0f;
    CHECK(run_ms(5000, 100) == FLIGHT_EVENT_NONE);
    in.gps_fix_age_ms = ANOM_GPS_OUTAGE_MS + 1;
    CHECK(run_ms(500, 100) == FLIGHT_EVENT_ANOM_GPS_OUTAGE);
}

TEST(test_landing_stops_the_world)
{
    reset();
    scenario_airborne(1);
    in.imu_degraded = 1;
    run_ms(ANOM_SENSOR_HOLD_MS + 1000, 100);       /* anomaly latches */
    CHECK(flight_events_anomaly_active());
    in.airborne = 0;                                /* landed latch trips */
    in.imu_degraded = 0;
    in.v_d_ms = 50.0f;                              /* impact fiction */
    CHECK(run_ms(ANOM_CLEAR_MS + 5000, 100) == FLIGHT_EVENT_NONE);
    CHECK(!flight_events_anomaly_active());
}

int main(void)
{
    run_test_no_events_on_the_pad();
    run_test_apogee_fires_once_after_hold();
    run_test_apogee_rejects_brief_dips_and_early_launch();
    run_test_drogue_then_main_step_down();
    run_test_main_needs_a_real_prior_descent();
    run_test_ballistic_after_grace_and_clear_hysteresis();
    run_test_ballistic_grace_recovers_if_drogue_opens_late();
    run_test_tumble_and_sensor_and_gps_outage();
    run_test_anomaly_priority_is_worst_first();
    run_test_kinematics_gated_on_nav_valid();
    run_test_landing_stops_the_world();
    return TEST_SUMMARY();
}
