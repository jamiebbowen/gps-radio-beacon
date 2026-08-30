/**
 * @file test_nav.cpp
 * @brief Host-side unit tests for the GPS+IMU fusion glue layer (nav.cpp).
 *
 * Runs the REAL 6-state EKF (linked ekf.cpp) under nav.cpp, with fakes for
 * the IMU accessors and millis(). Verifies tangent-plane anchoring, the
 * ENU->NED axis swap, GPS gating (fix quality / sat count), fused-snapshot
 * freshness flags (gps_fresh / dead_reckoning / imu_healthy) and the
 * lat/lon <-> NED round trip.
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include "include/nav.h"
#include "include/launch_detect.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino + IMU fakes                                                 */
/* ------------------------------------------------------------------ */

FakeSerial Serial;

static unsigned long now_ms = 100000;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }

static bool     fake_has_quat = false;
static float    fake_quat[4]  = {1, 0, 0, 0};        /* w,x,y,z identity */
static float    fake_accel[3] = {0, 0, 0};           /* body frame */
static uint32_t fake_accel_ms = 0;

bool imu_has_quaternion(void) { return fake_has_quat; }
void imu_get_quaternion(float *w, float *x, float *y, float *z)
{
    *w = fake_quat[0]; *x = fake_quat[1];
    *y = fake_quat[2]; *z = fake_quat[3];
}
void imu_get_linear_accel_body(float *x, float *y, float *z)
{
    *x = fake_accel[0]; *y = fake_accel[1]; *z = fake_accel[2];
}
uint32_t imu_last_accel_ms(void) { return fake_accel_ms; }

/* Include the module under test (real ekf.cpp is linked separately) */
#include "../firmware/nav.cpp"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

#define ANCHOR_LAT 39.8900000f
#define ANCHOR_LON -104.8850000f
#define ANCHOR_ALT 1650.0f

static void step(uint32_t ms)
{
    now_ms += ms;
    nav_predict();
}

static void fresh_nav_with_anchor(void)
{
    nav_init();
    fake_has_quat = false;
    memset(fake_accel, 0, sizeof(fake_accel));
    fake_accel_ms = 0;
    /* Anchoring requires two consecutive agreeing fixes (glitch guard) */
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    step(1000);
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_invalid_until_anchored)
{
    nav_init();
    CHECK(nav_is_valid() == false);

    NavFused_t f;
    memset(&f, 0xAA, sizeof(f));
    nav_get_fused(&f);
    CHECK(f.valid == false);

    /* Predict without an anchor must be harmless */
    nav_predict();          /* primes the dt clock */
    now_ms += 100;
    nav_predict();          /* runs but bails on !anchored */
    CHECK(nav_is_valid() == false);
}

TEST(test_gps_gating)
{
    nav_init();

    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 0);  /* no fix */
    CHECK(nav_is_valid() == false);

    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 3, 1);  /* < 4 sats */
    CHECK(nav_is_valid() == false);

    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 4, 1);  /* candidate */
    CHECK(nav_is_valid() == false);               /* one fix only seeds a candidate */

    now_ms += 1000;
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 4, 1);  /* confirms */
    CHECK(nav_is_valid() == true);
}

TEST(test_anchor_requires_two_agreeing_fixes)
{
    nav_init();
    fake_has_quat = false;
    fake_accel_ms = 0;

    /* Fix 1: candidate. Fix 2: 111 m away (0.001 deg) - disagreement,
     * candidate replaced, still not anchored. */
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_is_valid() == false);
    now_ms += 1000;
    nav_update_from_gps(ANCHOR_LAT + 0.001f, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_is_valid() == false);

    /* Fix 3 agrees with fix 2: anchors THERE (the newer position). */
    now_ms += 1000;
    nav_update_from_gps(ANCHOR_LAT + 0.001f, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_is_valid() == true);
    NavFused_t f;
    nav_get_fused(&f);
    CHECK(fabsf(f.lat_deg - (ANCHOR_LAT + 0.001f)) < 1e-5f);
}

TEST(test_anchor_candidate_expires)
{
    nav_init();
    fake_has_quat = false;
    fake_accel_ms = 0;

    /* Candidate goes stale after 10 s: an agreeing fix that arrives late
     * must NOT confirm it (could be hours later after a power blip). */
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    now_ms += 20000;
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_is_valid() == false);
    now_ms += 1000;
    nav_update_from_gps(ANCHOR_LAT, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_is_valid() == true);
}

TEST(test_anchor_and_roundtrip)
{
    fresh_nav_with_anchor();
    CHECK(nav_is_valid() == true);

    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.valid == true);
    /* At the anchor with zero EKF state, fused == anchor */
    CHECK(fabsf(f.lat_deg - ANCHOR_LAT) < 1e-5f);
    CHECK(fabsf(f.lon_deg - ANCHOR_LON) < 1e-5f);
    CHECK(fabsf(f.alt_m   - ANCHOR_ALT) < 0.1f);
    CHECK(fabsf(f.v_n) < 0.01f && fabsf(f.v_e) < 0.01f && fabsf(f.v_d) < 0.01f);
    CHECK(f.gps_fresh == true);
    CHECK(f.dead_reckoning == false);
    CHECK(f.imu_healthy == false);          /* no IMU data yet */
}

TEST(test_gps_updates_pull_position)
{
    fresh_nav_with_anchor();

    /* Feed fixes ~111 m north of the anchor; EKF should converge there */
    float north_lat = ANCHOR_LAT + 0.001f;
    for (int i = 0; i < 20; i++) {
        step(100);
        nav_update_from_gps(north_lat, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    }

    NavFused_t f;
    nav_get_fused(&f);
    /* Converged most of the way north (EKF blends, so allow slack) */
    CHECK(f.lat_deg > ANCHOR_LAT + 0.0007f);
    CHECK(f.lat_deg < ANCHOR_LAT + 0.0011f);
    CHECK(fabsf(f.lon_deg - ANCHOR_LON) < 1e-4f);
    /* Residual logging exercised (IMU_FUSION_LOG_RESIDUALS=1) */
    CHECK(strstr(Serial.log, "[Nav] GPS update") != NULL);
}

TEST(test_predict_axis_mapping_enu_to_ned)
{
    fresh_nav_with_anchor();

    /* Identity quaternion: body == ENU. Body +Z accel = ENU up = NED -down,
     * so v_d must go NEGATIVE (climbing). */
    fake_has_quat = true;
    fake_quat[0] = 1; fake_quat[1] = 0; fake_quat[2] = 0; fake_quat[3] = 0;
    fake_accel[0] = 0; fake_accel[1] = 0; fake_accel[2] = 10.0f;
    fake_accel_ms = now_ms;

    nav_predict();                 /* prime dt */
    for (int i = 0; i < 10; i++) {
        now_ms += 100;
        fake_accel_ms = now_ms;
        nav_predict();
    }

    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.v_d < -5.0f);          /* ~1 s of 10 m/s^2 up */
    CHECK(fabsf(f.v_n) < 0.5f && fabsf(f.v_e) < 0.5f);
    CHECK(f.imu_healthy == true);  /* fresh accel + quaternion */

    /* Body +X accel with identity quat = ENU east = NED east: v_e grows */
    fresh_nav_with_anchor();
    fake_has_quat = true;
    fake_accel[0] = 10.0f; fake_accel[1] = 0; fake_accel[2] = 0;
    nav_predict();
    for (int i = 0; i < 10; i++) {
        now_ms += 100;
        fake_accel_ms = now_ms;
        nav_predict();
    }
    nav_get_fused(&f);
    CHECK(f.v_e > 5.0f);
    CHECK(fabsf(f.v_n) < 0.5f);

    /* Body +Y accel = ENU north = NED north: v_n grows */
    fresh_nav_with_anchor();
    fake_has_quat = true;
    fake_accel[0] = 0; fake_accel[1] = 10.0f; fake_accel[2] = 0;
    nav_predict();
    for (int i = 0; i < 10; i++) {
        now_ms += 100;
        fake_accel_ms = now_ms;
        nav_predict();
    }
    nav_get_fused(&f);
    CHECK(f.v_n > 5.0f);
    CHECK(fabsf(f.v_e) < 0.5f);
}

TEST(test_innovation_gate_rejects_gps_glitch)
{
    fresh_nav_with_anchor();
    CHECK(nav_get_gps_rejects() == 0);

    /* IMU healthy: the gate is armed */
    fake_has_quat = true;
    fake_quat[0] = 1.0f;
    fake_accel_ms = now_ms;
    now_ms += 100;
    nav_predict();

    /* A 111 m multipath jump against a tight filter: rejected, no movement */
    nav_update_from_gps(ANCHOR_LAT + 0.001f, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    NavFused_t f;
    nav_get_fused(&f);
    CHECK(fabsf(f.lat_deg - ANCHOR_LAT) < 1e-5f);
    CHECK(nav_get_gps_rejects() == 1);
    CHECK(strstr(Serial.log, "REJECTED") != NULL);

    /* An honest 3 m step: under the 20 m floor, always accepted */
    fake_accel_ms = now_ms;
    nav_update_from_gps(ANCHOR_LAT + 0.00003f, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_get_gps_rejects() == 1);              /* no new rejection */
    nav_get_fused(&f);
    CHECK(f.lat_deg > ANCHOR_LAT);                  /* pulled north */
}

TEST(test_innovation_gate_fails_open_when_imu_dead)
{
    fresh_nav_with_anchor();

    /* IMU explicitly dead (no quaternion): gate off, big jump flows through */
    fake_has_quat = false;
    nav_update_from_gps(ANCHOR_LAT + 0.001f, ANCHOR_LON, ANCHOR_ALT, 8, 1);
    CHECK(nav_get_gps_rejects() == 0);
    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.lat_deg > ANCHOR_LAT + 0.0003f);  /* one update, Kp=0.5: ~55 m */
}

TEST(test_rotated_quaternion)
{
    /* 90 deg yaw about Z: body +X maps to ENU +Y (north) */
    fresh_nav_with_anchor();
    fake_has_quat = true;
    float s = sinf((float)M_PI / 4.0f);
    fake_quat[0] = cosf((float)M_PI / 4.0f);  /* w */
    fake_quat[1] = 0; fake_quat[2] = 0; fake_quat[3] = s;  /* z */
    fake_accel[0] = 10.0f; fake_accel[1] = 0; fake_accel[2] = 0;

    nav_predict();
    for (int i = 0; i < 10; i++) {
        now_ms += 100;
        fake_accel_ms = now_ms;
        nav_predict();
    }

    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.v_n > 5.0f);           /* rotated into north */
    CHECK(fabsf(f.v_e) < 0.5f);
}

TEST(test_zero_dt_predict_is_noop)
{
    fresh_nav_with_anchor();
    fake_has_quat = true;
    fake_accel[2] = 10.0f;

    nav_predict();                 /* prime */
    now_ms += 100;
    nav_predict();
    NavFused_t before;
    nav_get_fused(&before);

    nav_predict();                 /* same millis: dt == 0, must not step */
    NavFused_t after;
    nav_get_fused(&after);
    CHECK(before.v_d == after.v_d);
}

TEST(test_freshness_flags_age_out)
{
    fresh_nav_with_anchor();
    fake_has_quat = true;
    fake_accel_ms = now_ms;

    NavFused_t f;
    nav_get_fused(&f);
    CHECK(f.gps_fresh == true && f.dead_reckoning == false);
    CHECK(f.age_ds == 0);

    /* 2 s: not fresh, but not dead-reckoning yet (DR at 3 s) */
    now_ms += 2000;
    nav_get_fused(&f);
    CHECK(f.gps_fresh == false);
    CHECK(f.dead_reckoning == false);
    CHECK(f.age_ds == 20);
    CHECK(f.imu_healthy == false);          /* accel is 2 s stale too */

    /* 30 s: dead reckoning; age saturates at 255 ds */
    now_ms += 28000;
    nav_get_fused(&f);
    CHECK(f.dead_reckoning == true);
    CHECK(f.age_ds == 255);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_invalid_until_anchored();
    run_test_gps_gating();
    run_test_anchor_requires_two_agreeing_fixes();
    run_test_anchor_candidate_expires();
    run_test_anchor_and_roundtrip();
    run_test_gps_updates_pull_position();
    run_test_innovation_gate_rejects_gps_glitch();
    run_test_innovation_gate_fails_open_when_imu_dead();
    run_test_predict_axis_mapping_enu_to_ned();
    run_test_rotated_quaternion();
    run_test_zero_dt_predict_is_noop();
    run_test_freshness_flags_age_out();

    return TEST_SUMMARY();
}
