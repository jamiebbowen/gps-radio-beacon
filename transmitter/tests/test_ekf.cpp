/**
 * @file test_ekf.cpp
 * @brief Host-side unit tests for the transmitter's 6-state GPS/IMU EKF.
 *
 * ekf.cpp depends only on <math.h>/<string.h>, so it compiles unmodified
 * with the host compiler. Tests cover initialization, the predict-step
 * kinematics, dt sanity gating, covariance growth/shrinkage, GPS update
 * convergence, and numerical symmetry of the covariance matrix.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "include/ekf.h"
#include "test_harness.h"

/* Tuning values mirroring config.h defaults */
#define SIGMA_A      0.5f
#define SIGMA_GPS_H  3.0f
#define SIGMA_GPS_V  6.0f
#define INIT_POS_VAR 25.0f
#define INIT_VEL_VAR 4.0f

static void make_filter(Ekf_t *e) {
    ekf_init(e, SIGMA_A, SIGMA_GPS_H, SIGMA_GPS_V, INIT_POS_VAR, INIT_VEL_VAR);
}

/* ------------------------------------------------------------------ */
/* Initialization / anchoring                                          */
/* ------------------------------------------------------------------ */

TEST(test_init_state) {
    Ekf_t e;
    make_filter(&e);

    CHECK(e.initialized == 0);
    for (int i = 0; i < 6; i++) CHECK_NEAR(e.x[i], 0.0f, 1e-9);
    for (int i = 0; i < 3; i++) CHECK_NEAR(e.P[i][i], INIT_POS_VAR, 1e-6);
    for (int i = 3; i < 6; i++) CHECK_NEAR(e.P[i][i], INIT_VEL_VAR, 1e-6);
}

TEST(test_update_rejected_before_anchor) {
    Ekf_t e;
    make_filter(&e);

    float z[3] = {1.0f, 2.0f, 3.0f};
    CHECK(ekf_update_gps_position(&e, z) == 0);   /* not anchored yet */

    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);
    CHECK(e.initialized == 1);
    CHECK(ekf_update_gps_position(&e, z) == 1);   /* accepted after anchor */
}

TEST(test_set_position_anchors_and_zeroes_velocity) {
    Ekf_t e;
    make_filter(&e);
    e.x[3] = 5.0f;  /* pretend stale velocity */

    ekf_set_position(&e, 10.0f, -20.0f, 30.0f);
    CHECK_NEAR(e.x[0], 10.0f, 1e-6);
    CHECK_NEAR(e.x[1], -20.0f, 1e-6);
    CHECK_NEAR(e.x[2], 30.0f, 1e-6);
    CHECK_NEAR(e.x[3], 0.0f, 1e-9);
    /* Position uncertainty collapses to GPS noise level */
    CHECK_NEAR(e.P[0][0], SIGMA_GPS_H * SIGMA_GPS_H, 1e-4);
    CHECK_NEAR(e.P[2][2], SIGMA_GPS_V * SIGMA_GPS_V, 1e-4);
}

/* ------------------------------------------------------------------ */
/* Predict step                                                        */
/* ------------------------------------------------------------------ */

TEST(test_predict_rejects_absurd_dt) {
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 1.0f, 2.0f, 3.0f);

    float a[3] = {100.0f, 100.0f, 100.0f};
    ekf_predict(&e, a, 0.0f);    /* dt == 0    */
    ekf_predict(&e, a, -0.1f);   /* dt  < 0    */
    ekf_predict(&e, a, 2.0f);    /* dt  > 1 s  */

    CHECK_NEAR(e.x[0], 1.0f, 1e-9);  /* state untouched */
    CHECK_NEAR(e.x[3], 0.0f, 1e-9);
}

TEST(test_predict_constant_acceleration_kinematics) {
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);

    /* 1 m/s^2 north for 1 s in 10 ms steps: v = 1 m/s, p = 0.5 m */
    float a[3] = {1.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++) {
        ekf_predict(&e, a, 0.01f);
    }
    CHECK_NEAR(ekf_get_vn(&e), 1.0f, 1e-3);
    CHECK_NEAR(ekf_get_pn(&e), 0.5f, 1e-3);
    /* Other axes untouched */
    CHECK_NEAR(ekf_get_pe(&e), 0.0f, 1e-6);
    CHECK_NEAR(ekf_get_vd(&e), 0.0f, 1e-6);
}

TEST(test_predict_grows_uncertainty) {
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);

    float p0 = e.P[0][0];
    float a[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++) {
        ekf_predict(&e, a, 0.01f);
    }
    CHECK(e.P[0][0] > p0);  /* coasting must increase position variance */
}

/* ------------------------------------------------------------------ */
/* GPS measurement update                                              */
/* ------------------------------------------------------------------ */

TEST(test_update_converges_to_static_truth) {
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);

    /* Truth is a stationary point; simulate 30 s of 1 Hz GPS at 100 Hz IMU */
    float truth[3] = {10.0f, -5.0f, 2.0f};
    float a[3] = {0.0f, 0.0f, 0.0f};
    for (int s = 0; s < 30; s++) {
        for (int i = 0; i < 100; i++) {
            ekf_predict(&e, a, 0.01f);
        }
        CHECK(ekf_update_gps_position(&e, truth) == 1);
    }

    CHECK_NEAR(ekf_get_pn(&e), truth[0], 0.5);
    CHECK_NEAR(ekf_get_pe(&e), truth[1], 0.5);
    CHECK_NEAR(ekf_get_pd(&e), truth[2], 0.5);
    /* Velocity should settle near zero for a static target */
    CHECK_NEAR(ekf_get_vn(&e), 0.0f, 0.2);
    /* Innovation shrinks once converged */
    CHECK(e.last_innov_norm < 1.0f);
}

TEST(test_update_learns_constant_velocity_without_imu) {
    /* Feed zero acceleration but a truth moving at 2 m/s north. The filter
     * must infer the velocity purely from GPS position updates (this is what
     * carries the beacon through GPS dropouts during descent). */
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);

    float a[3] = {0.0f, 0.0f, 0.0f};
    for (int s = 1; s <= 60; s++) {
        for (int i = 0; i < 100; i++) {
            ekf_predict(&e, a, 0.01f);
        }
        float z[3] = {2.0f * (float)s, 0.0f, 0.0f};
        CHECK(ekf_update_gps_position(&e, z) == 1);
    }

    CHECK_NEAR(ekf_get_vn(&e), 2.0f, 0.3);
    CHECK_NEAR(ekf_get_pn(&e), 120.0f, 2.0);
}

TEST(test_covariance_stays_symmetric_positive) {
    Ekf_t e;
    make_filter(&e);
    ekf_set_position(&e, 0.0f, 0.0f, 0.0f);

    float a[3] = {0.5f, -0.3f, 9.0f};
    for (int s = 0; s < 20; s++) {
        for (int i = 0; i < 100; i++) {
            ekf_predict(&e, a, 0.01f);
        }
        float z[3] = {(float)s, (float)-s, (float)s * 0.5f};
        ekf_update_gps_position(&e, z);
    }

    for (int i = 0; i < 6; i++) {
        CHECK(e.P[i][i] > 0.0f);                       /* positive variance */
        for (int j = 0; j < 6; j++) {
            CHECK_NEAR(e.P[i][j], e.P[j][i], 1e-5);    /* symmetric         */
        }
    }
}

/* ------------------------------------------------------------------ */

int main(void) {
    run_test_init_state();
    run_test_update_rejected_before_anchor();
    run_test_set_position_anchors_and_zeroes_velocity();
    run_test_predict_rejects_absurd_dt();
    run_test_predict_constant_acceleration_kinematics();
    run_test_predict_grows_uncertainty();
    run_test_update_converges_to_static_truth();
    run_test_covariance_stays_symmetric_positive();
    run_test_update_learns_constant_velocity_without_imu();

    return TEST_SUMMARY();
}
