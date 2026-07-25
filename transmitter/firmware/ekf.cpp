/*
 * 6-state EKF for GPS/IMU fusion.  See ekf.h for state definition.
 *
 * Design notes
 * ------------
 *   Process model (decoupled per axis i in {n,e,d}):
 *       p_i <- p_i + v_i*dt + 0.5*a_i*dt^2
 *       v_i <- v_i + a_i*dt
 *
 *   So F is block-structured as [[I, dt*I], [0, I]] (6x6) and the three
 *   position/velocity pairs are independent during predict.  We exploit this
 *   to do the covariance update as three 2x2 operations instead of a 6x6
 *   matrix multiply.
 *
 *   Measurement model (GPS position only):
 *       z = H x + noise,   H = [I_3 | 0_3]
 *   Innovation covariance S = H P H^T + R is 3x3 (again block-diagonal along
 *   the axes) and closed-form invertible per axis since axes decouple under
 *   the diagonal R we use here.
 *
 * Numerics
 * --------
 *   Covariance is updated with the Joseph form to stay symmetric PSD even
 *   when the gain K is slightly off due to float rounding.  After each
 *   predict/update we explicitly symmetrize (P + P^T)/2.
 */

#include "include/ekf.h"
#include <string.h>
#include <math.h>

/* --------- small helpers ------------------------------------------------- */

static void mat6_symmetrize(float P[6][6]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = i + 1; j < 6; ++j) {
            float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = m;
        }
    }
}

/* --------- public API ---------------------------------------------------- */

void ekf_init(Ekf_t *e,
              float sigma_a,
              float sigma_gps_h,
              float sigma_gps_v,
              float init_pos_var,
              float init_vel_var) {
    memset(e, 0, sizeof(*e));
    e->sigma_a  = sigma_a;
    e->sigma_gh = sigma_gps_h;
    e->sigma_gv = sigma_gps_v;

    /* P = diag(init_pos_var * 3, init_vel_var * 3) */
    for (int i = 0; i < 3; ++i) e->P[i][i]         = init_pos_var;
    for (int i = 3; i < 6; ++i) e->P[i][i]         = init_vel_var;
    e->initialized = 0;  /* position not yet anchored */
}

void ekf_set_position(Ekf_t *e, float pn, float pe, float pd) {
    e->x[0] = pn; e->x[1] = pe; e->x[2] = pd;
    e->x[3] = 0.0f; e->x[4] = 0.0f; e->x[5] = 0.0f;
    /* Shrink initial uncertainty now that we have a real anchor. */
    float p0 = e->sigma_gh * e->sigma_gh;
    float pv = e->sigma_gv * e->sigma_gv;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) e->P[i][j] = 0.0f;
    e->P[0][0] = p0;
    e->P[1][1] = p0;
    e->P[2][2] = pv;
    e->P[3][3] = 4.0f;  /* ±2 m/s 1σ initial velocity uncertainty */
    e->P[4][4] = 4.0f;
    e->P[5][5] = 4.0f;
    e->initialized = 1;
}

void ekf_predict(Ekf_t *e, const float a_earth[3], float dt) {
    if (dt <= 0.0f || dt > 1.0f) return;  /* sanity: ignore absurd dt */

    /* State propagation, per axis (decoupled) */
    for (int i = 0; i < 3; ++i) {
        float p = e->x[i];
        float v = e->x[i + 3];
        float a = a_earth[i];
        e->x[i]     = p + v * dt + 0.5f * a * dt * dt;
        e->x[i + 3] = v + a * dt;
    }

    /*
     * Covariance propagation.  Because F is block-diagonal in the three
     * axes, each (p_i, v_i) pair has a 2x2 sub-covariance that evolves
     * independently.  Let
     *     A = [[1, dt],[0, 1]],  q = sigma_a^2
     *     Q_i = q * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
     * Then  P_i <- A P_i A^T + Q_i.
     *
     * We do this for each axis and leave the cross-axis blocks at zero.
     * Cross-axis correlation only arises through measurement updates when
     * R is non-diagonal (ours is diagonal) - so the block-diagonal structure
     * is preserved indefinitely and we can skip those entries.  The full 6x6
     * P array still exists for symmetry with the update step.
     */
    const float q = e->sigma_a * e->sigma_a;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float Q00 = q * dt4 * 0.25f;
    const float Q01 = q * dt3 * 0.5f;
    const float Q11 = q * dt2;

    for (int i = 0; i < 3; ++i) {
        /* 2x2 block indices: pp=i,i  pv=i,i+3  vp=i+3,i  vv=i+3,i+3 */
        float pp = e->P[i][i];
        float pv = e->P[i][i + 3];
        float vv = e->P[i + 3][i + 3];

        /* A P A^T where A = [[1, dt],[0,1]]:
         *   new_pp = pp + 2 dt pv + dt^2 vv
         *   new_pv = pv + dt vv
         *   new_vv = vv
         */
        float npp = pp + 2.0f * dt * pv + dt2 * vv + Q00;
        float npv = pv + dt * vv              + Q01;
        float nvv = vv                        + Q11;

        e->P[i][i]         = npp;
        e->P[i][i + 3]     = npv;
        e->P[i + 3][i]     = npv;
        e->P[i + 3][i + 3] = nvv;
    }
    mat6_symmetrize(e->P);
}

uint8_t ekf_update_gps_position(Ekf_t *e, const float z_ned[3]) {
    if (!e->initialized) return 0;

    /*
     * Measurement model: z = [I | 0] x + v,  R = diag(σh², σh², σv²)
     *
     * Because H picks the first 3 state components and P's cross-axis
     * position entries are zero (see predict notes), each axis can be
     * updated independently with a scalar Kalman gain.  That keeps us
     * out of any 3x3 matrix inverse.
     *
     *   innov_i = z_i - x_i
     *   S_i     = P_ii + R_ii
     *   K_p     = P_ii / S_i              (position gain)
     *   K_v     = P_iv / S_i              (coupling-into-velocity gain)
     *   x_i   <- x_i   + K_p * innov_i
     *   x_v   <- x_v   + K_v * innov_i
     *   Joseph-form 2x2 covariance update on the (p_i, v_i) block.
     */
    float R[3] = {
        e->sigma_gh * e->sigma_gh,
        e->sigma_gh * e->sigma_gh,
        e->sigma_gv * e->sigma_gv,
    };

    float innov_norm_sq = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float pp = e->P[i][i];
        float pv = e->P[i][i + 3];
        float vv = e->P[i + 3][i + 3];

        float S    = pp + R[i];
        if (S <= 1e-6f) continue;  /* pathological; skip */
        float Kp   = pp / S;
        float Kv   = pv / S;
        float innov = z_ned[i] - e->x[i];
        e->last_innov[i] = innov;
        innov_norm_sq   += innov * innov;

        e->x[i]     += Kp * innov;
        e->x[i + 3] += Kv * innov;

        /*
         * Joseph form for the 2x2 block:
         *   P' = (I - K H) P (I - K H)^T + K R K^T
         * with H = [1, 0] for this axis, so
         *   (I - K H) = [[1 - Kp, 0],[-Kv, 1]]
         */
        float a11 = 1.0f - Kp;  /* (I-KH)[0][0] */
        float a21 = -Kv;        /* (I-KH)[1][0] */

        /* M = (I-KH) * P_block */
        float M00 = a11 * pp;               /* = a11*pp + 0*pv */
        float M01 = a11 * pv;
        float M10 = a21 * pp + pv;          /* = a21*pp + 1*pv */
        float M11 = a21 * pv + vv;

        /* P' = M * (I-KH)^T */
        float npp = M00 * a11 + M01 * 0.0f;
        float npv = M00 * a21 + M01;        /* *1 */
        float nvv = M10 * a21 + M11;

        /* + K R K^T (rank-1 in this axis) */
        npp += Kp * R[i] * Kp;
        npv += Kp * R[i] * Kv;
        nvv += Kv * R[i] * Kv;

        e->P[i][i]         = npp;
        e->P[i][i + 3]     = npv;
        e->P[i + 3][i]     = npv;
        e->P[i + 3][i + 3] = nvv;
    }

    e->last_innov_norm = sqrtf(innov_norm_sq);
    mat6_symmetrize(e->P);
    return 1;
}
