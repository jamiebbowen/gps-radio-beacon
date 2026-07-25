#ifndef EKF_H
#define EKF_H

/*
 * 6-state Extended Kalman Filter for GPS/IMU fusion.
 *
 *   State x = [p_n p_e p_d v_n v_e v_d]^T
 *   Frame   = local-tangent NED, origin set by nav layer on first GPS fix
 *   Input   = earth-frame acceleration in m/s^2 (gravity already removed)
 *   Measurement = GPS position in NED meters (velocity not measured)
 *
 * Orientation is taken from the BNO085's fused rotation vector upstream;
 * this filter only touches position and velocity.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[6];      /* state: p_n p_e p_d v_n v_e v_d */
    float P[6][6];   /* covariance                     */

    /* cached tuning */
    float sigma_a;   /* process accel noise (m/s^2)    */
    float sigma_gh;  /* GPS horizontal noise (m)       */
    float sigma_gv;  /* GPS vertical noise (m)         */

    /* last innovation, for logging / health checks */
    float last_innov[3];
    float last_innov_norm;
    uint8_t initialized;
} Ekf_t;

/* Reset state to zero and covariance to initial diagonal. */
void ekf_init(Ekf_t *e,
              float sigma_a,
              float sigma_gps_h,
              float sigma_gps_v,
              float init_pos_var,
              float init_vel_var);

/* Hard-set state to the supplied position (meters NED) and zero velocity.
 * Typically called on the first GPS fix after the tangent plane is anchored. */
void ekf_set_position(Ekf_t *e, float pn, float pe, float pd);

/* Predict step.  a_earth is gravity-removed earth-frame accel (m/s^2).
 * dt is seconds since last predict; must be > 0. */
void ekf_predict(Ekf_t *e, const float a_earth[3], float dt);

/* Measurement update from a GPS position (meters NED).
 * Returns 1 if the update was applied, 0 if rejected for sanity reasons. */
uint8_t ekf_update_gps_position(Ekf_t *e, const float z_ned[3]);

/* Helpers */
static inline float ekf_get_pn(const Ekf_t *e) { return e->x[0]; }
static inline float ekf_get_pe(const Ekf_t *e) { return e->x[1]; }
static inline float ekf_get_pd(const Ekf_t *e) { return e->x[2]; }
static inline float ekf_get_vn(const Ekf_t *e) { return e->x[3]; }
static inline float ekf_get_ve(const Ekf_t *e) { return e->x[4]; }
static inline float ekf_get_vd(const Ekf_t *e) { return e->x[5]; }

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */
