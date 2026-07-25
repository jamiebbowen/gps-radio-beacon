#ifndef NAV_H
#define NAV_H

/*
 * Navigation / fusion layer.  Sits between sensors (GPS + IMU) and beacon TX.
 *
 *   Typical flow (main loop):
 *     nav_init()                  once at boot
 *     nav_predict()               every iteration, drives the EKF off the IMU
 *     nav_update_from_gps(...)    on each fresh GPS fix
 *     nav_get_fused(...)          when building a fused TX packet
 *
 * Internal frame is local-tangent NED anchored at the first valid GPS fix.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    lat_deg;      /* fused latitude  (deg)          */
    float    lon_deg;      /* fused longitude (deg)          */
    float    alt_m;        /* fused altitude  (m MSL)        */
    float    v_n;          /* velocity north  (m/s)          */
    float    v_e;          /* velocity east   (m/s)          */
    float    v_d;          /* velocity down   (m/s)          */
    uint8_t  age_ds;       /* deciseconds since last GPS fix, saturating */
    bool     gps_fresh;    /* true if GPS updated within NAV_DR_TIMEOUT_S */
    bool     dead_reckoning;
    bool     imu_healthy;
    bool     valid;        /* false until origin is anchored */
} NavFused_t;

void nav_init(void);

/* Propagate the EKF using whatever IMU data has accumulated.  Safe to call at
 * any rate; internal dt handling uses millis(). */
void nav_predict(void);

/* Feed a fresh GPS fix (decimal degrees, meters MSL).  sats/fix_quality are
 * passed through for sanity checks; fix_quality < 1 is ignored. */
void nav_update_from_gps(float lat_deg, float lon_deg, float alt_m,
                         uint8_t sats, uint8_t fix_quality);

/* Snapshot of the fused state. */
void nav_get_fused(NavFused_t *out);

/* True once a valid GPS fix has anchored the tangent plane. */
bool nav_is_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* NAV_H */
