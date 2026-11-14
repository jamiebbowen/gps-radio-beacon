/*
 * nav.cpp - GPS + IMU fusion glue layer.
 *
 * Maintains a local-tangent NED frame anchored at the first valid GPS fix,
 * drives the 6-state EKF in ekf.cpp with BNO085 rotation+accel, and exposes
 * a fused (lat, lon, alt, v_ned) snapshot to the beacon TX path.
 *
 * Frame conventions
 * -----------------
 *   Body frame: BNO085 native (right-handed, X forward, Y left, Z up per
 *       Adafruit_BNO08x defaults - actual axis mapping depends on mounting).
 *       Linear acceleration from SH2 has gravity already removed.
 *   Earth frame: local NED anchored at first fix.  n/e are flat-earth
 *       linearizations about the anchor; d is positive downward.  Valid to
 *       well under 0.1% error within ~100 km of the anchor.
 *
 * Approximations
 * --------------
 *   - Flat earth around the anchor:
 *       n_m = (lat  - lat0) * 111320.0
 *       e_m = (lon  - lon0) * 111320.0 * cos(lat0)
 *       d_m = -(alt - alt0)
 *     Inverses used when exporting fused position.
 *   - We assume the BNO085 returns a body->earth quaternion with "earth" =
 *     ENU (X east, Y north, Z up) per its datasheet.  We rotate accel to ENU
 *     then swap into NED:  a_ned = (a_enu.y, a_enu.x, -a_enu.z).
 */

#include "include/nav.h"
#include "include/ekf.h"
#include "include/config.h"
#include "include/launch_detect.h"   /* imu_* accessors */
#include <Arduino.h>
#include <math.h>
#include <string.h>

/* --------- module state -------------------------------------------------- */

static Ekf_t    s_ekf;
static bool     s_anchored = false;
static double   s_lat0 = 0.0;      /* anchor latitude  (deg) - double:  */
static double   s_lon0 = 0.0;      /* anchor longitude (deg) float32 is ~0.5 m */
static float    s_alt0 = 0.0f;     /* anchor altitude  (m)   */
static float    s_m_per_deg_lat = 111320.0f;
static float    s_m_per_deg_lon = 111320.0f;

static uint32_t s_last_gps_ms      = 0;
static uint32_t s_last_predict_ms  = 0;
static bool     s_have_any_predict = false;
static uint32_t s_gps_rejects      = 0;

/* Anchor confirmation: the first fix only becomes a candidate; the tangent
 * plane anchors on a second fix within NAV_ANCHOR_CONFIRM_M of it. */
static bool     s_anchor_pending   = false;
static double   s_cand_lat = 0.0, s_cand_lon = 0.0;
static float    s_cand_alt = 0.0f;
static uint32_t s_cand_ms = 0;

/* --------- helpers ------------------------------------------------------- */

static inline void anchor_set(double lat, double lon, float alt) {
    s_lat0 = lat;
    s_lon0 = lon;
    s_alt0 = alt;
    s_m_per_deg_lat = 111320.0f;
    s_m_per_deg_lon = (float)(111320.0 * cos(lat * M_PI / 180.0));
    s_anchored = true;
}

static inline void latlon_to_ned(double lat, double lon, float alt,
                                 float *n, float *e, float *d) {
    *n = (float)((lat - s_lat0) * s_m_per_deg_lat);
    *e = (float)((lon - s_lon0) * s_m_per_deg_lon);
    *d = -(alt - s_alt0);
}

static inline void ned_to_latlon(float n, float e, float d,
                                 double *lat, double *lon, float *alt) {
    *lat = s_lat0 + (double)n / s_m_per_deg_lat;
    *lon = s_lon0 + (double)e / s_m_per_deg_lon;
    *alt = s_alt0 - d;
}

/*
 * Rotate a body-frame vector by the BNO085 rotation quaternion.
 *
 * Standard passive->active rotation: v_earth = q * v_body * q*.  For a unit
 * quaternion this expands to:
 *   v' = v + 2 * r x (r x v + w v)
 * where r = (x,y,z).  Cheaper than building the 3x3 matrix explicitly.
 */
static void rotate_by_quat(float w, float x, float y, float z,
                           const float v[3], float out[3]) {
    float tx = 2.0f * (y * v[2] - z * v[1]);
    float ty = 2.0f * (z * v[0] - x * v[2]);
    float tz = 2.0f * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

/* --------- API ----------------------------------------------------------- */

void nav_init(void) {
    ekf_init(&s_ekf,
             EKF_SIGMA_ACCEL_MS2,
             EKF_SIGMA_GPS_HORIZ_M,
             EKF_SIGMA_GPS_VERT_M,
             EKF_INIT_POS_VAR_M2,
             EKF_INIT_VEL_VAR_MS2);
    s_anchored         = false;
    s_have_any_predict = false;
    s_last_gps_ms      = 0;
    s_last_predict_ms  = 0;
    s_gps_rejects      = 0;
    s_anchor_pending   = false;
}

/* A blocking SF10 transmit holds the loop ~1.1 s, and ekf_predict rejects
 * any single step over 1 s - so long gaps are integrated in sub-steps
 * instead of being dropped. The latest acceleration is held for at most
 * NAV_ACCEL_HOLD_S of a gap (past that it says nothing about the airframe);
 * the rest coasts on velocity. NAV_MAX_GAP_S bounds the work after an
 * abnormal stall. */
#define NAV_PREDICT_SUBSTEP_S  0.25f
#define NAV_ACCEL_HOLD_S       1.5f
#define NAV_MAX_GAP_S          30.0f

void nav_predict(void) {
    uint32_t now = millis();
    if (!s_have_any_predict) {
        s_last_predict_ms  = now;
        s_have_any_predict = true;
        return;
    }
    uint32_t dt_ms = now - s_last_predict_ms;
    if (dt_ms == 0) return;
    float dt = (float)dt_ms * 0.001f;
    if (dt > NAV_MAX_GAP_S) dt = NAV_MAX_GAP_S;
    s_last_predict_ms = now;

    if (!s_anchored) return;  /* can't predict meaningfully without anchor */

    /* Fetch latest IMU data.  If no rotation vector yet, integrate zero
     * accel (filter just grows covariance via Q).  If no accel event yet,
     * treat accel as zero. */
    float accel_body[3] = {0, 0, 0};
    float a_earth_ned[3] = {0, 0, 0};

    if (imu_has_quaternion()) {
        float w, qx, qy, qz;
        imu_get_quaternion(&w, &qx, &qy, &qz);
        imu_get_linear_accel_body(&accel_body[0], &accel_body[1], &accel_body[2]);

        /* rotate_by_quat's output scales with |q|^2, and the BNO085 can
         * report a not-yet-unit quaternion while fusion converges or a
         * sensor reset recovers: a saturated sample would amplify accel
         * ~4x, a near-zero one erase it. Normalize sane samples; a
         * degenerate quaternion can't orient the body at all, so that
         * predict step coasts on zero accel (same as the no-quat path). */
        float n2 = w*w + qx*qx + qy*qy + qz*qz;
        if (isfinite(n2) && n2 > 0.25f && n2 < 4.0f) {
            float inv = 1.0f / sqrtf(n2);
            float a_enu[3];
            rotate_by_quat(w*inv, qx*inv, qy*inv, qz*inv, accel_body, a_enu);

            /* ENU -> NED:  n = north = a_enu.y
             *              e = east  = a_enu.x
             *              d = down  = -a_enu.z (up -> down-positive) */
            a_earth_ned[0] =  a_enu[1];
            a_earth_ned[1] =  a_enu[0];
            a_earth_ned[2] = -a_enu[2];
        }
    }

    static const float coast[3] = {0, 0, 0};
    float done = 0.0f;
    while (done < dt) {
        float h = dt - done;
        if (h > NAV_PREDICT_SUBSTEP_S) h = NAV_PREDICT_SUBSTEP_S;
        ekf_predict(&s_ekf, (done < NAV_ACCEL_HOLD_S) ? a_earth_ned : coast, h);
        done += h;
    }
}

void nav_update_from_gps(double lat_deg, double lon_deg, float alt_m,
                         uint8_t sats, uint8_t fix_quality) {
    if (fix_quality < 1) return;
    if (sats < 4)        return;
    /* NaN/Inf guard: atof() parses "nan"/"inf" field text literally, and
     * one NaN into ekf_update_gps_position poisons the filter state
     * permanently (x += K*NaN propagates forever). Boundary-check any
     * upstream garbage here, same contract as the sats/fix gates. */
    if (!isfinite(lat_deg) || !isfinite(lon_deg) || !isfinite(alt_m)) return;
    /* Polar bound: the tangent plane divides by 111320*cos(lat); past 85deg
     * that factor collapses toward zero and every metre offset explodes.
     * |lat|>85 is format-valid NMEA but physically impossible here, so a
     * glitch fix claiming it must not anchor (or re-anchor) the frame. */
    if (lat_deg > 85.0f || lat_deg < -85.0f) return;

    if (!s_anchored) {
        /* Anchor on two CONSECUTIVE fixes that agree: a single glitch fix
         * (multipath at power-on, stale cold-start position) used to anchor
         * the tangent plane wrong, after which the innovation gate would
         * reject every honest fix for tens of seconds. A bad candidate is
         * simply replaced by the next fix, so one glitch costs one fix. */
        uint32_t now = millis();
        if (s_anchor_pending && (now - s_cand_ms) < 10000u) {
            float dn = (float)((lat_deg - s_cand_lat) * 111320.0);
            float de = (float)((lon_deg - s_cand_lon) * 111320.0
                     * cos(s_cand_lat * M_PI / 180.0));
            float dd = alt_m - s_cand_alt;
            float dist2 = dn * dn + de * de + dd * dd;
            if (dist2 <= NAV_ANCHOR_CONFIRM_M * NAV_ANCHOR_CONFIRM_M) {
                anchor_set(lat_deg, lon_deg, alt_m);
                ekf_set_position(&s_ekf, 0.0f, 0.0f, 0.0f);
                s_anchor_pending = false;
                s_last_gps_ms = now;
                return;
            }
        }
        /* No candidate yet, stale candidate, or disagreement: (re)seed. */
        s_cand_lat = lat_deg; s_cand_lon = lon_deg; s_cand_alt = alt_m;
        s_cand_ms = now;
        s_anchor_pending = true;
        return;
    }

    float z[3];
    latlon_to_ned(lat_deg, lon_deg, alt_m, &z[0], &z[1], &z[2]);
    uint32_t now = millis();

    /* Rescue a starved/diverged filter. Observed 2026-09-11: a ~5 min GPS
     * hole mid-drive left ~40 m/s of phantom velocity in the coast state;
     * the innovation gate then rejected every honest fix forever and the
     * fused stream trailed off 22 km away. Past NAV_RESCUE_NOFIX_MS the
     * gate itself becomes suspect, so a fix is only let through in pairs
     * that are motion-consistent (<= NAV_RESCUE_MAX_STEP_M, <= 5 s apart)
     * - which a lone stale hot-start glitch also can't fake. */
    bool starved = (now - s_last_gps_ms) > NAV_RESCUE_NOFIX_MS;
    if (starved) {
        static double   prev_lat, prev_lon;
        static float    prev_alt;
        static uint32_t prev_ms = 0;
        bool consistent = false;
        if (prev_ms != 0 && (now - prev_ms) <= 5000u) {
            float dn = (float)((lat_deg - prev_lat) * 111320.0);
            float de = (float)((lon_deg - prev_lon) * 111320.0
                     * cos(prev_lat * M_PI / 180.0));
            float dd = alt_m - prev_alt;
            consistent = (dn*dn + de*de + dd*dd)
                       <= NAV_RESCUE_MAX_STEP_M * NAV_RESCUE_MAX_STEP_M;
        }
        prev_lat = lat_deg; prev_lon = lon_deg; prev_alt = alt_m;
        prev_ms  = now;
        if (!consistent) return;   /* wait for the confirming fix */
    }

    /* Re-anchor far-from-origin fixes while starved. Unstarved flight legs
     * past NAV_REANCHOR_M of the anchor are completely normal, which is why
     * this must stay gated on starved. A re-anchor hard-resets the filter
     * (position = fix, velocity = 0, fresh small covariances) - also exactly
     * what flushes a diverged coast state. */
    float anchor_r2 = z[0]*z[0] + z[1]*z[1];
    if (starved && anchor_r2 > NAV_REANCHOR_M*NAV_REANCHOR_M) {
        anchor_set(lat_deg, lon_deg, alt_m);
        ekf_init(&s_ekf, EKF_SIGMA_ACCEL_MS2,
                 EKF_SIGMA_GPS_HORIZ_M, EKF_SIGMA_GPS_VERT_M,
                 EKF_INIT_POS_VAR_M2, EKF_INIT_VEL_VAR_MS2);
        Serial.println(F("[Nav] RESCUE: starved + far - anchor moved"));
        latlon_to_ned(lat_deg, lon_deg, alt_m, &z[0], &z[1], &z[2]);
    }
    if (starved) {
        /* Snap - don't blend: the coast state we are rescuing from is
         * exactly the untrustworthy part. Zero velocity, fresh small
         * covariances, position = the confirmed fix. */
        ekf_set_position(&s_ekf, z[0], z[1], z[2]);
        s_last_gps_ms = now;
        Serial.println(F("[Nav] RESCUE: fix accepted, state snapped"));
        return;
    }

    /* Innovation gate: reject multipath jumps / stale-position glitches.
     * Armed only while the IMU is healthy - with live rotation+accel the
     * predict step tracks real dynamics, so a big normalized innovation
     * means the FIX is wrong. With a dead IMU the filter coasts at constant
     * velocity and honest boost-phase motion produces huge innovations;
     * gating there would reject good fixes exactly when GPS is the only
     * position source, so fail open. A rejected fix does not refresh
     * s_last_gps_ms, so age/dead-reckoning flags reflect only accepted
     * fixes. Never reached in starved/rescue mode (handled above). */
    {
        bool imu_ok = imu_has_quaternion()
                   && (imu_last_accel_ms() > 0)
                   && ((now - imu_last_accel_ms()) < 500u);
        if (imu_ok) {
            float R[3] = {
                s_ekf.sigma_gh * s_ekf.sigma_gh,
                s_ekf.sigma_gh * s_ekf.sigma_gh,
                s_ekf.sigma_gv * s_ekf.sigma_gv,
            };
            float nis = 0.0f, norm2 = 0.0f;
            for (int i = 0; i < 3; i++) {
                float S = s_ekf.P[i][i] + R[i];
                if (S <= 1e-6f) continue;
                float innov = z[i] - s_ekf.x[i];
                nis   += innov * innov / S;
                norm2 += innov * innov;
            }
            if (nis > NAV_NIS_REJECT && norm2 > NAV_GATE_MIN_M * NAV_GATE_MIN_M) {
                s_gps_rejects++;
#if IMU_FUSION_LOG_RESIDUALS
                Serial.print(F("[Nav] GPS REJECTED: nis="));
                Serial.print(nis, 1);
                Serial.print(F(" |innov|="));
                Serial.println(sqrtf(norm2), 1);
#endif
                return;
            }
        }
    }

    ekf_update_gps_position(&s_ekf, z);
    s_last_gps_ms = millis();

#if IMU_FUSION_LOG_RESIDUALS
    Serial.print(F("[Nav] GPS update: innov=("));
    Serial.print(s_ekf.last_innov[0], 2); Serial.print(F(","));
    Serial.print(s_ekf.last_innov[1], 2); Serial.print(F(","));
    Serial.print(s_ekf.last_innov[2], 2);
    Serial.print(F(")  |innov|=")); Serial.print(s_ekf.last_innov_norm, 2);
    Serial.print(F("  vNED=("));
    Serial.print(ekf_get_vn(&s_ekf), 2); Serial.print(F(","));
    Serial.print(ekf_get_ve(&s_ekf), 2); Serial.print(F(","));
    Serial.print(ekf_get_vd(&s_ekf), 2); Serial.println(F(")"));
#endif
}

void nav_get_fused(NavFused_t *out) {
    memset(out, 0, sizeof(*out));
    if (!s_anchored) {
        out->valid = false;
        return;
    }
    float n = ekf_get_pn(&s_ekf);
    float e = ekf_get_pe(&s_ekf);
    float d = ekf_get_pd(&s_ekf);
    ned_to_latlon(n, e, d, &out->lat_deg, &out->lon_deg, &out->alt_m);

    out->v_n = ekf_get_vn(&s_ekf);
    out->v_e = ekf_get_ve(&s_ekf);
    out->v_d = ekf_get_vd(&s_ekf);

    uint32_t now       = millis();
    uint32_t age_ms    = (s_last_gps_ms > 0) ? (now - s_last_gps_ms) : 0xFFFFFFFFu;
    out->gps_fresh     = (s_last_gps_ms > 0) && (age_ms < 1500u);
    out->dead_reckoning= (age_ms > (uint32_t)(NAV_DR_TIMEOUT_S * 1000.0f));

    uint32_t imu_age_ms = now - imu_last_accel_ms();
    out->imu_healthy   = (imu_last_accel_ms() > 0) && (imu_age_ms < 500u)
                      && imu_has_quaternion();

    /* FUSED_FLAG_SENSOR_DEGRADED: the BNO085 is the EKF's only inertial
     * source, so "degraded" = init failed at boot, the stream never
     * started, or it went silent past NAV_SENSOR_DEAD_MS (wedged I2C, or
     * a chip reset being recovered through the wasReset() re-enable - i.e.
     * "in retry"). A level, not a latch: it clears when the stream comes
     * back. Where imu_healthy answers "staleness right now" (500 ms),
     * this answers "the sensor is actually gone" so the RX preflight page
     * can tell a dead IMU apart from an ordinary fix gap. */
    out->sensor_degraded =
           !launch_detect_get_imu_status()
        || (now > NAV_SENSOR_DEAD_MS && imu_last_accel_ms() == 0)
        || (imu_last_accel_ms() > 0 && imu_age_ms > NAV_SENSOR_DEAD_MS);

    out->valid = true;
}

bool nav_is_valid(void) { return s_anchored; }

uint32_t nav_get_gps_rejects(void) { return s_gps_rejects; }
