#include "include/flight_events.h"
#include <math.h>
#include <string.h>

/* Detector state. "Hold" timers debounce a level condition: 0 = idle,
 * else the now_ms at which the condition first went true; the condition
 * going false resets the timer. One-shot events (apogee/drogue/main) latch
 * for the whole flight; the anomaly set is a level with entry edge +
 * all-clear hysteresis. */
static struct {
    uint8_t  apogee_done;
    uint8_t  drogue_done;
    uint8_t  main_done;
    uint32_t apogee_t_ms;
    float    drogue_baseline_ms;    /* v_d at drogue declaration          */
    float    max_vd_since_apogee;   /* fallback baseline if no drogue     */

    uint32_t apogee_since;
    uint32_t drogue_since;
    uint32_t main_since;
    uint32_t ballistic_since;
    uint32_t tumble_since;
    uint32_t sensor_since;

    uint8_t  anomaly_active;
    uint32_t all_clear_since;       /* 0 = a condition is currently held  */

    /* Pending (detected, not yet handed to the caller) one-shots.
     * Anomaly entry is one pending slot that re-snapshots the active code
     * at drain time so a same-pass escalation reports the worst. */
    uint8_t  pend_apogee, pend_drogue, pend_main, pend_anomaly;
    int16_t  pend_apogee_val, pend_drogue_val, pend_main_val;
    uint8_t  anom_ballistic, anom_tumble, anom_sensor, anom_gps;
} s;

static int held(uint32_t *timer, int cond, uint32_t now_ms, uint32_t hold_ms)
{
    if (!cond) { *timer = 0; return 0; }
    if (*timer == 0) { *timer = now_ms; return 0; }
    return (uint32_t)(now_ms - *timer) >= hold_ms;
}

static int16_t cms(float v) { return (int16_t)lroundf(v * 100.0f); }

static uint8_t current_anomaly_code(void)
{
    if (s.anom_ballistic) return FLIGHT_EVENT_ANOM_BALLISTIC;
    if (s.anom_tumble)    return FLIGHT_EVENT_ANOM_TUMBLE;
    if (s.anom_sensor)    return FLIGHT_EVENT_ANOM_SENSOR_LOSS;
    if (s.anom_gps)       return FLIGHT_EVENT_ANOM_GPS_OUTAGE;
    return FLIGHT_EVENT_NONE;
}

void flight_events_init(void)
{
    memset(&s, 0, sizeof(s));
}

uint8_t flight_events_feed(const flight_events_input_t *in,
                           uint32_t now_ms, int16_t *value_out)
{
    if (value_out) *value_out = 0;

    /* ---- anomaly level conditions (gyro/sensor/GPS paths work even with
     * an un-anchored EKF; the kinematic ones require nav_valid) --------- */
    s.anom_ballistic = held(&s.ballistic_since,
        in->airborne && in->nav_valid && s.apogee_done &&
        (now_ms - s.apogee_t_ms) >= (uint32_t)ANOM_BALLISTIC_GRACE_S * 1000UL &&
        in->v_d_ms >= ANOM_BALLISTIC_VD_MS,
        now_ms, ANOM_BALLISTIC_HOLD_MS);

    s.anom_tumble = held(&s.tumble_since,
        in->airborne && in->gyro_mag_rads >= ANOM_TUMBLE_RADS,
        now_ms, ANOM_TUMBLE_HOLD_MS);

    s.anom_sensor = held(&s.sensor_since,
        in->airborne && in->imu_degraded,
        now_ms, ANOM_SENSOR_HOLD_MS);

    /* fix_age is already a level measured by the GPS layer; no extra hold */
    s.anom_gps = in->airborne && in->gps_fix_age_ms >= ANOM_GPS_OUTAGE_MS;

    uint8_t any_anom = s.anom_ballistic || s.anom_tumble ||
                       s.anom_sensor || s.anom_gps;
    if (any_anom) {
        s.all_clear_since = 0;
        if (!s.anomaly_active) {
            s.anomaly_active = 1;
            s.pend_anomaly = 1;      /* entry edge -> radio event */
        }
    } else if (s.anomaly_active) {
        if (s.all_clear_since == 0) {
            s.all_clear_since = now_ms;
        } else if ((uint32_t)(now_ms - s.all_clear_since) >= ANOM_CLEAR_MS) {
            s.anomaly_active = 0;    /* silent exit; repeats just stop */
            s.all_clear_since = 0;
        }
    }

    /* ---- life-cycle one-shots (kinematic: require an anchored filter) - */
    if (in->airborne && in->nav_valid) {
        if (s.apogee_done && in->v_d_ms > s.max_vd_since_apogee) {
            s.max_vd_since_apogee = in->v_d_ms;
        }

        if (!s.apogee_done &&
            held(&s.apogee_since,
                 in->t_since_launch_s >= APOGEE_MIN_T_SINCE_LAUNCH_S &&
                 in->v_d_ms >= APOGEE_VD_MIN_MS,
                 now_ms, APOGEE_HOLD_MS)) {
            s.apogee_done = 1;
            s.apogee_t_ms = now_ms;
            s.pend_apogee = 1;
            float alt = in->alt_m;
            if (alt > 32767.0f) alt = 32767.0f;
            if (alt < -32768.0f) alt = -32768.0f;
            s.pend_apogee_val = (int16_t)lroundf(alt);
            s.max_vd_since_apogee = in->v_d_ms;
        }

        if (s.apogee_done && !s.drogue_done &&
            held(&s.drogue_since,
                 in->v_d_ms >= DROGUE_BAND_MIN_MS &&
                 in->v_d_ms <= DROGUE_BAND_MAX_MS,
                 now_ms, DROGUE_HOLD_MS)) {
            s.drogue_done = 1;
            s.drogue_baseline_ms = in->v_d_ms;
            s.pend_drogue = 1;
            s.pend_drogue_val = cms(in->v_d_ms);
        }

        if (s.apogee_done && !s.main_done) {
            float baseline = s.drogue_done ? s.drogue_baseline_ms
                                           : s.max_vd_since_apogee;
            float step_max = MAIN_STEP_RATIO * baseline;
            if (step_max > MAIN_BAND_MAX_MS) step_max = MAIN_BAND_MAX_MS;
            if (held(&s.main_since,
                     baseline >= MAIN_MIN_BASELINE_MS &&
                     in->v_d_ms >= 0.0f && in->v_d_ms <= step_max,
                     now_ms, MAIN_HOLD_MS)) {
                s.main_done = 1;
                s.pend_main = 1;
                s.pend_main_val = cms(in->v_d_ms);
            }
        }
    } else {
        s.apogee_since = s.drogue_since = s.main_since = 0;
    }

    /* ---- drain one pending edge, worst first -------------------------- */
    if (s.pend_anomaly) {
        s.pend_anomaly = 0;
        if (value_out) *value_out = cms(in->v_d_ms);
        return current_anomaly_code();
    }
    if (s.pend_apogee) {
        s.pend_apogee = 0;
        if (value_out) *value_out = s.pend_apogee_val;
        return FLIGHT_EVENT_APOGEE;
    }
    if (s.pend_drogue) {
        s.pend_drogue = 0;
        if (value_out) *value_out = s.pend_drogue_val;
        return FLIGHT_EVENT_DROGUE;
    }
    if (s.pend_main) {
        s.pend_main = 0;
        if (value_out) *value_out = s.pend_main_val;
        return FLIGHT_EVENT_MAIN;
    }
    return FLIGHT_EVENT_NONE;
}

uint8_t flight_events_anomaly_active(void)
{
    return s.anomaly_active;
}

uint8_t flight_events_anomaly_code(void)
{
    return s.anomaly_active ? current_anomaly_code() : FLIGHT_EVENT_NONE;
}
