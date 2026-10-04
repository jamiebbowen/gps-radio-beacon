#include "include/flight_maxima.h"
#include <math.h>
#include <string.h>

static float    s_max_alt_m = -32768.0f;
static uint8_t  s_alt_seen = 0;          /* first airborne nav-valid feed  */
static uint32_t s_t_maxalt_s = 0;
static float    s_max_speed_ms = 0.0f;
static float    s_max_accel_g = 0.0f;
static float    s_max_gyro_dps = 0.0f;

void flight_maxima_init(void)
{
    s_max_alt_m = -32768.0f;
    s_alt_seen = 0;
    s_t_maxalt_s = 0;
    s_max_speed_ms = 0.0f;
    s_max_accel_g = 0.0f;
    s_max_gyro_dps = 0.0f;
}

void flight_maxima_feed(const flight_maxima_input_t *in)
{
    if (!in->airborne) return;   /* pad handling/rail bumps are not flight */

    if (in->nav_valid) {
        if (!s_alt_seen || in->alt_m > s_max_alt_m) {
            s_alt_seen = 1;
            s_max_alt_m = in->alt_m;
            s_t_maxalt_s = in->uptime_s;
        }
        float speed = sqrtf(in->v_n_ms * in->v_n_ms +
                            in->v_e_ms * in->v_e_ms +
                            in->v_d_ms * in->v_d_ms);
        if (speed > s_max_speed_ms) s_max_speed_ms = speed;
    }

    /* Inertial channels stand on their own: at a dead-GPS boost they are
     * the only evidence there is. */
    float accel_g = fabsf(in->accel_ms2) / 9.80665f;
    if (accel_g > s_max_accel_g) s_max_accel_g = accel_g;

    float gyro_dps = in->gyro_mag_rads * 57.29578f;
    if (gyro_dps > s_max_gyro_dps) s_max_gyro_dps = gyro_dps;
}

void flight_maxima_get(MaximaPacket_t *out)
{
    memset(out, 0, sizeof(*out));
    /* No airborne nav-valid sample yet: report a legible 0 m / t=0 instead
     * of the -32768 initial sentinel. */
    float alt = s_alt_seen ? s_max_alt_m : 0.0f;
    if (alt > 32767.0f) alt = 32767.0f;
    if (alt < -32768.0f) alt = -32768.0f;
    out->max_alt_m     = (int16_t)lroundf(alt);
    out->t_maxalt_s    = (s_t_maxalt_s > 65535UL) ? 65535U : (uint16_t)s_t_maxalt_s;

    float speed_cms = s_max_speed_ms * 100.0f;
    out->max_speed_cms = (speed_cms > 65535.0f) ? 65535U : (uint16_t)lroundf(speed_cms);

    float accel_cg = s_max_accel_g * 100.0f;
    out->max_accel_cg = (accel_cg > 65535.0f) ? 65535U : (uint16_t)lroundf(accel_cg);

    /* 16-dps units: 0..4080 dps wraps the BNO085's own +-2000 dps range;
     * a single byte keeps the packet under its airtime step. */
    float gyro16 = s_max_gyro_dps / (float)MAXIMA_GYRO_DPS_SCALE;
    out->max_gyro_dps16 = (gyro16 > 255.0f) ? 255U : (uint8_t)lroundf(gyro16);
}
