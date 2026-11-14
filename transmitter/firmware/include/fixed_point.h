/**
 * @file fixed_point.h
 * @brief Saturating float -> fixed-point conversions for wire and flash
 *        records, plus the unit constants they share.
 *
 * Every telemetry field is a fixed-width integer. A bare
 * (int16_t)lroundf(x) wraps silently once x leaves the type's range (a
 * 6 rad/s spin in centi-deg/s read back as -31159) and is undefined for
 * NaN/Inf. These saturate instead: NaN -> 0, out of range -> the type's
 * limit, so a pinned value reads as "at least this much".
 *
 * Header-only and usable from both C and C++.
 */
#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>
#include <math.h>

#define MS2_PER_G    9.80665f      /* standard gravity: m/s^2 per g */
#define DEG_PER_RAD  57.2957795f

/** Round to nearest, saturating at INT16_MIN/INT16_MAX; NaN -> 0. */
static inline int16_t sat_i16(float v)
{
    if (!(v == v)) return 0;                    /* NaN */
    if (v >= 32767.0f) return INT16_MAX;
    if (v <= -32768.0f) return INT16_MIN;
    return (int16_t)lroundf(v);
}

/** Round to nearest, saturating at INT32_MIN/INT32_MAX; NaN -> 0. */
static inline int32_t sat_i32(double v)
{
    if (!(v == v)) return 0;                    /* NaN */
    if (v >= 2147483647.0) return INT32_MAX;
    if (v <= -2147483648.0) return INT32_MIN;
    return (int32_t)lround(v);
}

#endif /* FIXED_POINT_H */
