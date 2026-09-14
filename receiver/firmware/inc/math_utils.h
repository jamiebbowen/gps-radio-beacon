/**
 * @file math_utils.h
 * @brief Math utility functions for GPS calculations
 */

#ifndef __MATH_UTILS_H
#define __MATH_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Earth radius in meters */
#define EARTH_RADIUS_METERS 6371000.0f

/**
 * @brief Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Distance in meters
 */
float calculate_distance(float lat1, float lon1, float lat2, float lon2);

/**
 * @brief Calculate bearing between two GPS coordinates
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Bearing in degrees (0-359.9)
 */
float calculate_bearing(float lat1, float lon1, float lat2, float lon2);

/**
 * @brief Normalize angle to 0-359.9 degrees
 * @param angle Angle in degrees
 * @return Normalized angle in degrees (0-359.9)
 */
float normalize_angle(float angle);

/**
 * @brief Predicate: is a restored last-beacon position implausibly far
 *        from the operator's own local fix? A rocket flight moves the
 *        beacon a few km at most; > ~100 km means the SD-card restore was
 *        from a different trip/state (cross-state launch). Used once at
 *        first local fix after a last-beacon bootload.
 * @return 1 if distance > SAVED_BEACON_MAX_PLAUSIBLE_M
 */
#define SAVED_BEACON_MAX_PLAUSIBLE_M  100000.0f
int saved_beacon_implausibly_far(float saved_lat, float saved_lon,
                                 float local_lat, float local_lon);

#ifdef __cplusplus
}
#endif

#endif /* __MATH_UTILS_H */
