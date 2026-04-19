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

#ifdef __cplusplus
}
#endif

#endif /* __MATH_UTILS_H */
