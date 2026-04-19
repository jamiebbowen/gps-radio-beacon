/**
 * @file math_utils.c
 * @brief Math utility functions implementation for GPS calculations
 */

#include "math_utils.h"
#include <math.h>

/**
 * @brief Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Distance in meters
 */
float calculate_distance(float lat1, float lon1, float lat2, float lon2)
{
  /* Convert degrees to radians */
  float lat1_rad = lat1 * M_PI / 180.0f;
  float lon1_rad = lon1 * M_PI / 180.0f;
  float lat2_rad = lat2 * M_PI / 180.0f;
  float lon2_rad = lon2 * M_PI / 180.0f;
  
  /* Haversine formula */
  float dlon = lon2_rad - lon1_rad;
  float dlat = lat2_rad - lat1_rad;
  float a = sinf(dlat/2) * sinf(dlat/2) + cosf(lat1_rad) * cosf(lat2_rad) * sinf(dlon/2) * sinf(dlon/2);
  float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
  float distance = EARTH_RADIUS_METERS * c;
  
  return distance;
}

/**
 * @brief Calculate bearing between two GPS coordinates
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Bearing in degrees (0-359.9)
 */
float calculate_bearing(float lat1, float lon1, float lat2, float lon2)
{
  /* Convert degrees to radians */
  float lat1_rad = lat1 * M_PI / 180.0f;
  float lon1_rad = lon1 * M_PI / 180.0f;
  float lat2_rad = lat2 * M_PI / 180.0f;
  float lon2_rad = lon2 * M_PI / 180.0f;
  
  /* Calculate bearing */
  float y = sinf(lon2_rad - lon1_rad) * cosf(lat2_rad);
  float x = cosf(lat1_rad) * sinf(lat2_rad) - sinf(lat1_rad) * cosf(lat2_rad) * cosf(lon2_rad - lon1_rad);
  float bearing_rad = atan2f(y, x);
  float bearing_deg = bearing_rad * 180.0f / M_PI;
  
  /* Normalize to 0-360 degrees */
  return normalize_angle(bearing_deg);
}

/**
 * @brief Normalize angle to 0-359.9 degrees
 * @param angle Angle in degrees
 * @return Normalized angle in degrees (0-359.9)
 */
float normalize_angle(float angle)
{
  /* Normalize angle to 0-360 degrees */
  while (angle < 0.0f) {
    angle += 360.0f;
  }
  while (angle >= 360.0f) {
    angle -= 360.0f;
  }
  
  return angle;
}
