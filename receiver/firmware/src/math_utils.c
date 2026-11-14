/**
 * @file math_utils.c
 * @brief Math utility functions implementation for GPS calculations
 */

#include "math_utils.h"
#include <string.h>
#include <math.h>

/**
 * @brief Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1 Latitude of first point in decimal degrees
 * @param lon1 Longitude of first point in decimal degrees
 * @param lat2 Latitude of second point in decimal degrees
 * @param lon2 Longitude of second point in decimal degrees
 * @return Distance in meters
 */
int saved_beacon_implausibly_far(float saved_lat, float saved_lon,
                                 float local_lat, float local_lon)
{
  return calculate_distance(saved_lat, saved_lon, local_lat, local_lon)
         > SAVED_BEACON_MAX_PLAUSIBLE_M;
}

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
  /* Float rounding can push a slightly past 1 when both points are nearly
   * identical; sqrtf(1-a) would then return NaN and poison the distance. */
  if (a > 1.0f) a = 1.0f;
  if (a < 0.0f) a = 0.0f;
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

static int nav_point_ok(float lat, float lon)
{
  /* (0, 0) is the "no position yet" placeholder; the range checks also
   * reject NaN. */
  return lat != 0.0f && lon != 0.0f &&
         lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f;
}

void Nav_Compute(float base_lat, float base_lon, uint8_t base_live,
                 float tgt_lat, float tgt_lon, float heading_deg,
                 Nav_Solution_t *out)
{
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  out->base_stale = base_live ? 0 : 1;
  if (!nav_point_ok(base_lat, base_lon) || !nav_point_ok(tgt_lat, tgt_lon)) {
    return;
  }
  out->distance_km  = calculate_distance(base_lat, base_lon, tgt_lat, tgt_lon) / 1000.0f;
  out->bearing_deg  = calculate_bearing(base_lat, base_lon, tgt_lat, tgt_lon);
  out->relative_deg = normalize_angle(out->bearing_deg - heading_deg);
  out->valid = 1;
}
