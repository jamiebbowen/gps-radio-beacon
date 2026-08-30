/**
 ******************************************************************************
 * @file           : compass_mode.c
 * @brief          : Compass display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "display_modes/compass_mode.h"
#include "display.h"
#include "compass.h"

/**
 * @brief Helper function to update compass data and handle errors
 * @param compass_data Pointer to compass data structure
 * @param compass_heading Pointer to compass heading variable (can be NULL)
 * @return true if update was successful, false otherwise
 */
static bool UpdateCompassData(Compass_Data *compass_data, float *compass_heading)
{
  if (Compass_Update(compass_data) == COMPASS_OK) {
    /* Store compass heading for navigation calculations if pointer is provided */
    if (compass_heading != NULL) {
      *compass_heading = compass_data->heading;
    }
    return true;
  } else if (compass_data->heading_stale) {
    /* Comms down but we have a last-known heading: keep showing it
     * (flagged with * by the callers) instead of blanking the screen. */
    return true;
  } else {
    /* Display error if compass update failed */
    Display_DrawTextRowCol(1, 0, "BNO055 ERROR");
    /* Max 21 cols (128px / 6px font) - longer strings clip silently */
    Display_DrawTextRowCol(3, 0, "Compass no response");
    Display_DrawTextRowCol(5, 0, "Check connections");
    
    /* Set default heading if pointer is provided */
    if (compass_heading != NULL) {
      *compass_heading = 0.0f;
    }
    return false;
  }
}

/**
 * @brief Visual compass screen: compass rose on left, heading/pitch/roll on right.
 * @param compass_data Pointer to compass data structure
 * @param compass_heading Pointer to compass heading variable
 * @retval None
 */
void DisplayMode_CompassVisual(Compass_Data *compass_data, float *compass_heading)
{
  char buffer[16];

  if (!UpdateCompassData(compass_data, compass_heading)) return;

  float heading = compass_data->heading;
  const float D2R = 3.14159265f / 180.0f;

  /* --- Compass rose (left half: x=0..63, y=0..63) --- */
  int cx = 32, cy = 32, r = 20;

  Display_DrawCircle((uint8_t)cx, (uint8_t)cy, (uint8_t)r, 1);
  Display_DrawCircle((uint8_t)cx, (uint8_t)cy, 2, 1);

  /* Tick marks: long (5px) at cardinals, short (3px) at intercardinals */
  for (int d = 0; d < 360; d += 45) {
    float rad  = d * D2R;
    int tick   = (d % 90 == 0) ? 5 : 3;
    int x0 = cx + (int)((r - tick) * sinf(rad));
    int y0 = cy - (int)((r - tick) * cosf(rad));
    int x1 = cx + (int)(r * sinf(rad));
    int y1 = cy - (int)(r * cosf(rad));
    Display_DrawLine((uint8_t)x0, (uint8_t)y0, (uint8_t)x1, (uint8_t)y1, 1);
  }

  /* Cardinal labels */
  Display_DrawText((uint8_t)(cx - 2), 1,            "N");
  Display_DrawText((uint8_t)(cx - 2), (uint8_t)(cy + r + 2), "S");
  Display_DrawText((uint8_t)(cx + r + 2), (uint8_t)(cy - 4), "E");
  Display_DrawText((uint8_t)(cx - r - 7), (uint8_t)(cy - 4), "W");

  /* Heading needle — draw for live or stale-but-calibrated headings */
  if (compass_data->heading_valid || compass_data->heading_stale) {
    float h_rad   = heading * D2R;
    int tip_x  = cx + (int)(17.0f * sinf(h_rad));
    int tip_y  = cy - (int)(17.0f * cosf(h_rad));
    int tail_x = cx - (int)( 5.0f * sinf(h_rad));
    int tail_y = cy + (int)( 5.0f * cosf(h_rad));

    if (tip_x  <  0) tip_x  =  0;
    if (tip_x  > 63) tip_x  = 63;
    if (tip_y  <  0) tip_y  =  0;
    if (tip_y  > 63) tip_y  = 63;
    if (tail_x <  0) tail_x =  0;
    if (tail_x > 63) tail_x = 63;
    if (tail_y <  0) tail_y =  0;
    if (tail_y > 63) tail_y = 63;

    Display_DrawLine((uint8_t)tail_x, (uint8_t)tail_y,
                     (uint8_t)tip_x,  (uint8_t)tip_y, 1);
  } else if (!compass_data->orientation_valid) {
    /* Device axis near-vertical: heading geometrically undefined - this
     * is a HOLD-TILT problem, not a calibration problem */
    Display_DrawText((uint8_t)(cx - 9), (uint8_t)(cy - 4), "VERT");
  } else {
    /* Magnetometer not yet calibrated — heading unreliable */
    Display_DrawText((uint8_t)(cx - 6), (uint8_t)(cy - 4), "CAL");
  }

  /* --- Right half: text data (x=66) --- */
  if (compass_data->heading_valid) {
    snprintf(buffer, sizeof(buffer), "Hdg:%3.0fd", heading);
  } else if (compass_data->heading_stale) {
    snprintf(buffer, sizeof(buffer), "Hdg:%3.0fd*", heading); /* * = stale */
  } else if (!compass_data->orientation_valid) {
    snprintf(buffer, sizeof(buffer), "Hdg: VERT");
  } else {
    snprintf(buffer, sizeof(buffer), "Hdg: CAL?");
  }
  Display_DrawText(66, 0, buffer);

  snprintf(buffer, sizeof(buffer), "P:%5.1fd", compass_data->pitch);
  Display_DrawText(66, 10, buffer);

  snprintf(buffer, sizeof(buffer), "R:%5.1fd", compass_data->roll);
  Display_DrawText(66, 20, buffer);

  uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
  Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
  /* BNO055 cal statuses are each in [0,3]; mask so GCC can prove the
   * "S#G#A#M#" format fits in the 16-byte buffer. */
  snprintf(buffer, sizeof(buffer), "S%uG%uA%uM%u",
           sys_cal & 0x3u, gyro_cal & 0x3u, accel_cal & 0x3u, mag_cal & 0x3u);
  Display_DrawText(66, 32, buffer);

  int8_t temp;
  Compass_GetTemperature(&temp);
  snprintf(buffer, sizeof(buffer), "Tmp:%dC", temp);
  Display_DrawText(66, 44, buffer);
}

/**
 * @brief Display all BNO055 data on a single condensed screen:
 *        heading/pitch/roll, calibration, raw sensor data, system diagnostics.
 * @param compass_data Pointer to compass data structure
 * @param compass_heading Pointer to compass heading variable
 * @retval None
 */
void DisplayMode_CompassHeading(Compass_Data *compass_data, float *compass_heading)
{
  char buffer[32];

  if (!UpdateCompassData(compass_data, compass_heading)) return;

  /* Row 0: heading; trailing * = value predates a comm failure (stale) */
  sprintf(buffer, "BNO055 Hdg:%3.0fd%s", compass_data->heading,
          compass_data->heading_stale ? "*" : "");
  Display_DrawTextRowCol(0, 0, buffer);

  /* Row 1: pitch and roll */
  sprintf(buffer, "P:%5.1fd R:%5.1fd", compass_data->pitch, compass_data->roll);
  Display_DrawTextRowCol(1, 0, buffer);

  /* Row 2: calibration status. Trailing "R" = offsets were restored from SD
   * this boot, so the heading is trustworthy even while the BNO055's live
   * CALIB_STAT still reads 0 (it stays 0 until fresh motion re-verifies). */
  uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
  Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
  sprintf(buffer, "Cal S%d G%d A%d M%d%s", sys_cal, gyro_cal, accel_cal, mag_cal,
          Compass_WasCalRestored() ? " R" : "");
  Display_DrawTextRowCol(2, 0, buffer);

  /* Row 3-5: raw sensor values (int16_t counts). No colon after the axis
   * letter: three full-range int16s ("-32768") plus two commas is 20 chars,
   * so the 1-char prefix makes worst case exactly the 21-column width. */
  sprintf(buffer, "A%d,%d,%d",
          compass_data->accel_x, compass_data->accel_y, compass_data->accel_z);
  Display_DrawTextRowCol(3, 0, buffer);

  sprintf(buffer, "G%d,%d,%d",
          compass_data->gyro_x, compass_data->gyro_y, compass_data->gyro_z);
  Display_DrawTextRowCol(4, 0, buffer);

  sprintf(buffer, "M%d,%d,%d",
          compass_data->mag_x, compass_data->mag_y, compass_data->mag_z);
  Display_DrawTextRowCol(5, 0, buffer);

  /* Row 6: system status */
  uint8_t sys_stat, self_test, sys_err;
  Compass_GetSystemStatus(&sys_stat, &self_test, &sys_err);
  sprintf(buffer, "St:%02X Re:%02X Er:%02X", sys_stat, self_test, sys_err);
  Display_DrawTextRowCol(6, 0, buffer);

  /* Row 7: operation mode and temperature */
  uint8_t op_mode;
  int8_t temp;
  Compass_GetOperationMode(&op_mode);
  Compass_GetTemperature(&temp);
  sprintf(buffer, "Op:%02X  Tmp:%dC", op_mode, temp);
  Display_DrawTextRowCol(7, 0, buffer);
}
