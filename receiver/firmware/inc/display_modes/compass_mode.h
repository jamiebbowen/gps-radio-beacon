/**
 ******************************************************************************
 * @file           : compass_mode.h
 * @brief          : Compass display mode functions
 ******************************************************************************
 */

#ifndef COMPASS_MODE_H
#define COMPASS_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "compass.h"

/**
 * @brief Display compass heading and orientation data
 * @param compass_data Pointer to compass data structure
 * @param compass_heading Pointer to compass heading value
 * @retval None
 */
void DisplayMode_CompassHeading(Compass_Data *compass_data, float *compass_heading);
void DisplayMode_CompassVisual(Compass_Data *compass_data, float *compass_heading);

#ifdef __cplusplus
}
#endif

#endif /* COMPASS_MODE_H */
