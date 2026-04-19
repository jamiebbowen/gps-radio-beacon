/**
 ******************************************************************************
 * @file           : i2c_devices.h
 * @brief          : I2C device scan display functions
 ******************************************************************************
 */

#ifndef I2C_DEVICES_H
#define I2C_DEVICES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
 * @brief Display I2C device scan results
 * @param devices Array of detected I2C device addresses
 * @param count Number of detected devices
 * @retval None
 */
void Display_ShowI2CDevices(uint8_t *devices, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DEVICES_H */
