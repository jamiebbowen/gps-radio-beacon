/**
 ******************************************************************************
 * @file           : i2c_devices.c
 * @brief          : I2C device scan display implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "display_modes/i2c_devices.h"
#include "display.h"

/**
 * @brief Display I2C device scan results
 * @param devices Array of detected I2C device addresses
 * @param count Number of detected devices
 * @retval None
 */
void Display_ShowI2CDevices(uint8_t *devices, uint8_t count)
{
  char buffer[32];
  
  Display_Clear();
  Display_DrawTextRowCol(0, 0, "I2C Devices:");
  
  if (count == 0) {
    Display_DrawTextRowCol(2, 0, "No devices found");
  } else {
    sprintf(buffer, "Found %d devices:", count);
    Display_DrawTextRowCol(2, 0, buffer);
    
    /* Display up to 8 device addresses */
    for (uint8_t i = 0; i < count && i < 8; i++) {
      sprintf(buffer, "0x%02X", devices[i]);
      Display_DrawTextRowCol(4 + i, 0, buffer);
    }
  }
  
  Display_Update();
}
