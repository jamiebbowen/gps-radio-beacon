/**
 * @file imu_test_mode.c
 * @brief IMU self-test display mode implementation
 *
 * The board carries an Adafruit 4754 (BNO085). The BNO085 has no
 * host-triggerable self-test register, no temperature register, and no
 * op-mode/power-mode registers; Compass_SelfTest checks liveness +
 * fusion flow instead, and this screen shows the SHTP-side counters
 * that are actually diagnostic on this chip (RV sample count, I2C addr,
 * per-sensor accuracy).
 */

#include "display_modes/imu_test_mode.h"
#include "display.h"
#include "compass.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief Display IMU self-test screen
 * 
 * Shows the results of the BNO055 IMU self-test and calibration status
 */
void DisplayMode_IMUTest(void)
{
  Display_Clear();
  
  /* Check the hub answers and the SH-2 fusion stream is flowing */
  uint8_t self_test_result = Compass_SelfTest();

  /* Display result */
  if (self_test_result == COMPASS_OK) {
    /* Get calibration status (per-sensor SH-2 accuracy, 0-3 each) */
    uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
    Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);

    Display_DrawTextRowCol(0, 0, "BNO085");
    /* Display calibration status */
    char cal_buffer[32];
    snprintf(cal_buffer, sizeof(cal_buffer), "Cal: S%d G%d A%d M%d",
             sys_cal, gyro_cal, accel_cal, mag_cal);
    Display_DrawTextRowCol(1, 0, cal_buffer);

    /* Fusion stream sanity: total rotation-vector samples received */
    char rv_buffer[32];
    snprintf(rv_buffer, sizeof(rv_buffer), "RV samples: %lu",
             (unsigned long)Compass_GetRvSampleCount());
    Display_DrawTextRowCol(2, 0, rv_buffer);

    /* I2C address the driver bound to (0x28 default / 0x29 alt strap) */
    uint8_t active_addr = 0;
    Compass_GetDebugInfo(&active_addr, NULL, NULL, NULL);
    char addr_buffer[32];
    snprintf(addr_buffer, sizeof(addr_buffer), "I2C addr: 0x%02X", active_addr);
    Display_DrawTextRowCol(3, 0, addr_buffer);

    Display_DrawTextRowCol(4, 0, "Mode: SH2 fused");
  } else {
    Display_DrawTextRowCol(0, 0, "BNO085");
    Display_DrawTextRowCol(1, 0, "Check: FAIL");
    
    /* Display error message */
    const char* error_msg = Compass_GetErrorMessage();
    Display_DrawTextRowCol(2, 0, error_msg);
    
    /* Display error code */
    uint8_t error_code = Compass_GetErrorCode();
    char error_buffer[32];
    snprintf(error_buffer, sizeof(error_buffer), "Err: 0x%02X", error_code);
    Display_DrawTextRowCol(3, 0, error_buffer);
  }
  
  Display_Update();
}
