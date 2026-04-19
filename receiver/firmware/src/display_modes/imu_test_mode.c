/**
 * @file imu_test_mode.c
 * @brief IMU self-test display mode implementation
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
  
  /* Run self-test */
  uint8_t self_test_result = Compass_SelfTest();
  
  /* Display self-test result */
  if (self_test_result == COMPASS_OK) {    
    /* Get calibration status */
    uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
    Compass_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
    
    Display_DrawTextRowCol(0, 0, "BNO055");
    /* Display calibration status */
    char cal_buffer[32];
    snprintf(cal_buffer, sizeof(cal_buffer), "Cal: S%d G%d A%d M%d", 
             sys_cal, gyro_cal, accel_cal, mag_cal);
    Display_DrawTextRowCol(1, 0, cal_buffer);
    
    /* Get system status */
    uint8_t system_status, self_test_result_reg, system_error;
    Compass_GetSystemStatus(&system_status, &self_test_result_reg, &system_error);
    
    /* Display system status */
    char status_buffer[32];
    snprintf(status_buffer, sizeof(status_buffer), "Sys: 0x%02X", system_status);
    Display_DrawTextRowCol(2, 0, status_buffer);
    
    /* Display operation mode */
    uint8_t op_mode;
    Compass_GetOperationMode(&op_mode);
    char mode_buffer[32];
    snprintf(mode_buffer, sizeof(mode_buffer), "Op: 0x%02X", op_mode);
    Display_DrawTextRowCol(3, 0, mode_buffer);
    
    /* Display temperature */
    int8_t temp;
    Compass_GetTemperature(&temp);
    char temp_buffer[32];
    snprintf(temp_buffer, sizeof(temp_buffer), "Temp: %d C", temp);
    Display_DrawTextRowCol(4, 0, temp_buffer);
  } else {
    Display_DrawTextRowCol(0, 0, "BNO055");
    Display_DrawTextRowCol(1, 0, "Self-Test: FAIL");
    
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
