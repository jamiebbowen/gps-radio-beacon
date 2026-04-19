/**
 * @file compass.h
 * @brief Compass module driver header for Adafruit BNO055 IMU
 */

#ifndef __COMPASS_H
#define __COMPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Compass status codes */
#define COMPASS_OK     0
#define COMPASS_ERROR  1
#define COMPASS_TIMEOUT 2

/* Detailed error codes */
#define COMPASS_ERROR_NONE        0
#define COMPASS_ERROR_I2C_INIT    1
#define COMPASS_ERROR_ID_CHECK    2
#define COMPASS_ERROR_READ_REG    3
#define COMPASS_ERROR_WRITE_REG   4
#define COMPASS_ERROR_READ_DATA   5

/* BNO055 specific defines */
#define BNO055_I2C_ADDR         0x28  /* 7-bit I2C address for BNO055 (default) */
#define BNO055_I2C_ADDR_ALT     0x29  /* 7-bit I2C address for BNO055 (alternate) */
#define BNO055_CHIP_ID_VALUE    0xA0  /* BNO055 CHIP_ID value */

/* Compass data structure */
typedef struct {
  float heading;          /* Heading in degrees (0-359.9) */
  float pitch;            /* Pitch in degrees */
  float roll;             /* Roll in degrees */
  int16_t mag_x;          /* Raw magnetometer X value */
  int16_t mag_y;          /* Raw magnetometer Y value */
  int16_t mag_z;          /* Raw magnetometer Z value */
  int16_t accel_x;        /* Raw accelerometer X value */
  int16_t accel_y;        /* Raw accelerometer Y value */
  int16_t accel_z;        /* Raw accelerometer Z value */
  int16_t gyro_x;         /* Raw gyroscope X value */
  int16_t gyro_y;         /* Raw gyroscope Y value */
  int16_t gyro_z;         /* Raw gyroscope Z value */
  float cal_accel_x;      /* Calibrated accelerometer X value */
  float cal_accel_y;      /* Calibrated accelerometer Y value */
  float cal_accel_z;      /* Calibrated accelerometer Z value */
  float cal_gyro_x;       /* Calibrated gyroscope X value */
  float cal_gyro_y;       /* Calibrated gyroscope Y value */
  float cal_gyro_z;       /* Calibrated gyroscope Z value */
  uint32_t timestamp;     /* Timestamp of last update (system ticks) */
  uint32_t last_update;   /* Previous update timestamp for dt calculation */
  uint8_t  heading_valid; /* 1 = mag_cal >= 1 and heading is trustworthy, 0 = calibrating */
  uint8_t  mag_cal;       /* Magnetometer calibration level: 0=uncal, 3=fully cal */
} Compass_Data;

/* Function prototypes */
uint8_t Compass_Init(void);
uint8_t Compass_Update(Compass_Data *compass_data);
uint8_t Compass_Calibrate(void);
uint8_t Compass_SelfTest(void);
uint8_t Compass_GetCalibrationStatus(uint8_t *sys_cal, uint8_t *gyro_cal, uint8_t *accel_cal, uint8_t *mag_cal);
float Compass_GetHeading(void);
const char* Compass_GetErrorMessage(void);
uint8_t Compass_GetErrorCode(void);
void Compass_GetDebugInfo(uint8_t *active_addr, uint8_t *raw_bytes, uint8_t *status_reg, uint8_t *calib_status);
uint8_t Compass_GetError(void);
uint8_t Compass_GetFoundDeviceCount(void);
const uint8_t* Compass_GetFoundAddresses(void);
uint8_t Compass_DisplayI2CScan(void);
uint8_t Compass_SetHeadingOffset(float offset_deg);

/* Calibration persistence */
#define BNO055_CAL_DATA_LEN  22  /* ACC(6) + MAG(6) + GYR(6) + radii(4) bytes */
uint8_t Compass_GetCalibrationData(uint8_t *data, uint8_t len);
uint8_t Compass_SetCalibrationData(const uint8_t *data, uint8_t len);

/* Error reporting functions */
uint8_t Compass_GetLastErrorCode(void);
const char* Compass_GetErrorMessage(void);

/* Debug functions */
const char* Compass_GetErrorMessage(void);
const char* Compass_GetDebugMessage(void);
const char* Compass_GetDebugMessageByIndex(uint8_t index);
uint32_t Compass_GetDebugTimestamp(void);

/* System status functions */
uint8_t Compass_GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error);
uint8_t Compass_GetOperationMode(uint8_t *op_mode);
uint8_t Compass_GetPowerMode(uint8_t *pwr_mode);
uint8_t Compass_GetTemperature(int8_t *temp);

#ifdef __cplusplus
}
#endif

#endif /* __COMPASS_H */
