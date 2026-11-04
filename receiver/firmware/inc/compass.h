/**
 * @file compass.h
 * @brief Compass driver header - Adafruit 4754 breakout (BNO085, SH-2/SHTP
 *        over I2C). The board carries the BNO085, not the BNO055 named by
 *        some legacy field/comment strings; the public API below is
 *        unchanged from the BNO055 build so the display modes and SD
 *        persistence call sites are untouched.
 */

#ifndef __COMPASS_H
#define __COMPASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Compass status codes (function return values) */
#define COMPASS_OK     0
#define COMPASS_ERROR  1
#define COMPASS_TIMEOUT 2

/* Detailed error codes - a SEPARATE namespace from the return statuses
 * above. These are stored via Compass_SetError and read back through
 * Compass_GetErrorCode / shown as "Err: 0xNN" on the IMU test screen.
 * (So COMPASS_ERROR_I2C_INIT == 1 == COMPASS_ERROR is coincidence, not
 * equivalence.) Values 0x06-0x08 continue the sequence in compass.c. */
#define COMPASS_ERROR_NONE        0
#define COMPASS_ERROR_I2C_INIT    1
#define COMPASS_ERROR_ID_CHECK    2
#define COMPASS_ERROR_READ_REG    3
#define COMPASS_ERROR_WRITE_REG   4
#define COMPASS_ERROR_READ_DATA   5

/* BNO085 I2C addresses are defined in bno085.h (0x28 default / 0x29 alt,
 * same straps as the BNO055 - the jogpad at 0x5A shares the bus). There is
 * no CHIP_ID: the BNO085 has no register map, identification is the SHTP
 * handshake + fusion-data arrival in the driver's init. */

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
  uint8_t  orientation_valid; /* 0 = device axis ~vertical; heading geometrically
                               * undefined regardless of calibration (tilt, not CAL) */
  uint8_t  heading_stale;     /* 1 = heading value predates a comm failure; keep
                               * pointing at it but tell the user it is stale */
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
/** Set the magnetic-declination term only; mounting offset stays baked in. */
uint8_t Compass_SetDeclination(float decl_deg);
/** Position-aware declination: applies the CONUS bilinear grid value for
 *  the caller's lat/lon exactly once per boot (first-call wins). Returns
 *  COMPASS_ERROR_GENERIC_FAIL when the coords are outside the grid's
 *  plausible domain - caller keeps the compile-time default. */
uint8_t Compass_UpdateLocation(float lat_deg, float lon_deg);
/** Table lookup only (no side effect); exported for tests. */
float Compass_DeclinationFromLocation(float lat_deg, float lon_deg);

/* Quaternion->heading convention lock. 1-8 = fixed convention from the
 * candidate table (compass.c). On the BNO085 the SH-2 rotation vector is
 * ENU-frame so the default is conv 5; the SD-persisted value can override
 * it for field tuning. NOTE: a QUATLOCK file written by the BNO055 build
 * is hardware-local to that sensor/mount - delete it on first BNO085
 * boot or the arrow may be rotated. */
uint8_t Compass_GetQuatLockConv(void);
uint8_t Compass_SetQuatLockConv(uint8_t conv);

/* Calibration persistence: the BNO085 does not expose host-managed
 * offsets (dynamic in-sensor calibration; DCD is unimplemented), so
 * Compass_Get/SetCalibrationData always return COMPASS_ERROR and
 * main.c skips the SD save/restore via Compass_HasHostCalOffsets(). */
#define BNO055_CAL_DATA_LEN  22  /* legacy buffer size kept for SD file compat */
uint8_t Compass_GetCalibrationData(uint8_t *data, uint8_t len);
uint8_t Compass_SetCalibrationData(const uint8_t *data, uint8_t len);
uint8_t Compass_WasCalRestored(void);
/** 1 if the sensor supports host offset save/restore (BNO085: 0). */
uint8_t Compass_HasHostCalOffsets(void);
/** Rotation-vector samples received since init (bring-up diagnostics). */
uint32_t Compass_GetRvSampleCount(void);

/* Error reporting functions */
uint8_t Compass_GetLastErrorCode(void);
const char* Compass_GetErrorMessage(void);

/* Debug functions */
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
