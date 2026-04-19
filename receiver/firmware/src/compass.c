/**
 * @file compass.c
 * @brief Compass module driver implementation for Adafruit BNO055
 */

#include "compass.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "display.h"

/* Private defines */
#define BNO055_I2C                I2C1  /* Using I2C1 via hi2c_display */
#define BNO055_I2C_ADDR           0x28  /* Default 7-bit I2C address (ADR pin low) */
#define BNO055_I2C_ADDR_ALT       0x29  /* Alternative 7-bit I2C address (ADR pin high) */
#define BNO055_I2C_TIMEOUT        10  /* Timeout for I2C communication - short to prevent blocking LoRa RX */

/* Additional error codes */
#define COMPASS_ERROR_DEVICE_NOT_FOUND 6

/* BNO055 register addresses */
#define BNO055_CHIP_ID_ADDR       0x00  /* BNO055 CHIP ID register address */
#define BNO055_ID                 0xA0  /* BNO055 CHIP ID value */

/* BNO055 registers - Updated with correct addresses from datasheet */
#define BNO055_PAGE_ID_ADDR       0x07  /* Page ID register */
#define BNO055_CHIP_ID            0x00  /* Chip ID register */
#define BNO055_ACCEL_REV_ID_ADDR  0x01  /* Accelerometer revision ID */
#define BNO055_MAG_REV_ID_ADDR    0x02  /* Magnetometer revision ID */
#define BNO055_GYRO_REV_ID_ADDR   0x03  /* Gyroscope revision ID */
#define BNO055_SW_REV_ID_LSB_ADDR 0x04  /* SW revision ID LSB */
#define BNO055_SW_REV_ID_MSB_ADDR 0x05  /* SW revision ID MSB */
#define BNO055_BL_REV_ID_ADDR     0x06  /* Bootloader version */

/* BNO055 system registers */
#define BNO055_SYS_TRIGGER_ADDR   0x3F  /* System trigger register */
#define BNO055_PWR_MODE_ADDR      0x3E  /* Power mode register */
#define BNO055_OPR_MODE_ADDR      0x3D  /* Operation mode register */
#define BNO055_SYS_STAT_ADDR      0x39  /* System status register */
#define BNO055_SYS_ERR_ADDR       0x3A  /* System error register */
#define BNO055_UNIT_SEL_ADDR      0x3B  /* Unit selection register */
#define BNO055_AXIS_MAP_CONFIG    0x41  /* Axis map configuration register */
#define BNO055_AXIS_MAP_SIGN      0x42  /* Axis map sign register */

/* BNO055 calibration registers */
#define BNO055_CALIB_STAT_ADDR    0x35  /* Calibration status register */
#define BNO055_ST_RESULT_ADDR     0x36  /* Self-test result register */
#define BNO055_CAL_DATA_ADDR      0x55  /* First calibration offset register (ACC_OFFSET_X_LSB) */

/* BNO055 axis data registers */
#define BNO055_ACCEL_DATA_X_LSB   0x08  /* Acceleration data X LSB */
#define BNO055_ACCEL_DATA_X_MSB   0x09  /* Acceleration data X MSB */
#define BNO055_ACCEL_DATA_Y_LSB   0x0A  /* Acceleration data Y LSB */
#define BNO055_ACCEL_DATA_Y_MSB   0x0B  /* Acceleration data Y MSB */
#define BNO055_ACCEL_DATA_Z_LSB   0x0C  /* Acceleration data Z LSB */
#define BNO055_ACCEL_DATA_Z_MSB   0x0D  /* Acceleration data Z MSB */

#define BNO055_GYRO_DATA_X_LSB    0x14  /* Gyroscope data X LSB */
#define BNO055_GYRO_DATA_X_MSB    0x15  /* Gyroscope data X MSB */
#define BNO055_GYRO_DATA_Y_LSB    0x16  /* Gyroscope data Y LSB */
#define BNO055_GYRO_DATA_Y_MSB    0x17  /* Gyroscope data Y MSB */
#define BNO055_GYRO_DATA_Z_LSB    0x18  /* Gyroscope data Z LSB */
#define BNO055_GYRO_DATA_Z_MSB    0x19  /* Gyroscope data Z MSB */

#define BNO055_MAG_DATA_X_LSB     0x0E  /* Magnetometer data X LSB */
#define BNO055_MAG_DATA_X_MSB     0x0F  /* Magnetometer data X MSB */
#define BNO055_MAG_DATA_Y_LSB     0x10  /* Magnetometer data Y LSB */
#define BNO055_MAG_DATA_Y_MSB     0x11  /* Magnetometer data Y MSB */
#define BNO055_MAG_DATA_Z_LSB     0x12  /* Magnetometer data Z LSB */
#define BNO055_MAG_DATA_Z_MSB     0x13  /* Magnetometer data Z MSB */

/* BNO055 Euler angle data registers */
#define BNO055_EULER_H_LSB        0x1A  /* Heading LSB */
#define BNO055_EULER_H_MSB        0x1B  /* Heading MSB */
#define BNO055_EULER_R_LSB        0x1C  /* Roll LSB */
#define BNO055_EULER_R_MSB        0x1D  /* Roll MSB */
#define BNO055_EULER_P_LSB        0x1E  /* Pitch LSB */
#define BNO055_EULER_P_MSB        0x1F  /* Pitch MSB */
/* BNO055 configuration values */
#define BNO055_CHIP_ID_VALUE      0xA0  /* Expected chip ID value */

/* BNO055 power modes */
#define BNO055_POWER_MODE_NORMAL   0x00  /* Normal power mode */
#define BNO055_POWER_MODE_LOW      0x01  /* Low power mode */
#define BNO055_POWER_MODE_SUSPEND  0x02  /* Suspend power mode */

/* BNO055 operation modes */
#define BNO055_OPR_MODE_CONFIG     0x00  /* Configuration mode */
#define BNO055_OPR_MODE_ACCONLY    0x01  /* Accelerometer only mode */
#define BNO055_OPR_MODE_MAGONLY    0x02  /* Magnetometer only mode */
#define BNO055_OPR_MODE_GYROONLY   0x03  /* Gyroscope only mode */
#define BNO055_OPR_MODE_ACCMAG     0x04  /* Accelerometer and magnetometer mode */
#define BNO055_OPR_MODE_ACCGYRO    0x05  /* Accelerometer and gyroscope mode */
#define BNO055_OPR_MODE_MAGGYRO    0x06  /* Magnetometer and gyroscope mode */
#define BNO055_OPR_MODE_AMG        0x07  /* All three sensors mode */
#define BNO055_OPR_MODE_IMU        0x08  /* IMU mode */
#define BNO055_OPR_MODE_COMPASS    0x09  /* Compass mode */
#define BNO055_OPR_MODE_M4G        0x0A  /* Magnetometer for gyroscope mode */
#define BNO055_OPR_MODE_NDOF_FMC   0x0B  /* NDOF mode with fast magnetometer calibration */
#define BNO055_OPR_MODE_NDOF       0x0C  /* NDOF mode */

/* BNO055 system status values */
#define BNO055_SYS_STATUS_IDLE     0x00  /* System idle */
#define BNO055_SYS_STATUS_ERROR    0x01  /* System error */
#define BNO055_SYS_STATUS_INIT_PERIPH 0x02  /* Initializing peripherals */
#define BNO055_SYS_STATUS_INIT_SYS 0x03  /* System initialization */
#define BNO055_SYS_STATUS_SELFTEST 0x04  /* Self-test */
#define BNO055_SYS_STATUS_RUNNING  0x05  /* System running with fusion */
#define BNO055_SYS_STATUS_RUNNING_NO_FUSION 0x06  /* System running without fusion */

/* Additional error codes (COMPASS_ERROR_NONE is already defined in compass.h) */
#define COMPASS_ERROR_COMM        0x05  /* Communication error */
#define COMPASS_ERROR_CALIB       0x06  /* Calibration error */
#define COMPASS_ERROR_MODE        0x07  /* Mode setting error */
#define COMPASS_ERROR_SYS_STATUS  0x08  /* System status error */

/* BNO055 calibration status bits */
#define BNO055_CALIB_STAT_SYS_MASK  0xC0  /* System calibration status mask */
#define BNO055_CALIB_STAT_GYR_MASK  0x30  /* Gyroscope calibration status mask */
#define BNO055_CALIB_STAT_ACC_MASK  0x0C  /* Accelerometer calibration status mask */
#define BNO055_CALIB_STAT_MAG_MASK  0x03  /* Magnetometer calibration status mask */

/* -----------------------------------------------------------------------
 * Heading correction constants
 * -----------------------------------------------------------------------
 * COMPASS_MAGNETIC_DECLINATION_DEG
 *   Magnetic declination for the operating location (degrees, + = East).
 *   Denver, CO area: ~+8.5°  (magnetic North is 8.5° East of true North)
 *   Update this when operating in a different region.
 *
 * COMPASS_MOUNTING_OFFSET_DEG
 *   Physical mounting correction so the calibrated BNO055 heading aligns
 *   with the antenna direction.  Sensor +X axis is 90° CCW from antenna.
 *   Measured with calibrated NDOF: pointing North displayed 180° (South)
 *   with 270° offset → correct = -270° ≡ 90°.
 * ----------------------------------------------------------------------- */
#define COMPASS_MAGNETIC_DECLINATION_DEG   8.5f
#define COMPASS_MOUNTING_OFFSET_DEG         90.0f  /* Sensor +X is 90° CCW from antenna direction.
                                                    * Derived: with 270° offset, calibrated heading showed 180° (South)
                                                    * when pointing North → BNO055_cal_raw = 261.5° →
                                                    * correct offset = 0° - 261.5° - 8.5° = -270° ≡ 90°.
                                                    * Note: uncalibrated heading is meaningless; only trust
                                                    * heading when heading_valid=1 (mag_cal >= 1). */

/* BNO055 axis remapping (applied in Compass_Init)
 * AXIS_MAP_CONFIG 0x24 = 0b00_10_01_00:
 *   bits[5:4]=2 (new Z <- physical Z)
 *   bits[3:2]=1 (new Y <- physical Y)
 *   bits[1:0]=0 (new X <- physical X)
 *   → identity mapping (P1 default); physical axes are passed through.
 *
 * AXIS_MAP_SIGN  0x00 = 0b000_0_0_000:
 *   bit0=0 no negate X, bit1=0 no negate Y, bit2=0 keep Z
 *   → no sign inversion (identity). Mounting correction
 *   is handled purely in software via COMPASS_MOUNTING_OFFSET_DEG. */
#define COMPASS_AXIS_MAP_CONFIG  0x24u
#define COMPASS_AXIS_MAP_SIGN    0x00u

/* Complementary filter weight for heading fusion.
 * 0.0 = trust gyro only, 1.0 = trust magnetometer only.
 * 0.98 keeps the magnetometer as primary reference while gyro
 * smooths out short-term jitter between magnetometer reads. */
#define COMPASS_COMP_FILTER_ALPHA  0.98f

/* Error reporting */
static uint8_t compass_error_code = COMPASS_ERROR_NONE;
static char compass_error_msg[64] = "No error";

/* Debug messages */
static uint32_t debug_timestamp = 0;

/* I2C handle */
extern I2C_HandleTypeDef hi2c_display; /* Using display I2C for compass */

/* Active I2C address */
static uint8_t active_addr = BNO055_I2C_ADDR;

/* Debug structure for compass data */
typedef struct {
  uint8_t active_addr;      /* Active I2C address */
  uint8_t raw_bytes[6];     /* Raw bytes from sensor */
  uint8_t status_reg;       /* Status register value */
  uint8_t calib_status;     /* Combined calibration status */
  uint8_t calib_sys;        /* System calibration status (0-3) */
  uint8_t calib_gyro;       /* Gyroscope calibration status (0-3) */
  uint8_t calib_accel;      /* Accelerometer calibration status (0-3) */
  uint8_t calib_mag;        /* Magnetometer calibration status (0-3) */
  uint8_t sys_status;       /* System status */
  uint8_t found_device_count; /* Number of found I2C devices */
  int16_t mag_data[3];      /* X, Y, Z magnetometer data */
  int16_t accel_data[3];    /* X, Y, Z accelerometer data */
  int16_t gyro_data[3];     /* X, Y, Z gyroscope data */
  float heading;            /* Calculated heading in degrees */
  float roll;               /* Roll angle in degrees */
  float pitch;              /* Pitch angle in degrees */
  char debug_messages[8][64]; /* Debug message strings */
  char error_message[64];   /* Error message */
  uint8_t error_code;       /* Error code */
} Compass_Debug_t;

/* Global debug structure */
static Compass_Debug_t compass_debug = {0};

/* Global variables */
static int16_t raw_data[3] = {0};
static uint8_t found_i2c_addresses[128] = {0}; /* Array to store found I2C addresses */

/* Calibration variables - BNO055 handles calibration internally */
static float heading_offset = 0.0f; /* Optional heading offset for declination */

/* Function prototypes */
static uint8_t Compass_ReadRegister(uint8_t reg, uint8_t *data);
static uint8_t Compass_WriteRegister(uint8_t reg, uint8_t data);
static void Compass_SetError(uint8_t error_code, const char *error_msg, uint8_t hal_status);
static void Compass_SetDebug(uint8_t msg_type, const char *msg);
static uint8_t Compass_ReadData(void);
static uint8_t Compass_SetMode(uint8_t mode);
static uint8_t Compass_CheckBNO055(void);

/**
 * @brief Check BNO055 CHIP_ID register with robust retry logic
 * @retval 1 if BNO055 found, 0 otherwise
 */
static uint8_t Compass_CheckBNO055(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t chip_id = 0;
  uint8_t addr;
  char debug_msg[100];
  int retry;
  
  /* First try the default address with retry logic */
  addr = BNO055_I2C_ADDR; // 0x28
  
  /* BNO055 can take up to 850ms to boot - use retry logic with delays */
  for (retry = 0; retry < 5; retry++) {
    /* Read CHIP_ID register */
    hal_status = HAL_I2C_Mem_Read(
      &hi2c_display,
      addr << 1,
      BNO055_CHIP_ID_ADDR,
      I2C_MEMADD_SIZE_8BIT,
      &chip_id,
      1,
      BNO055_I2C_TIMEOUT
    );
    
    if (hal_status == HAL_OK) {
      snprintf(debug_msg, sizeof(debug_msg), "BNO055 CHIP_ID at addr 0x%02X = 0x%02X (expected 0xA0), retry %d", 
               addr, chip_id, retry);
      Compass_SetDebug(0, debug_msg);
      
      /* BNO055 CHIP_ID should be 0xA0 */
      if (chip_id == BNO055_ID) {
        compass_debug.active_addr = addr;
        return COMPASS_OK;
      }
    }
    
    /* Wait before retry - BNO055 needs time after power-up */
    HAL_Delay(100);
  }
  
  /* Try the alternative address with retry logic */
  addr = BNO055_I2C_ADDR_ALT; // 0x29
  
  for (retry = 0; retry < 5; retry++) {
    /* Read CHIP_ID register */
    hal_status = HAL_I2C_Mem_Read(
      &hi2c_display,
      addr << 1,
      BNO055_CHIP_ID_ADDR,
      I2C_MEMADD_SIZE_8BIT,
      &chip_id,
      1,
      BNO055_I2C_TIMEOUT
    );
    
    if (hal_status == HAL_OK) {
      snprintf(debug_msg, sizeof(debug_msg), "BNO055 CHIP_ID at addr 0x%02X = 0x%02X (expected 0xA0), retry %d", 
               addr, chip_id, retry);
      Compass_SetDebug(0, debug_msg);
      
      /* BNO055 CHIP_ID should be 0xA0 */
      if (chip_id == BNO055_ID) {
        compass_debug.active_addr = addr;
        return COMPASS_OK;
      }
    }
    
    /* Wait before retry */
    HAL_Delay(100);
  }
  
  /* If we get here, we couldn't find the BNO055 */
  /* Use the default address anyway */
  compass_debug.active_addr = BNO055_I2C_ADDR;
  snprintf(debug_msg, sizeof(debug_msg), "Using addr 0x%02X despite CHIP_ID failure after multiple retries", compass_debug.active_addr);
  Compass_SetError(COMPASS_ERROR_DEVICE_NOT_FOUND, debug_msg, 0);
  
  /* Return error */
  return COMPASS_ERROR;
}

/**
 * @brief Scan I2C bus for devices
 * @param detected_addrs Array to store detected addresses (must be at least 16 bytes)
 * @param max_addrs Maximum number of addresses to store
 * @retval Number of devices found
 */
static uint8_t Compass_ScanI2C(uint8_t *detected_addrs, uint8_t max_addrs)
{
  HAL_StatusTypeDef hal_status;
  uint8_t devices_found = 0;
  char debug_msg[100];
  
  /* Scan all possible 7-bit I2C addresses */
  for (uint8_t addr = 1; addr < 128 && devices_found < max_addrs; addr++) {
    /* Try to communicate with the device with multiple retries and longer timeout */
    hal_status = HAL_I2C_IsDeviceReady(&hi2c_display, addr << 1, 5, 200);
    
    if (hal_status == HAL_OK) {
      /* Store the actual address value, not shifted */
      detected_addrs[devices_found] = addr;
      devices_found++;
      
      /* Debug output for each found device */
      snprintf(debug_msg, sizeof(debug_msg), "I2C device found at address 0x%02X, stored at index %d", 
               addr, devices_found-1);
      Compass_SetError(0, debug_msg, 0);
    }
  }
  
  /* Debug output for all detected addresses */
  if (devices_found > 0) {
    snprintf(debug_msg, sizeof(debug_msg), "Detected addrs: 0x%02X 0x%02X", 
             detected_addrs[0], (devices_found > 1) ? detected_addrs[1] : 0);
    Compass_SetError(0, debug_msg, 0);
  }
  
  /* Store count in debug structure */
  compass_debug.found_device_count = devices_found;
  
  if (devices_found == 0) {
    Compass_SetError(COMPASS_ERROR_I2C_INIT, "No I2C devices found", 0);
    return 0;
  } else {
    snprintf(debug_msg, sizeof(debug_msg), "Found %d I2C device(s)", devices_found);
    Compass_SetError(0, debug_msg, 0);
    return devices_found;
  }
}

/**
 * @brief Set error message and code
 * @param error_code Error code
 * @param error_msg Error message
 * @param hal_status HAL status code
 */
static void Compass_SetError(uint8_t error_code, const char *error_msg, uint8_t hal_status)
{
  strncpy(compass_debug.error_message, error_msg, sizeof(compass_debug.error_message) - 1);
  compass_debug.error_message[sizeof(compass_debug.error_message) - 1] = '\0';
  compass_debug.error_code = error_code;
  /* HAL status is not stored in the current debug structure */
}

/**
 * @brief Set debug message for real-time display
 * @param msg_type Message type (0-7)
 * @param msg Debug message
 */
static void Compass_SetDebug(uint8_t msg_type, const char *msg)
{
  if (msg_type < 8) { // We have 8 debug messages in the debug_messages array
    strncpy(compass_debug.debug_messages[msg_type], msg, sizeof(compass_debug.debug_messages[0]) - 1);
    compass_debug.debug_messages[msg_type][sizeof(compass_debug.debug_messages[0]) - 1] = '\0';
  }
}

/**
 * @brief Get current debug message
 * @retval Debug message string
 */
const char* Compass_GetDebugMessage(void)
{
  uint8_t debug_msg_index = 0;
  return compass_debug.debug_messages[debug_msg_index];
}

/**
 * @brief Get debug message by index
 * @param index Index of debug message to retrieve (0-7)
 * @retval Debug message string at specified index
 */
const char* Compass_GetDebugMessageByIndex(uint8_t index)
{
  /* Ensure index is within valid range */
  if (index < 8) { /* We have 8 debug messages */
    return compass_debug.debug_messages[index];
  }
  return "";
}

/**
 * @brief Get debug timestamp
 * @retval Debug timestamp in milliseconds
 */
uint32_t Compass_GetDebugTimestamp(void)
{
  return debug_timestamp;
}

/**
 * @brief Read 22-byte BNO055 calibration offset/radius data
 * @note  BNO055 must be running (NDOF) before calling; function briefly
 *        switches to CONFIG mode, reads the data, then restores NDOF.
 * @param data  Output buffer of at least BNO055_CAL_DATA_LEN bytes
 * @param len   Must be >= BNO055_CAL_DATA_LEN
 * @retval COMPASS_OK on success
 */
uint8_t Compass_GetCalibrationData(uint8_t *data, uint8_t len)
{
  if (data == NULL || len < BNO055_CAL_DATA_LEN) return COMPASS_ERROR;

  if (Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG) != COMPASS_OK)
    return COMPASS_ERROR;
  HAL_Delay(25);

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
    &hi2c_display, compass_debug.active_addr << 1,
    BNO055_CAL_DATA_ADDR, I2C_MEMADD_SIZE_8BIT,
    data, BNO055_CAL_DATA_LEN, BNO055_I2C_TIMEOUT);

  Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF);
  HAL_Delay(20);

  return (status == HAL_OK) ? COMPASS_OK : COMPASS_ERROR;
}

/**
 * @brief Restore 22-byte BNO055 calibration offset/radius data
 * @note  Switches to CONFIG mode, writes the data, then restores NDOF.
 *        Call this right after Compass_Init() to skip the calibration dance.
 * @param data  Buffer of BNO055_CAL_DATA_LEN bytes previously read by
 *              Compass_GetCalibrationData()
 * @param len   Must be >= BNO055_CAL_DATA_LEN
 * @retval COMPASS_OK on success
 */
uint8_t Compass_SetCalibrationData(const uint8_t *data, uint8_t len)
{
  if (data == NULL || len < BNO055_CAL_DATA_LEN) return COMPASS_ERROR;

  if (Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG) != COMPASS_OK)
    return COMPASS_ERROR;
  HAL_Delay(25);

  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
    &hi2c_display, compass_debug.active_addr << 1,
    BNO055_CAL_DATA_ADDR, I2C_MEMADD_SIZE_8BIT,
    (uint8_t *)data, BNO055_CAL_DATA_LEN, BNO055_I2C_TIMEOUT);

  Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF);
  HAL_Delay(20);

  return (status == HAL_OK) ? COMPASS_OK : COMPASS_ERROR;
}

/**
 * @brief Read a register from the compass
 * @param reg Register address
 * @param data Pointer to data buffer
 * @retval Status code
 */
static uint8_t Compass_ReadRegister(uint8_t reg, uint8_t *data)
{
  uint8_t active_addr = compass_debug.active_addr;
  HAL_StatusTypeDef hal_status;
  
  /* Send register address */
  hal_status = HAL_I2C_Master_Transmit(&hi2c_display, active_addr << 1, &reg, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to send register address", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Read data */
  hal_status = HAL_I2C_Master_Receive(&hi2c_display, active_addr << 1, data, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read register", hal_status);
    return COMPASS_ERROR;
  }
  
  return 0;
}

/**
 * @brief Write to a register on the compass
 * @param reg Register address
 * @param data Data to write
 * @retval Status code
 */
static uint8_t Compass_WriteRegister(uint8_t reg, uint8_t data)
{
  uint8_t buf[2];
  uint8_t active_addr = compass_debug.active_addr;
  HAL_StatusTypeDef hal_status;
  
  /* Prepare buffer */
  buf[0] = reg;
  buf[1] = data;
  
  hal_status = HAL_I2C_Master_Transmit(&hi2c_display, active_addr << 1, buf, 2, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to write register", hal_status);
    return COMPASS_ERROR;
  }
  
  return 0;
}

/**
 * @brief Initialize the BNO055 IMU with robust initialization sequence
 * @retval Status code
 */
uint8_t Compass_Init(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t chip_id = 0;
  uint8_t status;
  char debug_msg[100];
  int retry;
  
  /* First scan the I2C bus for any devices */
  uint8_t detected_addrs[16] = {0};
  uint8_t device_count = Compass_ScanI2C(detected_addrs, 16);
  
  /* Check if any I2C devices were found */
  if (device_count == 0) {
    /* No I2C devices found at all, likely a hardware issue */
    Compass_SetError(COMPASS_ERROR, "No I2C devices found", 0);
    return COMPASS_ERROR;
  }
  
  /* Debug - log detected devices */
  snprintf(debug_msg, sizeof(debug_msg), "Found %d I2C device(s), proceeding with init", device_count);
  Compass_SetDebug(0, debug_msg);
  
  /* Use robust detection with retries - BNO055 can take up to 850ms to boot */
  int timeout = 850; /* ms */
  bool device_found = false;
  
  /* Try both addresses with retry logic */
  while (timeout > 0 && !device_found) {
    /* Try default address first */
    if (HAL_I2C_IsDeviceReady(&hi2c_display, BNO055_I2C_ADDR << 1, 1, 100) == HAL_OK) {
      active_addr = BNO055_I2C_ADDR;
      device_found = true;
      break;
    }
    
    /* Try alternate address */
    if (HAL_I2C_IsDeviceReady(&hi2c_display, BNO055_I2C_ADDR_ALT << 1, 1, 100) == HAL_OK) {
      active_addr = BNO055_I2C_ADDR_ALT;
      device_found = true;
      break;
    }
    
    /* Wait and retry */
    HAL_Delay(10);
    timeout -= 10;
  }
  
  if (!device_found) {
    Compass_SetError(COMPASS_ERROR_I2C_INIT, "BNO055 not found after timeout", 0);
    return COMPASS_ERROR;
  }
  
  compass_debug.active_addr = active_addr;
  snprintf(debug_msg, sizeof(debug_msg), "Using I2C address: 0x%02X for BNO055", active_addr);
  Compass_SetDebug(0, debug_msg);
  
  /* Check chip ID with retry logic */
  for (retry = 0; retry < 3; retry++) {
    hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_CHIP_ID_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &chip_id, 1, BNO055_I2C_TIMEOUT);
    
    if (hal_status == HAL_OK && chip_id == BNO055_CHIP_ID_VALUE) {
      break;
    }
    
    /* Wait before retry */
    HAL_Delay(100);
  }
  
  if (chip_id != BNO055_CHIP_ID_VALUE) {
    snprintf(debug_msg, sizeof(debug_msg), "BNO055 wrong chip ID: 0x%02X (expected 0xA0)", chip_id);
    Compass_SetError(COMPASS_ERROR_ID_CHECK, debug_msg, hal_status);
    return COMPASS_ERROR;
  }
  
  /* Switch to config mode (just in case since this is the default) */
  status = Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set CONFIG mode", 0);
    /* Continue anyway */
  }
  HAL_Delay(25); /* Wait for mode change */
  
  /* Reset the device - Adafruit library does this AFTER switching to config mode */
  status = Compass_WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x20); /* Reset system */
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to reset BNO055", 0);
    /* Continue anyway */
  }
  
  /* Delay increased to 30ms per Adafruit library due to power issues */
  HAL_Delay(30);
  
  /* Wait for the chip to come back online after reset */
  timeout = 1000; /* ms */
  while (timeout > 0) {
    hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_CHIP_ID_ADDR, 
                                I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 100);
    
    if (hal_status == HAL_OK && chip_id == BNO055_CHIP_ID_VALUE) {
      break;
    }
    
    HAL_Delay(10);
    timeout -= 10;
  }
  
  if (chip_id != BNO055_CHIP_ID_VALUE) {
    snprintf(debug_msg, sizeof(debug_msg), "BNO055 not responding after reset");
    Compass_SetError(COMPASS_ERROR_ID_CHECK, debug_msg, hal_status);
    return COMPASS_ERROR;
  }
  
  /* Additional delay after reset per Adafruit library */
  HAL_Delay(50);
  
  /* Set power mode to normal */
  status = Compass_WriteRegister(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set power mode", 0);
    /* Continue anyway */
  }
  HAL_Delay(10);
  
  /* Set page to 0 */
  status = Compass_WriteRegister(BNO055_PAGE_ID_ADDR, 0);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set page ID", 0);
    /* Continue anyway */
  }
  HAL_Delay(10);
  
  /* Reset system trigger to normal */
  status = Compass_WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x0);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to reset system trigger", 0);
    /* Continue anyway */
  }
  HAL_Delay(10);
  
  /* Configure axis remapping if needed */
  /* The BNO055 default orientation is: X=right, Y=forward, Z=up */
  /* If the sensor is mounted differently, we need to remap the axes */
  
  /* First, set the device to CONFIG mode to allow register changes */
  status = Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_CONFIG);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set CONFIG mode for axis remap", 0);
    /* Continue anyway */
  }
  HAL_Delay(25); /* Wait for mode change */
  
  /* Configure axis mapping for desired orientation: +X=right, +Y=forward, +Z=up */
  /* AXIS_MAP_CONFIG register bits [5:4]=new Z src, [3:2]=new Y src, [1:0]=new X src */
  /* Values: 0=X, 1=Y, 2=Z */
  /* 
   * 0x24 = 00 10 01 00 = new Z<-Z, new Y<-Y, new X<-X (P1 default, identity mapping)
   *   Result: X=right, Y=forward, Z=up (matches desired antenna orientation)
   */
  uint8_t axis_map_config = COMPASS_AXIS_MAP_CONFIG;
  uint8_t axis_map_sign    = COMPASS_AXIS_MAP_SIGN;
  
  /* Write axis remapping configuration */
  status = Compass_WriteRegister(BNO055_AXIS_MAP_CONFIG, axis_map_config);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set axis map config", 0);
    /* Continue anyway */
  }
  HAL_Delay(10);
  
  status = Compass_WriteRegister(BNO055_AXIS_MAP_SIGN, axis_map_sign);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set axis map sign", 0);
    /* Continue anyway */
  }
  HAL_Delay(10);
  
  /* Log the axis mapping configuration */
  snprintf(debug_msg, sizeof(debug_msg), "Axis map config: 0x%02X, sign: 0x%02X", 
           axis_map_config, axis_map_sign);
  Compass_SetDebug(1, debug_msg);
  
  /* Set to NDOF mode (9-axis fusion) */
  status = Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF);
  if (status != COMPASS_OK) {
    Compass_SetError(COMPASS_ERROR_WRITE_REG, "Failed to set NDOF mode", 0);
    /* Continue anyway */
  }
  
  /* Wait longer for mode change and sensor stabilization - BNO055 needs time to start fusion */
  HAL_Delay(200); /* Increased to 200ms to give more time for initialization */
  
  /* Verify operation mode */
  uint8_t opr_mode = 0;
  if (HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_OPR_MODE_ADDR, 
                     I2C_MEMADD_SIZE_8BIT, &opr_mode, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
    opr_mode &= 0x0F; /* Mask to get just the operation mode bits */
    if (opr_mode != BNO055_OPR_MODE_NDOF) {
      snprintf(debug_msg, sizeof(debug_msg), "BNO055 not in NDOF mode: 0x%02X", opr_mode);
      Compass_SetError(COMPASS_ERROR_MODE, debug_msg, 0);
      
      /* Try setting mode again */
      status = Compass_WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPR_MODE_NDOF);
      HAL_Delay(100);
    } else {
      snprintf(debug_msg, sizeof(debug_msg), "BNO055 in NDOF mode");
      Compass_SetDebug(1, debug_msg);
    }
  } else {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to verify operation mode", hal_status);
  }
  
  /* Read system status */
  uint8_t sys_status = 0;
  if (HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_SYS_STAT_ADDR, 
                     I2C_MEMADD_SIZE_8BIT, &sys_status, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
    snprintf(debug_msg, sizeof(debug_msg), "BNO055 system status: 0x%02X", sys_status);
    Compass_SetDebug(1, debug_msg);
    
    if (sys_status < BNO055_SYS_STATUS_RUNNING) {
      Compass_SetError(COMPASS_ERROR, "BNO055 not running with fusion", 0);
      /* Continue anyway */
      uint8_t sys_err = 0;
      if (HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_SYS_ERR_ADDR, 
                         I2C_MEMADD_SIZE_8BIT, &sys_err, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
        snprintf(debug_msg, sizeof(debug_msg), "System Error: 0x%02X", sys_err);
        Compass_SetError(COMPASS_ERROR, debug_msg, 0);
      }
    } else {
      snprintf(debug_msg, sizeof(debug_msg), "BNO055 running with fusion");
      Compass_SetDebug(2, debug_msg);
    }
  } else {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read system status", hal_status);
  }
  
  /* Read calibration status */
  uint8_t calib_stat = 0;
  if (HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_CALIB_STAT_ADDR, 
                     I2C_MEMADD_SIZE_8BIT, &calib_stat, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
    uint8_t sys_cal = (calib_stat >> 6) & 0x03;
    uint8_t gyro_cal = (calib_stat >> 4) & 0x03;
    uint8_t accel_cal = (calib_stat >> 2) & 0x03;
    uint8_t mag_cal = calib_stat & 0x03;
    
    snprintf(debug_msg, sizeof(debug_msg), "Initial Cal: S%d G%d A%d M%d", 
             sys_cal, gyro_cal, accel_cal, mag_cal);
    Compass_SetDebug(3, debug_msg);
    
    /* Store calibration status in debug structure */
    compass_debug.calib_status = calib_stat;
  } else {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read calibration status", hal_status);
  }
  
  /* Total heading offset = magnetic declination + physical mounting correction.
   * See constant definitions above for derivation details. */
  heading_offset = COMPASS_MAGNETIC_DECLINATION_DEG + COMPASS_MOUNTING_OFFSET_DEG;
  snprintf(debug_msg, sizeof(debug_msg), "Mag declination: %.1f deg", heading_offset);
  Compass_SetDebug(3, debug_msg);
  
  return COMPASS_OK;
}

/**
 * @brief Update compass data
 * @param compass_data Pointer to compass data structure
 * @retval Status code
 */
uint8_t Compass_Update(Compass_Data *compass_data_ptr)
{
  HAL_StatusTypeDef hal_status;
  char debug_msg[100];
  uint32_t current_time = HAL_GetTick();
  float dt = 0.0f;
  uint8_t data_buffer[12]; // Buffer for reading multiple registers
  
  /* Calculate time delta in seconds */
  if (compass_data_ptr->last_update > 0) {
    dt = (float)(current_time - compass_data_ptr->last_update) / 1000.0f; /* Convert ms to seconds */
    
    /* Sanity check on dt - if more than 1 second or negative, use default */
    if (dt > 1.0f || dt <= 0.0f) {
      dt = 0.01f; /* Default 10ms if time delta is unreasonable */
    }
  } else {
    dt = 0.01f; /* Default for first update */
  }
  compass_data_ptr->last_update = current_time;
  compass_data_ptr->timestamp = current_time;
  
  /* Read Euler angle data (heading, roll, pitch) */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_EULER_H_LSB, I2C_MEMADD_SIZE_8BIT, 
                               data_buffer, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read Euler angles", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse Euler angles - BNO055 stores data as little-endian */
  int16_t heading_raw = (int16_t)((data_buffer[1] << 8) | data_buffer[0]);
  int16_t roll_raw = (int16_t)((data_buffer[3] << 8) | data_buffer[2]);
  int16_t pitch_raw = (int16_t)((data_buffer[5] << 8) | data_buffer[4]);
  
  /* Store heading in debug structure */
  compass_debug.heading = (float)heading_raw / 16.0f;
  compass_debug.roll = (float)roll_raw / 16.0f;
  compass_debug.pitch = (float)pitch_raw / 16.0f;
  
  /* Convert to degrees (BNO055 Euler angles are in degrees/16) */
  float heading = (float)heading_raw / 16.0f;
  float roll = (float)roll_raw / 16.0f;
  float pitch = (float)pitch_raw / 16.0f;
  
  /* Log raw heading value for debugging */
  snprintf(debug_msg, sizeof(debug_msg), "Raw heading: %.1f", heading);
  Compass_SetDebug(3, debug_msg);
  
  /* Apply heading offset (magnetic declination) if needed */
  heading += heading_offset;
  
  /* Normalize heading to 0-360 degrees */
  while (heading < 0.0f) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;
  
  /* In BNO055, 0 degrees is north, 90 is east, etc. */
  /* This matches the expected behavior for the direction indicator */
  
  /* Store Euler angles in compass data structure */
  compass_data_ptr->heading = heading;
  compass_data_ptr->roll = roll;
  compass_data_ptr->pitch = pitch;
  
  /* Burst-read all raw sensor data: accel(0x08-0x0D) + mag(0x0E-0x13) + gyro(0x14-0x19) = 18 bytes */
  uint8_t raw_buf[18];
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_ACCEL_DATA_X_LSB,
                               I2C_MEMADD_SIZE_8BIT, raw_buf, 18, BNO055_I2C_TIMEOUT);
  if (hal_status == HAL_OK) {
    compass_data_ptr->accel_x = (int16_t)((raw_buf[1]  << 8) | raw_buf[0]);
    compass_data_ptr->accel_y = (int16_t)((raw_buf[3]  << 8) | raw_buf[2]);
    compass_data_ptr->accel_z = (int16_t)((raw_buf[5]  << 8) | raw_buf[4]);
    compass_data_ptr->mag_x   = (int16_t)((raw_buf[7]  << 8) | raw_buf[6]);
    compass_data_ptr->mag_y   = (int16_t)((raw_buf[9]  << 8) | raw_buf[8]);
    compass_data_ptr->mag_z   = (int16_t)((raw_buf[11] << 8) | raw_buf[10]);
    compass_data_ptr->gyro_x  = (int16_t)((raw_buf[13] << 8) | raw_buf[12]);
    compass_data_ptr->gyro_y  = (int16_t)((raw_buf[15] << 8) | raw_buf[14]);
    compass_data_ptr->gyro_z  = (int16_t)((raw_buf[17] << 8) | raw_buf[16]);
    compass_data_ptr->cal_gyro_z = (float)compass_data_ptr->gyro_z / 16.0f; /* 1/16 dps per LSB */
  } else {
    compass_data_ptr->cal_gyro_z = 0.0f;
  }

  static uint32_t last_calib_check = 0;
  if (current_time - last_calib_check > 1000) {
    uint8_t calib_stat = 0;
    if (HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_CALIB_STAT_ADDR, 
                       I2C_MEMADD_SIZE_8BIT, &calib_stat, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
      compass_debug.calib_status = calib_stat;
      
      /* Extract individual calibration statuses */
      uint8_t sys_cal = (calib_stat >> 6) & 0x03;
      uint8_t gyro_cal = (calib_stat >> 4) & 0x03;
      uint8_t accel_cal = (calib_stat >> 2) & 0x03;
      uint8_t mag_cal = calib_stat & 0x03;
      
      /* Update debug message with calibration status */
      snprintf(debug_msg, sizeof(debug_msg), "Cal: S%d G%d A%d M%d", sys_cal, gyro_cal, accel_cal, mag_cal);
      Compass_SetDebug(0, debug_msg);
    }
    last_calib_check = current_time;
  }
  
  /* Check system status and operation mode periodically */
  static uint32_t last_status_check = 0;
  if (current_time - last_status_check > 5000) { /* Every 5 seconds */
    /* Check operation mode */
    uint8_t opr_mode = 0;
    if (HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_OPR_MODE_ADDR, 
                       I2C_MEMADD_SIZE_8BIT, &opr_mode, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
      opr_mode &= 0x0F; /* Mask to get just the operation mode bits */
      if (opr_mode != BNO055_OPR_MODE_NDOF) {
        snprintf(debug_msg, sizeof(debug_msg), "Mode: 0x%02X (not NDOF)", opr_mode);
        Compass_SetError(COMPASS_ERROR_MODE, debug_msg, 0);
        
        /* Try setting mode again - first to CONFIG then to NDOF */
        HAL_I2C_Mem_Write(&hi2c_display, compass_debug.active_addr << 1, BNO055_OPR_MODE_ADDR, 
                         I2C_MEMADD_SIZE_8BIT, (uint8_t[]){BNO055_OPR_MODE_CONFIG}, 1, BNO055_I2C_TIMEOUT);
        HAL_Delay(20);
        HAL_I2C_Mem_Write(&hi2c_display, compass_debug.active_addr << 1, BNO055_OPR_MODE_ADDR, 
                         I2C_MEMADD_SIZE_8BIT, (uint8_t[]){BNO055_OPR_MODE_NDOF}, 1, BNO055_I2C_TIMEOUT);
        HAL_Delay(20);
      } else {
        snprintf(debug_msg, sizeof(debug_msg), "Mode: NDOF (0x%02X)", opr_mode);
        Compass_SetDebug(1, debug_msg);
      }
    }
    
    /* Check system status */
    uint8_t sys_status = 0;
    if (HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_SYS_STAT_ADDR, 
                       I2C_MEMADD_SIZE_8BIT, &sys_status, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
      compass_debug.sys_status = sys_status;
      
      if (sys_status < BNO055_SYS_STATUS_RUNNING) {
        /* System not running in fusion mode - log warning */
        snprintf(debug_msg, sizeof(debug_msg), "Warning: BNO055 status 0x%02X", sys_status);
        Compass_SetDebug(2, debug_msg);
        
        /* Check system error register if there's an error */
        if (sys_status == BNO055_SYS_STATUS_ERROR) {
          uint8_t sys_err = 0;
          if (HAL_I2C_Mem_Read(&hi2c_display, compass_debug.active_addr << 1, BNO055_SYS_ERR_ADDR, 
                             I2C_MEMADD_SIZE_8BIT, &sys_err, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
            snprintf(debug_msg, sizeof(debug_msg), "BNO055 error: 0x%02X", sys_err);
            Compass_SetError(COMPASS_ERROR, debug_msg, 0);
          }
        }
      } else {
        /* System running normally */
        snprintf(debug_msg, sizeof(debug_msg), "BNO055 status: 0x%02X (OK)", sys_status);
        Compass_SetDebug(1, debug_msg);
      }
    }
    last_status_check = current_time;
  }
  
  /* Log sensor values for debugging */
  snprintf(debug_msg, sizeof(debug_msg), "H%.1f P%.1f R%.1f", heading, pitch, roll);
  Compass_SetDebug(1, debug_msg);
  
  snprintf(debug_msg, sizeof(debug_msg), "Mag: X%d Y%d Z%d", 
           compass_data_ptr->mag_x, compass_data_ptr->mag_y, compass_data_ptr->mag_z);
  Compass_SetDebug(2, debug_msg);
  
  /* BNO055 NDOF already performs full 9-axis sensor fusion internally.
   * Use its output directly — no second filter needed or beneficial.
   *
   * Magnetometer calibration check:
   *   mag_cal=0 → heading unreliable (can be 180° wrong in a new environment)
   *   mag_cal=1 → basic calibration, roughly usable
   *   mag_cal=3 → fully calibrated, most accurate
   * Wave the device in a figure-8 pattern after power-up to calibrate. */
  uint8_t mag_cal = compass_debug.calib_status & 0x03;
  compass_data_ptr->heading_valid = (mag_cal >= 1) ? 1 : 0;
  compass_data_ptr->mag_cal = mag_cal;

  /* Store heading in compass data structure */
  heading = compass_data_ptr->heading; /* Store in global for compatibility */
  
  /* Log the heading for debugging */
  snprintf(debug_msg, sizeof(debug_msg), "Heading: %.1f deg, P:%.1f R:%.1f", 
           compass_data_ptr->heading, compass_data_ptr->pitch, compass_data_ptr->roll);
  Compass_SetError(0, debug_msg, 0);
  
  /* Update timestamp */
  compass_data_ptr->timestamp = HAL_GetTick();
  
  return COMPASS_OK;
}

/**
 * @brief Calibrate compass
 * @retval Status code
 */
uint8_t Compass_Calibrate(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t calib_status;
  uint8_t sys_status = 0;
  uint8_t self_test = 0;
  uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
  char debug_msg[100];
  
  /* BNO055 handles calibration internally, but we can check the status */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, 
                               &calib_status, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read calibration status", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Read system status */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_SYS_STAT_ADDR, 
                              I2C_MEMADD_SIZE_8BIT, &sys_status, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read system status", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Read self-test results */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_ST_RESULT_ADDR, 
                              I2C_MEMADD_SIZE_8BIT, &self_test, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read self-test results", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse calibration status */
  sys_cal = (calib_status >> 6) & 0x03;
  gyro_cal = (calib_status >> 4) & 0x03;
  accel_cal = (calib_status >> 2) & 0x03;
  mag_cal = calib_status & 0x03;
  
  /* Store calibration values in debug structure */
  compass_debug.calib_sys = sys_cal;
  compass_debug.calib_gyro = gyro_cal;
  compass_debug.calib_accel = accel_cal;
  compass_debug.calib_mag = mag_cal;
  
  /* Display calibration status */
  snprintf(debug_msg, sizeof(debug_msg), "Calibration: SYS=%d GYR=%d ACC=%d MAG=%d", 
           sys_cal, gyro_cal, accel_cal, mag_cal);
  Compass_SetDebug(0, debug_msg);
  
  /* Display system status */
  const char *status_str = "Unknown";
  switch (sys_status) {
    case 0: status_str = "Idle"; break;
    case 1: status_str = "System Error"; break;
    case 2: status_str = "Initializing Peripherals"; break;
    case 3: status_str = "System Initialization"; break;
    case 4: status_str = "Executing Self-Test"; break;
    case 5: status_str = "Sensor Fusion Running"; break;
    case 6: status_str = "System Running without Fusion"; break;
  }
  
  snprintf(debug_msg, sizeof(debug_msg), "System Status: %s", status_str);
  Compass_SetDebug(1, debug_msg);
  
  /* Display self-test results */
  snprintf(debug_msg, sizeof(debug_msg), "Self-Test: MCU=%d GYR=%d MAG=%d ACC=%d",
           (self_test >> 3) & 0x01, (self_test >> 2) & 0x01, 
           (self_test >> 1) & 0x01, self_test & 0x01);
  Compass_SetDebug(2, debug_msg);
  
  /* Check if fully calibrated */
  if (sys_cal == 3) {
    Compass_SetDebug(3, "System fully calibrated!");
    return COMPASS_OK;
  } else {
    /* Display detailed calibration instructions */
    if (mag_cal < 3) {
      /* Provide more detailed magnetometer calibration instructions */
      switch(mag_cal) {
        case 0:
          snprintf(debug_msg, sizeof(debug_msg), "MAG CAL: Move in figure 8 pattern (0/3)");
          break;
        case 1:
          snprintf(debug_msg, sizeof(debug_msg), "MAG CAL: Continue figure 8 motion (1/3)");
          break;
        case 2:
          snprintf(debug_msg, sizeof(debug_msg), "MAG CAL: Almost done! Keep moving (2/3)");
          break;
      }
      Compass_SetDebug(3, debug_msg);
      
      /* Additional magnetometer calibration guidance */
      snprintf(debug_msg, sizeof(debug_msg), "Rotate device in all directions");
      Compass_SetDebug(4, debug_msg);
      snprintf(debug_msg, sizeof(debug_msg), "Away from metal objects");
      Compass_SetDebug(5, debug_msg);
    } else {
      snprintf(debug_msg, sizeof(debug_msg), "MAG CAL: Complete! (3/3)");
      Compass_SetDebug(3, debug_msg);
      Compass_SetDebug(4, "");
      Compass_SetDebug(5, "");
    }
    
    if (gyro_cal < 3) {
      snprintf(debug_msg, sizeof(debug_msg), "GYR CAL: Keep still (%d/3)", gyro_cal);
      Compass_SetDebug(6, debug_msg);
    } else {
      snprintf(debug_msg, sizeof(debug_msg), "GYR CAL: Complete! (3/3)");
      Compass_SetDebug(6, debug_msg);
    }
    
    if (accel_cal < 3) {
      snprintf(debug_msg, sizeof(debug_msg), "ACC CAL: Place on 6 sides (%d/3)", accel_cal);
      Compass_SetDebug(7, debug_msg);
    } else {
      snprintf(debug_msg, sizeof(debug_msg), "ACC CAL: Complete! (3/3)");
      Compass_SetDebug(7, debug_msg);
    }
    
    return COMPASS_ERROR_CALIB;
  }
}

/**
 * @brief Perform self-test on BNO055
 * @retval Status code
 */
uint8_t Compass_SelfTest(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t sys_status, error_code;
  char debug_msg[100];
  
  /* Read system status register */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_SYS_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, 
                               &sys_status, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read system status", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Store system status in debug structure */
  compass_debug.sys_status = sys_status;
  
  /* Check system status */
  if (sys_status < BNO055_SYS_STATUS_RUNNING) {
    snprintf(debug_msg, sizeof(debug_msg), "System not running: status=0x%02X", sys_status);
    Compass_SetError(COMPASS_ERROR_SYS_STATUS, debug_msg, 0);
    return COMPASS_ERROR_SYS_STATUS;
  }
  
  /* Read error code register */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, 0x3A, I2C_MEMADD_SIZE_8BIT, 
                               &error_code, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read error code", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Check for BNO055 errors */
  if (error_code != 0) {
    snprintf(debug_msg, sizeof(debug_msg), "BNO055 error: code=0x%02X", error_code);
    Compass_SetError(COMPASS_ERROR, debug_msg, 0);
    return COMPASS_ERROR;
  }
  
  /* Read calibration status */
  uint8_t calib_status;
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_CALIB_STAT_ADDR, I2C_MEMADD_SIZE_8BIT, 
                               &calib_status, 1, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read calibration status", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Extract calibration values */
  uint8_t sys_cal = (calib_status & BNO055_CALIB_STAT_SYS_MASK) >> 6;
  uint8_t gyro_cal = (calib_status & BNO055_CALIB_STAT_GYR_MASK) >> 4;
  uint8_t accel_cal = (calib_status & BNO055_CALIB_STAT_ACC_MASK) >> 2;
  uint8_t mag_cal = (calib_status & BNO055_CALIB_STAT_MAG_MASK);
  
  /* Log calibration status */
  snprintf(debug_msg, sizeof(debug_msg), "Cal: S%d G%d A%d M%d", sys_cal, gyro_cal, accel_cal, mag_cal);
  Compass_SetDebug(0, debug_msg);
  
  /* Test reading Euler angles */
  uint8_t data_buffer[6];
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, active_addr << 1, BNO055_EULER_H_LSB, I2C_MEMADD_SIZE_8BIT, 
                               data_buffer, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read Euler angles", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse Euler angles */
  int16_t heading_raw = (int16_t)((data_buffer[1] << 8) | data_buffer[0]);
  int16_t roll_raw = (int16_t)((data_buffer[3] << 8) | data_buffer[2]);
  int16_t pitch_raw = (int16_t)((data_buffer[5] << 8) | data_buffer[4]);
  
  /* Convert to degrees */
  float heading = (float)heading_raw / 16.0f;
  float roll = (float)roll_raw / 16.0f;
  float pitch = (float)pitch_raw / 16.0f;
  
  /* Log Euler angles */
  snprintf(debug_msg, sizeof(debug_msg), "H%.1f P%.1f R%.1f", heading, pitch, roll);
  Compass_SetDebug(1, debug_msg);
  
  return COMPASS_OK;
}

/**
 * @brief Get current heading
 * @retval Heading in degrees (0-360)
 */
float Compass_GetHeading(void)
{
  Compass_Data compass_data;
  
  if (Compass_Update(&compass_data) == 0) {
    return compass_data.heading;
  }
  
  return 0.0f;
}

/**
 * @brief Get calibration status of BNO055 sensors
 * @param sys_cal Pointer to store system calibration status (0-3)
 * @param gyro_cal Pointer to store gyroscope calibration status (0-3)
 * @param accel_cal Pointer to store accelerometer calibration status (0-3)
 * @param mag_cal Pointer to store magnetometer calibration status (0-3)
 * @retval Status code
 */
uint8_t Compass_GetCalibrationStatus(uint8_t *sys_cal, uint8_t *gyro_cal, uint8_t *accel_cal, uint8_t *mag_cal)
{
  /* Read calibration status from debug structure */
  uint8_t calib_status = compass_debug.calib_status;
  
  /* Extract individual calibration values */
  if (sys_cal != NULL) {
    *sys_cal = (calib_status & BNO055_CALIB_STAT_SYS_MASK) >> 6;
  }
  
  if (gyro_cal != NULL) {
    *gyro_cal = (calib_status & BNO055_CALIB_STAT_GYR_MASK) >> 4;
  }
  
  if (accel_cal != NULL) {
    *accel_cal = (calib_status & BNO055_CALIB_STAT_ACC_MASK) >> 2;
  }
  
  if (mag_cal != NULL) {
    *mag_cal = (calib_status & BNO055_CALIB_STAT_MAG_MASK);
  }
  
  return COMPASS_OK;
}

/**
 * @brief Set heading offset (e.g., magnetic declination correction)
 * @param offset_deg Offset in degrees (positive = East declination)
 * @retval Status code
 */
uint8_t Compass_SetHeadingOffset(float offset_deg)
{
  heading_offset = offset_deg;
  return COMPASS_OK;
}

/* Function removed to avoid duplication */

/**
 * @brief Get the last error code
 * @retval Error code
 */
uint8_t Compass_GetLastErrorCode(void)
{
  return compass_error_code;
}

/**
 * @brief Get error message
 * @retval Error message string
 */
const char* Compass_GetErrorMessage(void)
{
  return compass_error_msg;
}

/**
 * @brief Get the last compass error code
 * @retval Error code
 */
uint8_t Compass_GetErrorCode(void)
{
  return compass_error_code;
}

/**
 * @brief Get debug information
 * @param active_addr Pointer to store active I2C address
 * @param raw_bytes Pointer to store raw bytes
 * @param status_reg Pointer to store status register
 * @param calib_status Pointer to store calibration status
 * @retval None
 */
void Compass_GetDebugInfo(uint8_t *active_addr, uint8_t *raw_bytes, uint8_t *status_reg, uint8_t *calib_status)
{
  if (active_addr != NULL) {
    *active_addr = compass_debug.active_addr;
  }
  
  if (raw_bytes != NULL) {
    for (int i = 0; i < 6; i++) {
      raw_bytes[i] = compass_debug.raw_bytes[i];
    }
  }
  
  if (status_reg != NULL) {
    *status_reg = compass_debug.status_reg;
  }
  
  if (calib_status != NULL) {
    *calib_status = compass_debug.calib_status;
  }
}

/**
 * @brief Scan I2C bus and display results on the small display
 * @retval Number of devices found
 */
uint8_t Compass_DisplayI2CScan(void)
{
  uint8_t detected_addrs[16] = {0};
  uint8_t count;
  char line1[22];
  char line2[22];
  char line3[22];
  char debug_msg[100]; /* Added debug message buffer */
  
  /* Clear display */
  Display_Clear();
  Display_DrawText(0, 0, "I2C Bus Scan");
  Display_DrawText(0, 15, "Scanning...");
  Display_Update();
  
  /* Scan I2C bus */
  count = Compass_ScanI2C(detected_addrs, 16);
  
  /* Display results */
  Display_Clear();
  
  if (count == 0) {
    Display_DrawText(0, 0, "I2C Bus Scan");
    Display_DrawText(0, 15, "No devices found");
    Display_DrawText(0, 30, "Check connections");
  } else {
    snprintf(line1, sizeof(line1), "Found %d device(s)", count);
    Display_DrawText(0, 0, "I2C Bus Scan");
    Display_DrawText(0, 15, line1);
    
    /* Display up to 8 addresses, 4 per line */
    if (count > 0) {
      line2[0] = '\0';
      line3[0] = '\0';
      
      for (uint8_t i = 0; i < count && i < 4; i++) {
        char addr_str[8]; /* Increased size to fit "0x" prefix */
        snprintf(addr_str, sizeof(addr_str), "0x%02X ", detected_addrs[i]);
        strcat(line2, addr_str);
      }
      
      for (uint8_t i = 4; i < count && i < 8; i++) {
        char addr_str[8]; /* Increased size to fit "0x" prefix */
        snprintf(addr_str, sizeof(addr_str), "0x%02X ", detected_addrs[i]);
        strcat(line3, addr_str);
      }
      
      /* Debug - print detected addresses */
      snprintf(debug_msg, sizeof(debug_msg), "Addresses: %02X %02X %02X %02X", 
               detected_addrs[0], detected_addrs[1], 
               (count > 2) ? detected_addrs[2] : 0, 
               (count > 3) ? detected_addrs[3] : 0);
      Compass_SetError(0, debug_msg, 0);
      
      Display_DrawText(0, 30, line2);
      if (count > 4) {
        Display_DrawText(0, 45, line3);
      }
    }
  }
  
  Display_Update();
  return count;
}

/**
 * @brief Scan the I2C bus for connected devices
 * This function attempts to communicate with all possible I2C addresses
 * and records which addresses respond
 */
#ifdef DEBUG_COMPASS
static void Compass_ScanI2C(void)
{
  HAL_StatusTypeDef status;
  uint8_t address;
  compass_debug.found_device_count = 0;
  
  /* Clear the address array */
  memset(found_i2c_addresses, 0, sizeof(found_i2c_addresses));
  
  /* Scan all possible 7-bit I2C addresses (0-127) */
  for (address = 1; address < 128; address++) {
    /* Use HAL_I2C_IsDeviceReady which is specifically designed for device detection */
    status = HAL_I2C_IsDeviceReady(&hi2c_display, (uint16_t)(address << 1), 2, 10);
    
    /* If the device acknowledges, record its address */
    if (status == HAL_OK) {
      found_i2c_addresses[compass_debug.found_device_count++] = address;
      
      /* Append to error message for debugging */
      if (compass_debug.found_device_count <= 5) { /* Limit to first 5 devices to avoid buffer overflow */
        char temp[20];
        snprintf(temp, sizeof(temp), " 0x%02X", address);
        strncat(error_message, temp, sizeof(error_message) - strlen(error_message) - 1);
      }
    }
  }
  
  /* Update error message with scan results */
  if (compass_debug.found_device_count > 0) {
    char temp[40];
    snprintf(temp, sizeof(temp), "I2C scan: %d device(s):", compass_debug.found_device_count);
    strncpy(error_message, temp, sizeof(error_message) - 1);
    
    /* Make this message visible by setting it as the main error */
    last_error_code = COMPASS_ERROR_NONE;
  } else {
    strncpy(error_message, "I2C scan: No devices!", sizeof(error_message) - 1);
    Compass_SetError(COMPASS_ERROR_I2C_INIT, error_message, HAL_OK);
  }
}
#endif

/**
 * @brief Get the number of I2C devices found during scan
 * @retval Number of devices
 */
uint8_t Compass_GetFoundDeviceCount(void)
{
  return compass_debug.found_device_count;
}

/**
 * @brief Get the array of found I2C addresses
 * @return Pointer to the array of found addresses
 */
const uint8_t* Compass_GetFoundAddresses(void)
{
  return found_i2c_addresses;
}

/* I2C initialization is now handled by the display module */

/* Function removed to avoid duplication */

/* This function was a duplicate of the one defined earlier */

/**
 * @brief Read raw data from the compass
 * @retval Status code
 */
static uint8_t Compass_ReadData(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t data[6]; /* Buffer for reading sensor data */
  char debug_msg[100];
  
  /* Use the active address from global variable */
  uint8_t addr = compass_debug.active_addr;
  
  /* Read accelerometer data */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, addr << 1, BNO055_ACCEL_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, data, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read accelerometer data", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse accelerometer data (LSB first in BNO055) */
  int16_t accel_x = (int16_t)((data[1] << 8) | data[0]);
  int16_t accel_y = (int16_t)((data[3] << 8) | data[2]);
  int16_t accel_z = (int16_t)((data[5] << 8) | data[4]);
  
  /* Store in debug structure */
  compass_debug.accel_data[0] = accel_x;
  compass_debug.accel_data[1] = accel_y;
  compass_debug.accel_data[2] = accel_z;
  
  /* Read gyroscope data */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, addr << 1, BNO055_GYRO_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, data, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read gyroscope data", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse gyroscope data */
  int16_t gyro_x = (int16_t)((data[1] << 8) | data[0]);
  int16_t gyro_y = (int16_t)((data[3] << 8) | data[2]);
  int16_t gyro_z = (int16_t)((data[5] << 8) | data[4]);
  
  /* Store in debug structure */
  compass_debug.gyro_data[0] = gyro_x;
  compass_debug.gyro_data[1] = gyro_y;
  compass_debug.gyro_data[2] = gyro_z;
  
  /* Read magnetometer data */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, addr << 1, BNO055_MAG_DATA_X_LSB, 
                               I2C_MEMADD_SIZE_8BIT, data, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read magnetometer data", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse magnetometer data */
  int16_t mag_x = (int16_t)((data[1] << 8) | data[0]);
  int16_t mag_y = (int16_t)((data[3] << 8) | data[2]);
  int16_t mag_z = (int16_t)((data[5] << 8) | data[4]);
  
  /* Store in debug structure */
  compass_debug.mag_data[0] = mag_x;
  compass_debug.mag_data[1] = mag_y;
  compass_debug.mag_data[2] = mag_z;
  
  /* Read Euler angles */
  hal_status = HAL_I2C_Mem_Read(&hi2c_display, addr << 1, BNO055_EULER_H_LSB, 
                               I2C_MEMADD_SIZE_8BIT, data, 6, BNO055_I2C_TIMEOUT);
  if (hal_status != HAL_OK) {
    Compass_SetError(COMPASS_ERROR_READ_REG, "Failed to read Euler angles", hal_status);
    return COMPASS_ERROR;
  }
  
  /* Parse Euler angles (heading, roll, pitch) */
  int16_t heading = (int16_t)((data[1] << 8) | data[0]);
  int16_t roll = (int16_t)((data[3] << 8) | data[2]);
  int16_t pitch = (int16_t)((data[5] << 8) | data[4]);
  
  /* Store in debug structure */
  compass_debug.heading = heading;
  compass_debug.roll = roll;
  compass_debug.pitch = pitch;
  
  /* Log sensor values in debug messages */
  snprintf(debug_msg, sizeof(debug_msg), "Acc: X%d Y%d Z%d", accel_x, accel_y, accel_z);
  Compass_SetDebug(0, debug_msg);
  
  snprintf(debug_msg, sizeof(debug_msg), "Mag: X%d Y%d Z%d", mag_x, mag_y, mag_z);
  Compass_SetDebug(1, debug_msg);
  
  snprintf(debug_msg, sizeof(debug_msg), "Gyro: X%d Y%d Z%d", gyro_x, gyro_y, gyro_z);
  Compass_SetDebug(2, debug_msg);
  
  /* Store raw data for heading calculation */
  raw_data[0] = mag_x;
  raw_data[1] = mag_y;
  raw_data[2] = mag_z;
  
  return COMPASS_OK;
}

/**
 * @brief Get system status, self-test result, and system error
 * @param system_status Pointer to store system status
 * @param self_test_result Pointer to store self-test result
 * @param system_error Pointer to store system error
 * @retval Status code
 */
uint8_t Compass_GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  /* Read system status register */
  if (Compass_ReadRegister(BNO055_SYS_STAT_ADDR, system_status) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  /* Read self-test result register */
  if (Compass_ReadRegister(BNO055_ST_RESULT_ADDR, self_test_result) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  /* Read system error register */
  if (Compass_ReadRegister(BNO055_SYS_ERR_ADDR, system_error) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  return COMPASS_OK;
}

/**
 * @brief Get operation mode
 * @param op_mode Pointer to store operation mode
 * @retval Status code
 */
uint8_t Compass_GetOperationMode(uint8_t *op_mode)
{
  /* Read operation mode register */
  if (Compass_ReadRegister(BNO055_OPR_MODE_ADDR, op_mode) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  return COMPASS_OK;
}

/**
 * @brief Get power mode
 * @param pwr_mode Pointer to store power mode
 * @retval Status code
 */
uint8_t Compass_GetPowerMode(uint8_t *pwr_mode)
{
  /* Read power mode register */
  if (Compass_ReadRegister(BNO055_PWR_MODE_ADDR, pwr_mode) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  return COMPASS_OK;
}

/**
 * @brief Get temperature
 * @param temp Pointer to store temperature
 * @retval Status code
 */
uint8_t Compass_GetTemperature(int8_t *temp)
{
  uint8_t temp_data;
  
  /* Read temperature register */
  if (Compass_ReadRegister(0x34, &temp_data) != COMPASS_OK) {
    return COMPASS_ERROR;
  }
  
  /* Convert to signed temperature */
  *temp = (int8_t)temp_data;
  
  return COMPASS_OK;
}


