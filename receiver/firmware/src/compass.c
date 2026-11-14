/**
 * @file compass.c
 * @brief Compass driver for the Adafruit 4754 (BNO085) I2C fusion IMU.
 *
 * The board carries a BNO085 BREAKOUT (PID 4754), not the BNO055 this file
 * originally targeted. The chips share the I2C address (0x28/0x29) but not
 * the protocol: the BNO085 has no register map - it streams SH-2 reports
 * over SHTP frames (see bno085.c). All register-level access now lives
 * behind that driver; the public API in compass.h is unchanged.
 *
 * Protocol differences that shaped this rewrite vs the BNO055 driver:
 *  - Heading comes from the SH2_ROTATION_VECTOR quaternion directly (there
 *    is no Euler output to glitch near +-90 deg pitch), so the old
 *    "Euler primary, quat locked by search" machinery collapses to a fixed
 *    quat convention (COMPASS_BNO085_DEFAULT_CONV).
 *  - Calibration status is the per-report accuracy field (0-3); there is
 *    no CALIB_STAT register. mag_cal follows the RV accuracy.
 *  - Calibration data cannot be read/written by the host (the BNO085 keeps
 *    its dynamic calibration in-sensor; proper persistence would need the
 *    DCD/FRS path, which this driver deliberately does not implement).
 *    Compass_Get/SetCalibrationData therefore report COMPASS_ERROR and
 *    Compass_HasHostCalOffsets() returns 0 so main.c skips the SD
 *    save/restore dance instead of persisting meaningless bytes.
 */

#include "compass.h"
#include "bno085.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "display.h"

/* Header order in compass.c expects the BNO085 address macros to be the
 * single source of address truth; they live in bno085.h. */
#define COMPASS_I2C_ADDR           BNO085_I2C_ADDR
#define COMPASS_I2C_ADDR_ALT       BNO085_I2C_ADDR_ALT

/* Quaternion->heading convention. The SH-2 rotation vector is defined in
 * an ENU earth frame (X=East, Y=North, Z=Up), so the compass bearing of
 * the device +X axis is atan2(East, North) = 90 deg - atan2(vy, vx) -
 * candidate 5 of the preserved BNO055-era candidate table. The BNO055's
 * ambiguity was datasheet murk; here the frame is specified, so we seed
 * the lock instead of searching (no independent reference exists to
 * search against). If the arrow reads mirrored/rotated in the field,
 * override via SD QUATLOCK with another candidate 1..8. */
#define COMPASS_BNO085_DEFAULT_CONV  5

/* Heading freshness: a poll that succeeds on the wire but yields no RV
 * sample for this long means the fusion stalled (not a bus fault). */
#define COMPASS_RV_STALL_MS  700

/* -----------------------------------------------------------------------
 * Heading correction constants (unchanged from the BNO055 build; the 90 deg
 * mounting term was measured for the old breadboard IMU placement - VERIFY
 * on the new PCB: point North, compare against a phone compass).
 * ----------------------------------------------------------------------- */
#define COMPASS_MAGNETIC_DECLINATION_DEG   8.5f
#define COMPASS_MOUNTING_OFFSET_DEG         90.0f  /* Sensor +X vs antenna direction (bench-derived
                                                    * for BNO055; re-derive for the BNO085 mount). */

/* -----------------------------------------------------------------------
 * Magnetic declination by location (~2025, continental-US grid)
 * (unchanged - pure geography, independent of IMU silicon)
 * ----------------------------------------------------------------------- */
static const float declin_grid[5][13] = {
  /* lon:       -125    -120    -115    -110    -105    -100     -95     -90     -85     -80     -75     -70     -65 */
  /* 25N */ {  +12.6f, +11.6f, +10.4f,  +9.2f,  +7.9f,  +6.5f,  +4.9f,  +3.1f,  +1.1f,  -1.2f,  -3.8f,  -6.9f, -10.3f },
  /* 30N */ {  +12.9f, +12.0f, +10.8f,  +9.6f,  +8.2f,  +6.6f,  +5.0f,  +2.9f,  +0.6f,  -2.0f,  -5.1f,  -8.7f, -12.4f },
  /* 35N */ {  +13.5f, +12.6f, +11.4f, +10.1f,  +8.6f,  +6.9f,  +5.0f,  +2.8f,  +0.4f,  -2.5f,  -5.8f,  -9.5f, -13.4f },
  /* 40N */ {  +14.4f, +13.5f, +12.2f, +10.8f,  +9.1f,  +7.2f,  +5.0f,  +2.6f,   0.0f,  -3.2f,  -6.8f, -10.7f, -14.8f },
  /* 45N */ {  +15.6f, +14.7f, +13.3f, +11.5f,  +9.5f,  +7.3f,  +4.9f,  +2.2f,  -0.7f,  -4.0f,  -7.8f, -12.0f, -16.3f },
};
#define DECLIN_GRID_LON_MIN  (-125.0f)
#define DECLIN_GRID_LON_STEP   (5.0f)
#define DECLIN_GRID_LON_MAX   (-65.0f)
#define DECLIN_GRID_LAT_MIN    (25.0f)
#define DECLIN_GRID_LAT_STEP    (5.0f)
#define DECLIN_GRID_LAT_MAX    (45.0f)

float Compass_DeclinationFromLocation(float lat_deg, float lon_deg)
{
  if (lat_deg < DECLIN_GRID_LAT_MIN) lat_deg = DECLIN_GRID_LAT_MIN;
  if (lat_deg > DECLIN_GRID_LAT_MAX) lat_deg = DECLIN_GRID_LAT_MAX;
  if (lon_deg < DECLIN_GRID_LON_MIN) lon_deg = DECLIN_GRID_LON_MIN;
  if (lon_deg > DECLIN_GRID_LON_MAX) lon_deg = DECLIN_GRID_LON_MAX;

  float fx = (lon_deg - DECLIN_GRID_LON_MIN) / DECLIN_GRID_LON_STEP;
  float fy = (lat_deg - DECLIN_GRID_LAT_MIN) / DECLIN_GRID_LAT_STEP;
  int x0 = (int)fx; if (x0 > 11) x0 = 11;         /* bilinear neighborhood */
  int y0 = (int)fy; if (y0 > 3)  y0 = 3;
  float tx = fx - (float)x0;
  float ty = fy - (float)y0;
  const float a = declin_grid[y0  ][x0],  b = declin_grid[y0  ][x0+1];
  const float c = declin_grid[y0+1][x0],  d = declin_grid[y0+1][x0+1];
  return a*(1.0f-tx)*(1.0f-ty) + b*tx*(1.0f-ty) + c*(1.0f-tx)*ty + d*tx*ty;
}

/* -----------------------------------------------------------------------
 * Rolled over from the BNO055 build: debug plumbing, error plumbing, I2C
 * scan machinery. The register-level helpers are gone (no registers).
 * ----------------------------------------------------------------------- */

static uint32_t debug_timestamp = 0;

extern I2C_HandleTypeDef hi2c_display; /* Shared I2C1 bus (BNO085 + jogpad) */

static uint8_t active_addr = COMPASS_I2C_ADDR;

typedef struct {
  uint8_t active_addr;
  uint8_t calib_status;     /* packed S/G/A/M, same bit layout as before */
  uint8_t calib_sys;
  uint8_t calib_gyro;
  uint8_t calib_accel;
  uint8_t calib_mag;
  uint8_t found_device_count;
  char error_message[64];
  uint8_t error_code;
} Compass_Debug_t;

static Compass_Debug_t compass_debug = {0};
static uint8_t found_i2c_addresses[128] = {0};

static float heading_offset = 0.0f;
/* Latched once the magnetometer reports accuracy >= 1 this power cycle;
 * deliberately survives a Compass_Init() re-init (calibration lives
 * in-sensor and outlasts an I2C recovery). */
static uint8_t compass_cal_proven = 0;

static uint8_t decl_location_applied = 0;


uint8_t Compass_UpdateLocation(float lat_deg, float lon_deg)
{
  if (decl_location_applied) return COMPASS_OK;
  /* Sanity gate matches the GPS parser's: plausible US-like coords only. */
  if (lat_deg < 18.0f || lat_deg > 50.0f || lon_deg < -130.0f || lon_deg > -60.0f)
    return COMPASS_ERROR;
  float decl = Compass_DeclinationFromLocation(lat_deg, lon_deg);
  heading_offset = decl + COMPASS_MOUNTING_OFFSET_DEG;
  decl_location_applied = 1;
  char msg[48];
  snprintf(msg, sizeof(msg), "Site declination: %+.1f deg", (double)decl);
  return COMPASS_OK;
}

/* Kept for API compatibility: always 0 on the BNO085 - the sensor holds
 * its dynamic calibration internally, there are no host-loadable offsets. */

/* Runtime I2C recovery: same ladder as the BNO055 build (streak ->
 * bounded re-init, rate-limited). */
static uint8_t  compass_i2c_error_streak = 0;
static uint32_t compass_last_recover_ms  = 0;
#define COMPASS_ERROR_STREAK_RECOVER   10      /* ~2 s at the 5 Hz poll rate */
#define COMPASS_RECOVER_INTERVAL_MS    5000

/* Quaternion->heading convention lock. On the BNO055 this searched 8
 * candidates against the Euler register; the BNO085 has no Euler output,
 * so the lock is seeded with COMPASS_BNO085_DEFAULT_CONV at init and the
 * SD restore path can still override it for field tuning. */
static int8_t quat_conv_locked = 0;

static void Compass_SetError(uint8_t error_code, const char *error_msg, uint8_t hal_status);

/**
 * @brief Scan I2C bus for devices (unchanged from BNO055 build)
 */
static uint8_t Compass_ScanI2C(uint8_t *detected_addrs, uint8_t max_addrs)
{
  HAL_StatusTypeDef hal_status;
  uint8_t devices_found = 0;

  for (uint8_t addr = 1; addr < 128 && devices_found < max_addrs; addr++) {
    /* Single quick probe per address: a healthy bus answers NACK in
     * microseconds, so retries only multiply the worst case on a wedged
     * bus (127 x 5 x 200 ms = 2+ minutes of boot-time hang, before the
     * IWDG could ever help historically). 1 x 50 ms caps it at ~6.4 s. */
    hal_status = HAL_I2C_IsDeviceReady(&hi2c_display, addr << 1, 1, 50);

    if (hal_status == HAL_OK) {
      detected_addrs[devices_found] = addr;
      devices_found++;

    }
  }

  if (devices_found > 0) {
  }

  compass_debug.found_device_count = devices_found;

  if (devices_found == 0) {
    Compass_SetError(COMPASS_ERROR_I2C_INIT, "No I2C devices found", 0);
    return 0;
  } else {
    return devices_found;
  }
}

static void Compass_SetError(uint8_t error_code, const char *error_msg, uint8_t hal_status)
{
  (void)hal_status;
  const size_t cap = sizeof(compass_debug.error_message) - 1;
  size_t n = strlen(error_msg);
  if (n > cap) n = cap;
  memcpy(compass_debug.error_message, error_msg, n);
  compass_debug.error_message[n] = '\0';
  compass_debug.error_code = error_code;
}

/* -----------------------------------------------------------------------
 * Calibration persistence: NOT SUPPORTED on the BNO085.
 *
 * The BNO055 exposed 22 offset/radius bytes; the BNO085 keeps calibration
 * inside the sensor (SH-2 dynamic calibration; durable persistence would
 * need the DCD/FRS interface). Returning COMPASS_ERROR with no side
 * effects makes the main.c boot/persist paths no-ops instead of writing
 * garbage to SD.
 * ----------------------------------------------------------------------- */

/**
 * @brief Lightweight runtime recovery for a wedged BNO085/bus segment.
 *        Bounded worst case ~250 ms: probe + soft reset + feature reconfig,
 *        no bus scan. Same contract as the BNO055 Compass_Recover.
 */
static uint8_t Compass_Recover(void)
{
  if (BNO085_Recover() != BNO085_OK) {
    return COMPASS_ERROR;
  }
  return COMPASS_OK;
}

/**
 * @brief Normalize degrees to [0, 360)
 */
static float Compass_Normalize360(float deg)
{
  while (deg < 0.0f)     deg += 360.0f;
  while (deg >= 360.0f)  deg -= 360.0f;
  return deg;
}

/**
 * @brief Quaternion->heading candidate by index (1..8) - unchanged table
 *        from the BNO055 build; conv 5 (ENU, atan2(East,North)) is the
 *        BNO085 default per the SH-2 earth frame definition.
 */
static float Compass_QuatHeadingCandidate(int idx, float vx, float vy)
{
  float yaw = atan2f(vy, vx) * (180.0f / (float)M_PI);
  switch (idx) {
    case 1: return Compass_Normalize360(yaw);
    case 2: return Compass_Normalize360(-yaw);
    case 3: return Compass_Normalize360(yaw + 180.0f);
    case 4: return Compass_Normalize360(-yaw + 180.0f);
    case 5: return Compass_Normalize360(90.0f - yaw);
    case 6: return Compass_Normalize360(90.0f + yaw);
    case 7: return Compass_Normalize360(270.0f - yaw);
    default: return Compass_Normalize360(270.0f + yaw);
  }
}

uint8_t Compass_GetQuatLockConv(void)
{
    return (uint8_t)quat_conv_locked;
}

uint8_t Compass_SetQuatLockConv(uint8_t conv)
{
    if (conv < 1 || conv > 8) return COMPASS_ERROR;
    quat_conv_locked = (int8_t)conv;
    return COMPASS_OK;
}

/* Persistent sensor state for the BNO085 layer. */
static BNO085_State s_bno;

/**
 * @brief Initialize the BNO085: find it on the bus, soft-reset it, enable
 *        SH-2 rotation-vector + raw sensor reports.
 * @retval COMPASS_OK when the first fused sample has been received
 */
uint8_t Compass_Init(void)
{
  int retry;

  uint8_t detected_addrs[16] = {0};
  uint8_t device_count = Compass_ScanI2C(detected_addrs, 16);

  if (device_count == 0) {
    Compass_SetError(COMPASS_ERROR, "No I2C devices found", 0);
    return COMPASS_ERROR;
  }


  /* Find the BNO085 on either strap address. The hub answers I2C quickly
   * after power-up, but a fresh boot may still be composing its packet
   * advertisements - that is handled inside the driver handshake. */
  bool device_found = false;
  for (retry = 0; retry < 20 && !device_found; retry++) {
    if (HAL_I2C_IsDeviceReady(&hi2c_display, COMPASS_I2C_ADDR << 1, 1, 50) == HAL_OK) {
      active_addr = COMPASS_I2C_ADDR;
      device_found = true;
    } else if (HAL_I2C_IsDeviceReady(&hi2c_display, COMPASS_I2C_ADDR_ALT << 1, 1, 50) == HAL_OK) {
      active_addr = COMPASS_I2C_ADDR_ALT;
      device_found = true;
    }
    if (!device_found) HAL_Delay(10);
  }

  if (!device_found) {
    Compass_SetError(COMPASS_ERROR_I2C_INIT, "BNO085 not found after timeout", 0);
    return COMPASS_ERROR;
  }

  compass_debug.active_addr = active_addr;

  memset(&s_bno, 0, sizeof(s_bno));

  uint8_t r = BNO085_Init(&hi2c_display, active_addr);
  if (r == BNO085_ERROR) {
    Compass_SetError(COMPASS_ERROR_I2C_INIT, "BNO085 SHTP handshake failed", 0);
    return COMPASS_ERROR;
  }
  if (r == BNO085_TIMEOUT) {
    Compass_SetError(COMPASS_ERROR_ID_CHECK, "BNO085 no fusion data after reset", 0);
    return COMPASS_ERROR;
  }

  /* Fixed convention per the SH-2 ENU frame; SD restore may override for
   * field tuning (a stale BNO055-era QUATLOCK value rotates the arrow -
   * delete it along with the old cal file on first BNO085 boot). */
  quat_conv_locked = COMPASS_BNO085_DEFAULT_CONV;
  decl_location_applied = 0;
  compass_i2c_error_streak = 0;

  /* Seed the live state with whatever arrived during init so the
   * accessors (RV count, system status, cal digits) are real from t=0. */
  (void)BNO085_Poll(&s_bno);

  /* Pack the first accuracy readout into the legacy cal layout. */
  uint8_t mag_cal_f = s_bno.accuracy_mag;
  compass_debug.calib_status = (uint8_t)((mag_cal_f & 0x3) |
                             ((s_bno.accuracy_accel & 0x3) << 2) |
                             ((s_bno.accuracy_gyro  & 0x3) << 4) |
                             ((s_bno.accuracy_rv    & 0x3) << 6));

  heading_offset = COMPASS_MAGNETIC_DECLINATION_DEG + COMPASS_MOUNTING_OFFSET_DEG;

  return COMPASS_OK;
}

/**
 * @brief Update compass data from the BNO085 report stream
 * @param compass_data Pointer to compass data structure
 * @retval Status code
 */
uint8_t Compass_Update(Compass_Data *compass_data_ptr)
{
  uint32_t current_time = HAL_GetTick();
  float dt = 0.0f;

  if (compass_data_ptr->last_update > 0) {
    dt = (float)(current_time - compass_data_ptr->last_update) / 1000.0f;
    if (dt > 1.0f || dt <= 0.0f) {
      dt = 0.01f;
    }
  } else {
    dt = 0.01f;
  }
  compass_data_ptr->last_update = current_time;
  compass_data_ptr->timestamp = current_time;

  const uint8_t poll_ok = (BNO085_Poll(&s_bno) == BNO085_OK);
  const uint8_t stalled = poll_ok &&
      (s_bno.rv_count == 0 ||
       (current_time - s_bno.rv_ms) > COMPASS_RV_STALL_MS);
  if (!poll_ok || stalled) {
    /* Same failure contract as the BNO055 build: keep a trustworthy
     * heading flagged stale, streak -> bounded recovery, rate-limited.
     * Both wire faults AND a wedged (ACKing but silent) hub feed the
     * ladder: the recovery re-init fixes both. */
    compass_data_ptr->heading_stale =
        compass_cal_proven ? 1 : 0;
    compass_data_ptr->heading_valid = 0;
    if (compass_i2c_error_streak < 250) compass_i2c_error_streak++;
    if (compass_i2c_error_streak >= COMPASS_ERROR_STREAK_RECOVER &&
        current_time - compass_last_recover_ms >= COMPASS_RECOVER_INTERVAL_MS) {
      compass_last_recover_ms = current_time;
      if (Compass_Recover() == COMPASS_OK) {
        compass_i2c_error_streak = 0;
        /* The re-init cleared the report enables; let the next poll judge. */
      }
    }
    Compass_SetError(COMPASS_ERROR_READ_DATA,
                     poll_ok ? "BNO085 fusion stalled" : "BNO085 SHTP read failed", 0);
    return COMPASS_ERROR;
  }
  compass_i2c_error_streak = 0;

  /* Quaternion -> axes. Device +X in the earth frame; horizontal parts
   * give the heading, the verticality test gives orientation_valid. */
  const float qw = s_bno.quat_real;
  const float qx = s_bno.quat_i;
  const float qy = s_bno.quat_j;
  const float qz = s_bno.quat_k;

  const float vx = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float vy = 2.0f * (qx * qy + qw * qz);
  const float horiz2 = vx * vx + vy * vy;

  if (horiz2 < 0.04f) {
    /* Device X within ~11 deg of vertical: heading geometrically
     * undefined. Flag VERT instead of CAL (tilt, not calibration). */
    compass_data_ptr->orientation_valid = 0;
  }

  float heading = Compass_QuatHeadingCandidate(quat_conv_locked ? quat_conv_locked
                                                                : COMPASS_BNO085_DEFAULT_CONV,
                                               vx, vy);

  /* ZYX pitch/roll for the attitude readouts. Signs follow the SH-2 ENU
   * frame; screens tolerate sign drift and the arrow math only uses the
   * heading term. */
  float roll  = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy))
                * (180.0f / (float)M_PI);
  float sp    = 2.0f * (qw * qy - qz * qx);
  if (sp >  1.0f) sp =  1.0f;
  if (sp < -1.0f) sp = -1.0f;
  float pitch = asinf(sp) * (180.0f / (float)M_PI);

  heading += heading_offset;
  heading = Compass_Normalize360(heading);

  compass_data_ptr->heading = heading;
  compass_data_ptr->roll = roll;
  compass_data_ptr->pitch = pitch;
  compass_data_ptr->orientation_valid = (horiz2 < 0.04f) ? 0 : 1;

  /* Raw sensor channels (for the diagnostics screens). */
  compass_data_ptr->accel_x = s_bno.accel_raw[0];
  compass_data_ptr->accel_y = s_bno.accel_raw[1];
  compass_data_ptr->accel_z = s_bno.accel_raw[2];
  compass_data_ptr->mag_x   = s_bno.mag_raw[0];
  compass_data_ptr->mag_y   = s_bno.mag_raw[1];
  compass_data_ptr->mag_z   = s_bno.mag_raw[2];
  compass_data_ptr->gyro_x  = s_bno.gyro_raw[0];
  compass_data_ptr->gyro_y  = s_bno.gyro_raw[1];
  compass_data_ptr->gyro_z  = s_bno.gyro_raw[2];
  compass_data_ptr->cal_accel_x = s_bno.accel_ms2[0];
  compass_data_ptr->cal_accel_y = s_bno.accel_ms2[1];
  compass_data_ptr->cal_accel_z = s_bno.accel_ms2[2];
  compass_data_ptr->cal_gyro_x  = s_bno.gyro_rads[0];
  compass_data_ptr->cal_gyro_y  = s_bno.gyro_rads[1];
  compass_data_ptr->cal_gyro_z  = s_bno.gyro_rads[2];

  /* Per-sensor accuracy bits -> legacy S/G/A/M packing. The rotation
   * vector accuracy is the magnetometer-informed fusion confidence, so
   * it takes the "mag" slot AND gates heading_valid; the raw mag channel
   * accuracy is informational only. */
  uint8_t mag_cal  = s_bno.accuracy_rv;
  uint8_t gyro_cal = s_bno.accuracy_gyro;
  uint8_t accel_cal= s_bno.accuracy_accel;
  uint8_t sys_cal  = (uint8_t)((mag_cal < gyro_cal ? mag_cal : gyro_cal)
                               < accel_cal ? (mag_cal < gyro_cal ? mag_cal : gyro_cal)
                                          : accel_cal);
  compass_debug.calib_status = (uint8_t)((sys_cal << 6) | (gyro_cal << 4) |
                                         (accel_cal << 2) | mag_cal);
  compass_debug.calib_sys   = sys_cal;
  compass_debug.calib_gyro  = gyro_cal;
  compass_debug.calib_accel = accel_cal;
  compass_debug.calib_mag   = mag_cal;
  debug_timestamp = current_time;


  /* Latch calibration once PROVEN this session, identical semantics to
   * the BNO055 build: RV accuracy is a live confidence score that decays
   * when the device sits still, while the underlying calibration stays
   * good - so we trust the latch, not the live digit. */
  if (mag_cal >= 1) compass_cal_proven = 1;
  compass_data_ptr->heading_valid =
      (compass_cal_proven
       && compass_data_ptr->orientation_valid) ? 1 : 0;
  compass_data_ptr->heading_stale = 0;  /* live data */
  compass_data_ptr->mag_cal = mag_cal;


  compass_data_ptr->timestamp = HAL_GetTick();

  return COMPASS_OK;
}

uint8_t Compass_GetCalibrationStatus(uint8_t *sys_cal, uint8_t *gyro_cal, uint8_t *accel_cal, uint8_t *mag_cal)
{
  uint8_t calib_status = compass_debug.calib_status;

  if (sys_cal != NULL) {
    *sys_cal = (calib_status >> 6) & 0x03;
  }
  if (gyro_cal != NULL) {
    *gyro_cal = (calib_status >> 4) & 0x03;
  }
  if (accel_cal != NULL) {
    *accel_cal = (calib_status >> 2) & 0x03;
  }
  if (mag_cal != NULL) {
    *mag_cal = calib_status & 0x03;
  }

  return COMPASS_OK;
}

uint8_t Compass_SetDeclination(float decl_deg)
{
  heading_offset = decl_deg + COMPASS_MOUNTING_OFFSET_DEG;
  return COMPASS_OK;
}

const char* Compass_GetErrorMessage(void)
{
  return compass_debug.error_message;
}

uint8_t Compass_GetErrorCode(void)
{
  return compass_debug.error_code;
}

/** Rotation-vector samples received this boot (diagnostics). */
uint32_t Compass_GetRvSampleCount(void)
{
  return s_bno.rv_count;
}

/**
 * @brief Scan I2C bus and display results (unchanged behavior)
 */
uint8_t Compass_DisplayI2CScan(void)
{
  uint8_t detected_addrs[16] = {0};
  uint8_t count;
  char line1[22];
  char line2[22];
  char line3[22];

  Display_Clear();
  Display_DrawText(0, 0, "I2C Bus Scan");
  Display_DrawText(0, 15, "Scanning...");
  Display_Update();

  count = Compass_ScanI2C(detected_addrs, 16);

  Display_Clear();

  if (count == 0) {
    Display_DrawText(0, 0, "I2C Bus Scan");
    Display_DrawText(0, 15, "No devices found");
    Display_DrawText(0, 30, "Check connections");
  } else {
    snprintf(line1, sizeof(line1), "Found %d device(s)", count);
    Display_DrawText(0, 0, "I2C Bus Scan");
    Display_DrawText(0, 15, line1);

    if (count > 0) {
      line2[0] = '\0';
      line3[0] = '\0';

      for (uint8_t i = 0; i < count && i < 4; i++) {
        char addr_str[8];
        snprintf(addr_str, sizeof(addr_str), "0x%02X ", detected_addrs[i]);
        strcat(line2, addr_str);
      }

      for (uint8_t i = 4; i < count && i < 8; i++) {
        char addr_str[8];
        snprintf(addr_str, sizeof(addr_str), "0x%02X ", detected_addrs[i]);
        strcat(line3, addr_str);
      }


      Display_DrawText(0, 30, line2);
      if (count > 4) {
        Display_DrawText(0, 45, line3);
      }
    }
  }

  Display_Update();
  return count;
}

uint8_t Compass_GetFoundDeviceCount(void)
{
  return compass_debug.found_device_count;
}

const uint8_t* Compass_GetFoundAddresses(void)
{
  return found_i2c_addresses;
}

/* -----------------------------------------------------------------------
 * BNO055 register-based diagnostics, re-interpreted for the BNO085 so
 * the existing screens keep compiling and stay meaningful:
 *   system_status : 0x05 while RV fusion is streaming, else 0x03
 *   self_test     : 0x0F when the hub answers + fusion flows, else 0x00
 *   system_error  : last latched compass error code
 *   op_mode       : 0x0C ('fusion on all three sensors') - fixed
 *   pwr_mode      : 0x00 (the BNO085 manages power internally)
 * ----------------------------------------------------------------------- */

uint8_t Compass_GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  uint8_t streaming = (s_bno.rv_count > 0 &&
                       (HAL_GetTick() - s_bno.rv_ms) <= COMPASS_RV_STALL_MS);
  if (system_status != NULL)    *system_status    = streaming ? 0x05 : 0x03;
  if (self_test_result != NULL) *self_test_result = (streaming && BNO085_IsAlive() == BNO085_OK) ? 0x0F : 0x00;
  if (system_error != NULL)     *system_error     = compass_debug.error_code;
  return COMPASS_OK;
}

