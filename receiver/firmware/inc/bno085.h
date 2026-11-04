/**
 * @file bno085.h
 * @brief Minimal SHTP-over-I2C driver for the Hillcrest/Bosch BNO085
 *        9-axis fusion coprocessor (Adafruit 4754 breakout).
 *
 * The BNO085 speaks the SH-2 protocol over SHTP framing - there are no
 * memory-mapped registers like on the BNO055. Reports are requested with
 * SH2_SetFeature on the control channel and arrive as input reports on
 * channels 3/4/5. This driver enables only what the compass layer needs:
 *
 *   - SH2_ROTATION_VECTOR            (fused 9-axis quaternion -> heading)
 *   - SH2_ACCELEROMETER              (raw + accuracy)
 *   - SH2_GYROSCOPE_CALIBRATED       (raw + accuracy)
 *   - SH2_MAGNETIC_FIELD_CALIBRATED  (raw + accuracy)
 *
 * INT/RST/PS0/PS1 are unwired on the board, so the driver polls the hub
 * over I2C (the hub queues reports between polls) and relies on the
 * breakout's onboard pull-ups for the reset/boot lines.
 */

#ifndef __BNO085_H
#define __BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BNO085_OK        0
#define BNO085_ERROR     1
#define BNO085_TIMEOUT   2

/* Default 7-bit I2C address (PS0/PS1 both strapped low on the 4754
 * breakout -> I2C mode, ADR low); 0x29 with ADR high. */
#define BNO085_I2C_ADDR      0x28
#define BNO085_I2C_ADDR_ALT  0x29

/* Fixed-point scale factors from the SH-2 spec */
#define BNO085_Q_QUAT   16384.0f   /* rotation vector quaternion: Q14 */
#define BNO085_Q_ACCEL  256.0f     /* m/s^2:  Q8  */
#define BNO085_Q_GYRO   512.0f     /* rad/s:  Q9  */
#define BNO085_Q_MAG    16.0f      /* uT:     Q4  */

/* Latest sample snapshot. All fields update only when the hub delivers
 * the corresponding report; *_ms stamps carry HAL_GetTick() so callers
 * can judge freshness. accuracy_* hold the SH-2 report status bits
 * 0=unreliable 1=low 2=medium 3=high, used as the cal display digits. */
typedef struct {
  float    quat_i, quat_j, quat_k, quat_real; /* rotation vector */
  uint8_t  accuracy_rv;
  uint32_t rv_ms;
  uint32_t rv_count;      /* total RV samples received since init */
  int16_t  accel_raw[3];
  float    accel_ms2[3];
  uint8_t  accuracy_accel;
  uint32_t accel_ms;
  int16_t  gyro_raw[3];
  float    gyro_rads[3];
  uint8_t  accuracy_gyro;
  uint32_t gyro_ms;
  int16_t  mag_raw[3];
  float    mag_ut[3];
  uint8_t  accuracy_mag;
  uint32_t mag_ms;
} BNO085_State;

/**
 * @brief Soft-reset the hub and enable the rotation-vector + raw sensor
 *        reports the compass layer consumes.
 * @param hi2c   Initialized I2C handle (shared display/I2C1 bus)
 * @param addr7  7-bit I2C address (0x28 or 0x29)
 * @retval BNO085_OK on success (first rotation vector sample received)
 */
uint8_t BNO085_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7);

/**
 * @brief Drain all queued SHTP packets and fold them into *st.
 *        Call at the compass poll rate (~5 Hz); the hub buffers reports
 *        between calls because INT is unwired.
 * @retval BNO085_OK      bus transaction(s) succeeded (st may be unchanged
 *                        if no new reports were queued)
 * @retval BNO085_ERROR   bus fault (NACK etc.) - caller drives recovery
 */
uint8_t BNO085_Poll(BNO085_State *st);

/**
 * @brief Probe whether the hub ACKs on the configured I2C address.
 *        Cheap liveness check used by the error-recovery path. */
uint8_t BNO085_IsAlive(void);

/**
 * @brief Soft-reset the hub and re-enable reports without a bus scan.
 *        Bounded worst-case blocking (~250 ms), safe to call from the
 *        runtime recovery path (compass.c) on an error streak. */
uint8_t BNO085_Recover(void);

#ifdef __cplusplus
}
#endif

#endif /* __BNO085_H */
