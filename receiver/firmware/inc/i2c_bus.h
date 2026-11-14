/**
 * @file i2c_bus.h
 * @brief Shared I2C1 bus manager (BNO055 compass + NULLLAB jogpad)
 *
 * Receiver v2.0: the SSD1309 OLED is gone, so the I2C wedge/speed-ladder
 * logic that used to live in display.c moves here as a general bus service.
 * The handle keeps its old name (hi2c_display) because compass.c already
 * externs it everywhere.
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c_display;  /* legacy name: shared I2C1 handle */

void I2CBus_Init(void);

/* Guarded transactions: track errors, step the clock 400->100 kHz on a
 * proven streak, go "offline" when wedged, and self-recover with a cheap
 * probe every 10 s. Returns HAL status so callers can also log. */
HAL_StatusTypeDef I2CBus_MemRead(uint16_t addr8, uint8_t reg,
                                 uint8_t *buf, uint16_t len);

uint8_t  I2CBus_IsOnline(void);

#endif /* I2C_BUS_H */
