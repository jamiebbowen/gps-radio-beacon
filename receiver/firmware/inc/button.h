/**
 ******************************************************************************
 * @file    button.h
 * @brief   Button input handling for STM32F4
 ******************************************************************************
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* v2.0+ hardware: no physical buttons on the MCU at all. This API is a shim
 * over the I2C jogpad (see jogpad.h); the shim lives in button.c.
 *
 * Pin note for the schematic (hardware-receiver.md v2.3): PB2 is wired to
 * the BOOT1 boot-mode button (button to 3.3V, 100k pull-down). This driver
 * must NEVER touch PB2 - it is boot-strap hardware, not application GPIO.
 * (The v1.x Select/Home button that used to live here is now the jogpad's
 * Z click / B button.) */

/* Button states */
typedef enum {
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED = 1
} Button_State_t;

/* Function prototypes */
void Button_Init(void);
Button_State_t Button_GetState(void);
uint8_t Button_IsPressed(void);
uint8_t Button_IsReleased(void);
uint8_t Button_WasPressed(void);      /* Debounced short-press detection (fires at release) */
uint8_t Button_WasLongPressed(void);  /* Long-press detection (fires while held, ~700ms) */
uint8_t Button2_WasPressed(void);     /* Second button press detection (fires on press) */
uint8_t Button2_RawLevel(void);       /* Raw pin level right now (1=high idle, 0=low) */
uint8_t Button2_IsHeld(void);         /* Debounced held state (transition-logging support) */
void Button_Update(void);             /* Call this regularly for debouncing */

#endif /* BUTTON_H */
