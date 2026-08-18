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

/* Button configuration */
#define BUTTON_PIN GPIO_PIN_10  /* PB10 - Mode button (next page) */
#define BUTTON_GPIO_PORT GPIOB

/* Second button: context action (select channel on Rocket Select page,
 * jump home to Navigation elsewhere). Active low to GND, internal pull-up.
 * PB2 sits next to PB10 on the Black Pill header. NOTE: PB2 is BOOT1 -
 * ignored on normal boots (BOOT0=0), but it must read LOW when entering the
 * DFU bootloader; if DFU entry is flaky with the button wired, hold this
 * button (forcing PB2 low) while tapping reset. Unwired pin is held high by
 * the pull-up and simply never fires. */
#define BUTTON2_PIN GPIO_PIN_2  /* PB2 - Select/home button */
#define BUTTON2_GPIO_PORT GPIOB

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
void Button_Update(void);             /* Call this regularly for debouncing */

#endif /* BUTTON_H */
