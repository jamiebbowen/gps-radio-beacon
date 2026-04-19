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
#define BUTTON_PIN GPIO_PIN_10  /* PB10 - Mode button */
#define BUTTON_GPIO_PORT GPIOB

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
uint8_t Button_WasPressed(void);  /* Debounced press detection */
void Button_Update(void);         /* Call this regularly for debouncing */

#endif /* BUTTON_H */
