/**
 * @file stm32f4xx_hal.h (HOST TEST STUB)
 * @brief Minimal stand-in for the STM32 HAL so pure-logic modules
 *        (rf_parser.c) can be compiled and unit-tested on a host PC.
 *
 * Only the symbols actually referenced by the modules under test are
 * declared here. Do NOT add hardware behavior - tests must stay
 * hardware-independent.
 */

#ifndef __STM32F4xx_HAL_H
#define __STM32F4xx_HAL_H

#include <stdint.h>

/* Tick stub - some headers reference HAL_GetTick(). Provide a settable
 * fake so future tests can simulate the passage of time. */
uint32_t HAL_GetTick(void);
void     Test_SetTick(uint32_t tick_ms);

/* GPIO/SPI/DMA/NVIC surface used by rf_receiver.c (types + no-op macros;
 * function fakes live in the test executables that need them). */
#include "stm32f4xx_hal_ext.h"

#endif /* __STM32F4xx_HAL_H */
