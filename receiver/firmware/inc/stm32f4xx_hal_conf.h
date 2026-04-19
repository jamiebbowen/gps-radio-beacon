#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED

/* Include the required HAL headers */
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pcd.h"
#include "stm32f4xx_ll_usb.h"

/* Oscillator values (change if needed) */
#define HSE_VALUE    ((uint32_t)25000000)
#define HSI_VALUE    ((uint32_t)16000000)
#define LSI_VALUE    ((uint32_t)32000)
#define LSE_VALUE    ((uint32_t)32768)

/* Timeout values */
#define HSE_STARTUP_TIMEOUT    ((uint32_t)100)   /* Time out for HSE start up, in ms */
#define LSE_STARTUP_TIMEOUT    ((uint32_t)5000)  /* Time out for LSE start up, in ms */

/* Interrupt priority configuration */
#define TICK_INT_PRIORITY            ((uint32_t)0x00) /* Systick interrupt priority - must be highest to prevent HAL_GetTick starvation */

/* Assert macro */
#define assert_param(expr) ((void)0)

#endif /* __STM32F4xx_HAL_CONF_H */
