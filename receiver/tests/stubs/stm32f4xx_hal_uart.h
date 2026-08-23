/**
 * @file stm32f4xx_hal_uart.h (HOST TEST STUB)
 * @brief Minimal UART type stand-in so gps.h compiles on the host.
 */

#ifndef __STM32F4xx_HAL_UART_H
#define __STM32F4xx_HAL_UART_H

#include <stdint.h>
#include "stm32f4xx_hal_ext.h"  /* HAL_StatusTypeDef */

/* USART peripheral with the one register gps.c touches (CR1 RXNEIE) */
typedef struct {
  uint32_t CR1;
} USART_TypeDef;
#define USART_CR1_RXNEIE (1u << 5)

/* Fake USART2 instance lives in the test executable */
extern USART_TypeDef Test_USART2;
#define USART2 (&Test_USART2)

#define HAL_UART_STATE_READY 0x20u
#define HAL_UART_ERROR_NONE  0x00u

#define UART_WORDLENGTH_8B     0u
#define UART_STOPBITS_1        0u
#define UART_PARITY_NONE       0u
#define UART_MODE_TX_RX        0u
#define UART_HWCONTROL_NONE    0u
#define UART_OVERSAMPLING_16   0u

typedef struct {
  USART_TypeDef *Instance;
  struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode,
             HwFlowCtl, OverSampling;
  } Init;
  uint32_t RxState;    /* HAL_UART_STATE_* */
  uint32_t ErrorCode;  /* HAL_UART_ERROR_* */
} UART_HandleTypeDef;

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                      uint8_t *data, uint16_t size);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* Error-flag clear macros: no-ops on the host */
#define __HAL_UART_CLEAR_PEFLAG(h)  do { (void)(h); } while (0)
#define __HAL_UART_CLEAR_FEFLAG(h)  do { (void)(h); } while (0)
#define __HAL_UART_CLEAR_NEFLAG(h)  do { (void)(h); } while (0)
#define __HAL_UART_CLEAR_OREFLAG(h) do { (void)(h); } while (0)

#endif /* __STM32F4xx_HAL_UART_H */
