/**
 * @file stm32f4xx_hal_ext.h (HOST TEST STUB - extended surface)
 * @brief GPIO/SPI/DMA/NVIC types and macros needed to compile rf_receiver.c
 *        on the host. Included from the stm32f4xx_hal.h stub.
 *
 * Everything here is inert: the types only need to typecheck, the macros
 * expand to no-ops, and the functions are implemented as fakes inside the
 * test executable (see test_rf_receiver.c). Do NOT add hardware behavior.
 */

#ifndef __STM32F4xx_HAL_EXT_H
#define __STM32F4xx_HAL_EXT_H

#include <stdint.h>

/* ---- Status / GPIO ---- */
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct { uint32_t unused; } GPIO_TypeDef;
/* Distinct fake addresses; the fakes never dereference these */
#define GPIOA ((GPIO_TypeDef *)0xA000u)
#define GPIOB ((GPIO_TypeDef *)0xB000u)
#define GPIOC ((GPIO_TypeDef *)0xC000u)

typedef struct {
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_0   (1u << 0)
#define GPIO_PIN_1   (1u << 1)
#define GPIO_PIN_8   (1u << 8)
#define GPIO_PIN_9   (1u << 9)
#define GPIO_PIN_12  (1u << 12)
#define GPIO_PIN_13  (1u << 13)
#define GPIO_PIN_14  (1u << 14)
#define GPIO_PIN_15  (1u << 15)

#define GPIO_MODE_AF_PP            0u
#define GPIO_NOPULL                0u
#define GPIO_SPEED_FREQ_VERY_HIGH  0u
#define GPIO_AF5_SPI2              0u

void          HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st);

/* ---- DMA ---- */
typedef struct { uint32_t unused; } DMA_Stream_TypeDef;
#define DMA1_Stream3 ((DMA_Stream_TypeDef *)0xD300u)
#define DMA1_Stream4 ((DMA_Stream_TypeDef *)0xD400u)

typedef struct {
    DMA_Stream_TypeDef *Instance;
    struct {
        uint32_t Channel, Direction, PeriphInc, MemInc,
                 PeriphDataAlignment, MemDataAlignment,
                 Mode, Priority, FIFOMode;
    } Init;
} DMA_HandleTypeDef;

#define DMA_CHANNEL_0         0u
#define DMA_MEMORY_TO_PERIPH  0u
#define DMA_PERIPH_TO_MEMORY  1u
#define DMA_PINC_DISABLE      0u
#define DMA_MINC_ENABLE       0u
#define DMA_PDATAALIGN_BYTE   0u
#define DMA_MDATAALIGN_BYTE   0u
#define DMA_NORMAL            0u
#define DMA_PRIORITY_HIGH     0u
#define DMA_PRIORITY_VERY_HIGH 0u
#define DMA_FIFOMODE_DISABLE  0u

HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

/* ---- SPI ---- */
typedef struct { uint32_t unused; } SPI_TypeDef;
#define SPI2 ((SPI_TypeDef *)0x5200u)

typedef struct {
    SPI_TypeDef *Instance;
    struct {
        uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
                 BaudRatePrescaler, FirstBit, TIMode, CRCCalculation,
                 CRCPolynomial;
    } Init;
    DMA_HandleTypeDef *hdmatx;
    DMA_HandleTypeDef *hdmarx;
} SPI_HandleTypeDef;

#define SPI_MODE_MASTER            0u
#define SPI_DIRECTION_2LINES       0u
#define SPI_DATASIZE_8BIT          0u
#define SPI_POLARITY_LOW           0u
#define SPI_PHASE_1EDGE            0u
#define SPI_NSS_SOFT               0u
#define SPI_BAUDRATEPRESCALER_16   0u
#define SPI_FIRSTBIT_MSB           0u
#define SPI_TIMODE_DISABLE         0u
#define SPI_CRCCALCULATION_DISABLE 0u

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);

/* ---- NVIC / RCC / misc ---- */
typedef int IRQn_Type;
#define DMA1_Stream3_IRQn 59
#define DMA1_Stream4_IRQn 60

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t prio, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_Delay(uint32_t ms);

#define __HAL_RCC_DMA1_CLK_ENABLE()   do {} while (0)
#define __HAL_RCC_SPI2_CLK_ENABLE()   do {} while (0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  do {} while (0)
#define __HAL_LINKDMA(handle, field, dma) ((handle)->field = &(dma))

#endif /* __STM32F4xx_HAL_EXT_H */
