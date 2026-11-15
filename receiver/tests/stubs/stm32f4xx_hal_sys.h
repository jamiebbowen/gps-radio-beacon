/**
 * @file stm32f4xx_hal_sys.h (HOST TEST STUB - system surface)
 * @brief RCC/PWR/FLASH/ADC/IWDG and Cortex-M core types, constants and
 *        register blocks that main.c uses, so it compiles on the host.
 *        Included from the stm32f4xx_hal.h stub.
 *
 * Same rules as stm32f4xx_hal_ext.h: the types only need to typecheck,
 * the macros are no-ops, and the functions are fakes inside the test
 * executable that needs them (test_main.c). The registers main.c reads
 * and writes directly - IWDG, RCC->CSR, the SCB fault status - are
 * test-provided blocks, and the reset-flag and clock-divider values are
 * the reference manual's: main.c's reset-cause decoding depends on them.
 */

#ifndef __STM32F4xx_HAL_SYS_H
#define __STM32F4xx_HAL_SYS_H

#include <stdint.h>

#ifndef __IO
#define __IO volatile
#endif

#define DISABLE 0u
#define ENABLE  1u

/* ---- HAL core ---- */
#define TICK_INT_PRIORITY 0x00u     /* as inc/stm32f4xx_hal_conf.h */
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void              HAL_IncTick(void);

/* ---- RCC / PWR / FLASH ---- */
typedef struct {
    uint32_t PLLState, PLLSource, PLLM, PLLN, PLLP, PLLQ;
} RCC_PLLInitTypeDef;

typedef struct {
    uint32_t OscillatorType, HSEState, LSEState, HSIState;
    uint32_t HSICalibrationValue, LSIState;
    RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider;
} RCC_ClkInitTypeDef;

#define RCC_OSCILLATORTYPE_HSE    0x00000001u
#define RCC_HSE_ON                0x00010000u
#define RCC_PLL_ON                0x00000002u
#define RCC_PLLSOURCE_HSE         0x00400000u
#define RCC_PLLP_DIV2             0x00000002u
#define RCC_CLOCKTYPE_SYSCLK      0x00000001u
#define RCC_CLOCKTYPE_HCLK        0x00000002u
#define RCC_CLOCKTYPE_PCLK1       0x00000004u
#define RCC_CLOCKTYPE_PCLK2       0x00000008u
#define RCC_SYSCLKSOURCE_PLLCLK   0x00000002u
#define RCC_SYSCLK_DIV1           0x00000000u
#define RCC_HCLK_DIV1             0x00000000u
#define RCC_HCLK_DIV2             0x00001000u
#define RCC_HCLK_DIV4             0x00001400u
#define FLASH_LATENCY_1           0x00000001u
#define PWR_REGULATOR_VOLTAGE_SCALE2 0x00008000u

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t flash_latency);

#define __HAL_RCC_PWR_CLK_ENABLE()            do {} while (0)
#define __HAL_PWR_VOLTAGESCALING_CONFIG(scale) ((void)(scale))
#define __HAL_RCC_ADC1_CLK_ENABLE()           do {} while (0)
#define __HAL_RCC_ADC1_CLK_DISABLE()          do {} while (0)

/* ---- ADC ---- */
typedef struct { uint32_t unused; } ADC_TypeDef;
#define ADC1 ((ADC_TypeDef *)0xAD10u)    /* never dereferenced */

typedef struct {
    ADC_TypeDef *Instance;
    struct {
        uint32_t ClockPrescaler, Resolution, DataAlign, ScanConvMode,
                 EOCSelection, ContinuousConvMode, DiscontinuousConvMode,
                 NbrOfConversion, DMAContinuousRequests, ExternalTrigConv,
                 ExternalTrigConvEdge;
    } Init;
} ADC_HandleTypeDef;

typedef struct {
    uint32_t Channel, Rank, SamplingTime, Offset;
} ADC_ChannelConfTypeDef;

#define ADC_CLOCK_SYNC_PCLK_DIV4        0x00010000u
#define ADC_RESOLUTION_12B              0x00000000u
#define ADC_DATAALIGN_RIGHT             0x00000000u
#define ADC_EOC_SINGLE_CONV             0x00000001u
#define ADC_SOFTWARE_START              0x0F000001u
#define ADC_EXTERNALTRIGCONVEDGE_NONE   0x00000000u
#define ADC_CHANNEL_0                   0x00000000u
#define ADC_SAMPLETIME_480CYCLES        0x00000007u

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *cfg);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);

/* ---- Registers main.c pokes directly (test-provided blocks) ---- */
typedef struct { __IO uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
typedef struct { __IO uint32_t CSR; } RCC_TypeDef;          /* CSR only */
typedef struct { __IO uint32_t CFSR, HFSR, BFAR; } SCB_Type; /* fault status */
extern IWDG_TypeDef Test_IWDG;
extern RCC_TypeDef  Test_RCC;
extern SCB_Type     Test_SCB;
#define IWDG (&Test_IWDG)
#define RCC  (&Test_RCC)
#define SCB  (&Test_SCB)

/* RCC_CSR reset flags (RM0368 6.3.21) */
#define RCC_CSR_RMVF      (1u << 24)
#define RCC_CSR_BORRSTF   (1u << 25)
#define RCC_CSR_PINRSTF   (1u << 26)
#define RCC_CSR_PORRSTF   (1u << 27)
#define RCC_CSR_SFTRSTF   (1u << 28)
#define RCC_CSR_IWDGRSTF  (1u << 29)
#define RCC_CSR_WWDGRSTF  (1u << 30)
#define RCC_CSR_LPWRRSTF  (1u << 31)

/* ---- Cortex-M core (CMSIS) ---- */
uint32_t __get_MSP(void);
uint32_t __get_PSP(void);
__attribute__((noreturn)) void NVIC_SystemReset(void);
#define __DSB() do {} while (0)

#endif /* __STM32F4xx_HAL_SYS_H */
