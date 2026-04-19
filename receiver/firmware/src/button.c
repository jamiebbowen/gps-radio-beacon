/**
 ******************************************************************************
 * @file    button.c
 * @brief   Button input handling for STM32F4
 ******************************************************************************
 */

#include "button.h"

/* Private variables */
static uint32_t last_button_time = 0;
static Button_State_t last_stable_state = BUTTON_RELEASED;
static Button_State_t current_raw_state = BUTTON_RELEASED;
static uint8_t button_press_detected = 0;

/* Debounce time in milliseconds */
#define BUTTON_DEBOUNCE_MS 50  /* Typical button bounce is 5-20ms, 50ms is safe */

/**
 * @brief Initialize button GPIO with interrupt
 * @retval None
 */
void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PB12 as input with pull-up and falling edge interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge (button press)
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
    
    /* Enable and set EXTI line interrupt priority */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    /* Initialize state variables */
    last_button_time = HAL_GetTick();
    last_stable_state = BUTTON_RELEASED;
    current_raw_state = BUTTON_RELEASED;
    button_press_detected = 0;
}

/**
 * @brief Get current button state (raw, not debounced)
 * @retval Button_State_t Current button state
 */
Button_State_t Button_GetState(void)
{
    /* Read GPIO pin - button is active LOW with pull-up */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
    
    if (pin_state == GPIO_PIN_RESET) {
        return BUTTON_PRESSED;  /* Pin low = button pressed */
    } else {
        return BUTTON_RELEASED; /* Pin high = button released */
    }
}

/**
 * @brief Check if button is currently pressed (debounced)
 * @retval 1 if pressed, 0 if released
 */
uint8_t Button_IsPressed(void)
{
    return (last_stable_state == BUTTON_PRESSED) ? 1 : 0;
}

/**
 * @brief Check if button is currently released (debounced)
 * @retval 1 if released, 0 if pressed
 */
uint8_t Button_IsReleased(void)
{
    return (last_stable_state == BUTTON_RELEASED) ? 1 : 0;
}

/**
 * @brief Check if button was pressed (one-shot detection)
 * @retval 1 if a press was detected since last call, 0 otherwise
 * @note This function clears the press detection flag when called
 */
uint8_t Button_WasPressed(void)
{
    if (button_press_detected) {
        button_press_detected = 0;  /* Clear flag */
        return 1;
    }
    return 0;
}

/**
 * @brief Update button state with debouncing (handles release detection and interrupt re-enable)
 * @retval None
 * @note Called from main loop. Button presses are interrupt-driven.
 */
void Button_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    Button_State_t raw_state = Button_GetState();
    
    /* Check for button release after debounce time */
    if (raw_state == BUTTON_RELEASED && last_stable_state == BUTTON_PRESSED) {
        if ((current_time - last_button_time) >= BUTTON_DEBOUNCE_MS) {
            last_stable_state = BUTTON_RELEASED;
            
            /* Re-enable interrupt now that debounce period has passed */
            /* Clear any pending interrupts first */
            __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);
            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
    }
}

/**
 * @brief GPIO EXTI interrupt handler for button
 * @retval None
 */
void EXTI15_10_IRQHandler(void)
{
    /* Check if interrupt is from our button pin */
    if (__HAL_GPIO_EXTI_GET_IT(BUTTON_PIN) != RESET) {
        /* Clear interrupt flag FIRST to prevent re-triggering */
        __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);
        
        uint32_t current_time = HAL_GetTick();
        
        /* Simple debouncing - only register press if enough time has elapsed */
        if ((current_time - last_button_time) >= BUTTON_DEBOUNCE_MS) {
            last_button_time = current_time;
            last_stable_state = BUTTON_PRESSED;
            button_press_detected = 1;
            
            /* Disable interrupt temporarily to ignore bounce */
            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        }
    }
}
