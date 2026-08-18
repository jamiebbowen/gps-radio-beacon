/**
 ******************************************************************************
 * @file    button.c
 * @brief   Button input handling for STM32F4
 ******************************************************************************
 */

#include "button.h"

/* Private variables. ISR-shared variables are volatile: they are written
 * from EXTI15_10_IRQHandler and read/written from the main loop, so the
 * compiler must not cache them in registers across accesses. */
static volatile uint32_t last_button_time = 0;
static volatile Button_State_t last_stable_state = BUTTON_RELEASED;
static volatile uint8_t press_pending = 0;       /* Press seen, short/long undecided */
static volatile uint8_t button_press_detected = 0;       /* Short press (set at release) */
static uint8_t button_long_press_detected = 0;           /* Long press (set while held) */

/* Button 2 (PB2, EXTI2). Fires on press (no long-press semantics), so it
 * only needs the debounce state, not a pending-outcome flag. */
static volatile uint32_t last_button2_time = 0;
static volatile Button_State_t button2_stable_state = BUTTON_RELEASED;
static volatile uint8_t button2_press_detected = 0;

/* Debounce time in milliseconds */
#define BUTTON_DEBOUNCE_MS 50  /* Typical button bounce is 5-20ms, 50ms is safe */

/* Hold duration that turns a press into a long press. A press shorter than
 * this is reported by Button_WasPressed() at release; holding at least this
 * long fires Button_WasLongPressed() immediately (before release) and
 * suppresses the short-press event. */
#define BUTTON_LONG_PRESS_MS 700

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
    
    /* Configure PB2 (button 2) the same way on its own EXTI line */
    GPIO_InitStruct.Pin = BUTTON2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON2_GPIO_PORT, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    
    /* Initialize state variables */
    last_button_time = HAL_GetTick();
    last_stable_state = BUTTON_RELEASED;
    press_pending = 0;
    button_press_detected = 0;
    button_long_press_detected = 0;
    
    last_button2_time = HAL_GetTick();
    button2_stable_state = BUTTON_RELEASED;
    button2_press_detected = 0;
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
 * @brief Check if button was short-pressed (one-shot detection)
 * @retval 1 if a short press was detected since last call, 0 otherwise
 * @note Fires at release for presses shorter than BUTTON_LONG_PRESS_MS.
 *       This function clears the press detection flag when called.
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
 * @brief Check if button was long-pressed (one-shot detection)
 * @retval 1 if a long press was detected since last call, 0 otherwise
 * @note Fires while the button is still held, once the hold reaches
 *       BUTTON_LONG_PRESS_MS. The short-press event is then suppressed.
 */
uint8_t Button_WasLongPressed(void)
{
    if (button_long_press_detected) {
        button_long_press_detected = 0;  /* Clear flag */
        return 1;
    }
    return 0;
}

/**
 * @brief Check if button 2 was pressed (one-shot detection)
 * @retval 1 if a press was detected since last call, 0 otherwise
 * @note Fires on press (falling edge), so the action feels instant.
 *       This function clears the press detection flag when called.
 */
uint8_t Button2_WasPressed(void)
{
    if (button2_press_detected) {
        button2_press_detected = 0;  /* Clear flag */
        return 1;
    }
    return 0;
}

/**
 * @brief Update button state (long-press promotion and release detection)
 * @retval None
 * @note Called from main loop. Button presses are interrupt-driven.
 */
void Button_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    Button_State_t raw_state = Button_GetState();
    
    if (last_stable_state == BUTTON_PRESSED) {
        /* Promote a held press to a long press while still held */
        if (press_pending &&
            (current_time - last_button_time) >= BUTTON_LONG_PRESS_MS) {
            press_pending = 0;
            button_long_press_detected = 1;
        }
        
        /* Check for button release after debounce time */
        if (raw_state == BUTTON_RELEASED &&
            (current_time - last_button_time) >= BUTTON_DEBOUNCE_MS) {
            last_stable_state = BUTTON_RELEASED;
            
            /* Released before the long-press threshold: it's a short press */
            if (press_pending) {
                press_pending = 0;
                button_press_detected = 1;
            }
            
            /* Re-arm the debounce window at the release edge so any release
             * bounce (falling edges) within the next BUTTON_DEBOUNCE_MS is
             * rejected by the time gate in the ISR. */
            last_button_time = current_time;
        }
    }
    
    /* Button 2 release detection (same time-gate/state-guard scheme) */
    if (button2_stable_state == BUTTON_PRESSED &&
        HAL_GPIO_ReadPin(BUTTON2_GPIO_PORT, BUTTON2_PIN) == GPIO_PIN_SET &&
        (current_time - last_button2_time) >= BUTTON_DEBOUNCE_MS) {
        button2_stable_state = BUTTON_RELEASED;
        last_button2_time = current_time;
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
        
        /* Debounce without disabling the IRQ line (disabling it until the
         * main loop confirmed release could drop presses that arrived during
         * long loop iterations, e.g. SD writes or OLED I2C pushes):
         *  - time gate rejects press-bounce edges within BUTTON_DEBOUNCE_MS
         *  - state guard rejects release-bounce edges, which arrive while the
         *    debounced state is still BUTTON_PRESSED */
        if ((current_time - last_button_time) >= BUTTON_DEBOUNCE_MS &&
            last_stable_state == BUTTON_RELEASED) {
            last_button_time = current_time;
            last_stable_state = BUTTON_PRESSED;
            /* Outcome (short vs long) is decided in Button_Update */
            press_pending = 1;
        }
    }
}

/**
 * @brief GPIO EXTI interrupt handler for button 2 (PB2)
 * @retval None
 * @note Same debounce scheme as button 1, but the press event fires here on
 *       the falling edge - button 2 triggers a one-shot context action, so
 *       there is no short/long outcome to wait for.
 */
void EXTI2_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(BUTTON2_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(BUTTON2_PIN);
        
        uint32_t current_time = HAL_GetTick();
        
        if ((current_time - last_button2_time) >= BUTTON_DEBOUNCE_MS &&
            button2_stable_state == BUTTON_RELEASED) {
            last_button2_time = current_time;
            button2_stable_state = BUTTON_PRESSED;
            button2_press_detected = 1;
        }
    }
}
