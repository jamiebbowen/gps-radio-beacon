/**
 * @file test_button.c
 * @brief Host-side unit tests for the receiver's button driver.
 *
 * Simulates the EXTI interrupt controller and GPIO pins to exercise the
 * full debounce scheme: time-gated press edges, state-guarded release
 * bounce, short-press (fires at release) vs long-press (fires while held,
 * >= 700 ms), and the independent button 2 (fires on press).
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "button.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/button.c"

/* ------------------------------------------------------------------ */
/* Fake hardware: GPIO pins + EXTI pending bits + tick                 */
/* ------------------------------------------------------------------ */

static uint32_t now_ms = 1000;   /* non-zero so debounce windows are sane */

static GPIO_PinState fake_pin10 = GPIO_PIN_SET;   /* released (pull-up) */
static GPIO_PinState fake_pin2  = GPIO_PIN_SET;

static uint32_t exti_pending = 0;   /* bitmask of pending EXTI lines */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port;
    if (pin == BUTTON_PIN)  return fake_pin10;
    if (pin == BUTTON2_PIN) return fake_pin2;
    return GPIO_PIN_SET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    (void)port; (void)pin; (void)st;
}

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t p, uint32_t s)
{
    (void)irq; (void)p; (void)s;
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }

uint32_t Test_EXTI_GetIt(uint32_t pin)  { return exti_pending & pin; }
void     Test_EXTI_ClearIt(uint32_t pin) { exti_pending &= ~pin; }

/* ------------------------------------------------------------------ */
/* Simulation helpers                                                  */
/* ------------------------------------------------------------------ */

static void advance(uint32_t ms)
{
    now_ms += ms;
    Test_SetTick(now_ms);
}

/** Falling edge on button 1: pin goes low + EXTI fires */
static void press_edge(void)
{
    fake_pin10 = GPIO_PIN_RESET;
    exti_pending |= BUTTON_PIN;
    EXTI15_10_IRQHandler();
}

/** Button 1 physical release (pin high; detected by Button_Update) */
static void release_pin(void)
{
    fake_pin10 = GPIO_PIN_SET;
}

/** Falling edge on button 2 */
static void press2_edge(void)
{
    fake_pin2 = GPIO_PIN_RESET;
    exti_pending |= BUTTON2_PIN;
    EXTI2_IRQHandler();
}

static void release2_pin(void)
{
    fake_pin2 = GPIO_PIN_SET;
}

/** Full clean short-press cycle: press, hold, release, settle */
static void do_short_press(uint32_t hold_ms)
{
    press_edge();
    advance(hold_ms);
    Button_Update();
    release_pin();
    advance(1);
    Button_Update();
    advance(BUTTON_DEBOUNCE_MS + 1);   /* leave the release window */
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_and_raw_state)
{
    Button_Init();

    CHECK(Button_GetState() == BUTTON_RELEASED);
    CHECK(Button_IsPressed() == 0);
    CHECK(Button_IsReleased() == 1);
    CHECK(Button_WasPressed() == 0);
    CHECK(Button_WasLongPressed() == 0);
    CHECK(Button2_WasPressed() == 0);

    fake_pin10 = GPIO_PIN_RESET;
    CHECK(Button_GetState() == BUTTON_PRESSED);
    fake_pin10 = GPIO_PIN_SET;
}

TEST(test_short_press_fires_at_release)
{
    Button_Init();
    advance(100);

    press_edge();
    CHECK(Button_IsPressed() == 1);
    CHECK(Button_WasPressed() == 0);        /* not yet - outcome undecided */

    advance(100);                           /* held 100 ms < long threshold */
    Button_Update();
    CHECK(Button_WasPressed() == 0);        /* still held */

    release_pin();
    advance(1);
    Button_Update();

    CHECK(Button_IsReleased() == 1);
    CHECK(Button_WasPressed() == 1);        /* fires exactly once */
    CHECK(Button_WasPressed() == 0);        /* one-shot: cleared */
    CHECK(Button_WasLongPressed() == 0);    /* short, not long */
}

TEST(test_long_press_fires_while_held)
{
    Button_Init();
    advance(100);

    press_edge();
    advance(BUTTON_LONG_PRESS_MS);          /* reach the threshold while held */
    Button_Update();

    CHECK(Button_WasLongPressed() == 1);    /* fires before release */
    CHECK(Button_WasLongPressed() == 0);    /* one-shot */
    CHECK(Button_IsPressed() == 1);         /* still physically held */

    release_pin();
    advance(1);
    Button_Update();

    /* The short-press event must be suppressed after a long press */
    CHECK(Button_WasPressed() == 0);
    CHECK(Button_IsReleased() == 1);
}

TEST(test_press_bounce_rejected_by_time_gate)
{
    Button_Init();
    advance(100);

    press_edge();                    /* real press */
    exti_pending |= BUTTON_PIN;      /* bounce edge 5 ms later */
    advance(5);
    EXTI15_10_IRQHandler();

    advance(100);
    release_pin();
    advance(1);
    Button_Update();

    CHECK(Button_WasPressed() == 1); /* exactly one press despite the bounce */
    CHECK(Button_WasPressed() == 0);
}

TEST(test_release_bounce_rejected_by_state_guard)
{
    Button_Init();
    advance(100);

    press_edge();
    advance(100);
    Button_Update();

    /* Release with bounce: pin flickers low again right at release.
     * The ISR sees a falling edge while the debounced state is still
     * PRESSED - the state guard must reject it. */
    release_pin();
    exti_pending |= BUTTON_PIN;
    fake_pin10 = GPIO_PIN_RESET;     /* bounce: momentarily low again */
    EXTI15_10_IRQHandler();
    fake_pin10 = GPIO_PIN_SET;

    advance(1);
    Button_Update();

    CHECK(Button_WasPressed() == 1); /* single press event */
    CHECK(Button_WasPressed() == 0); /* no phantom second press */
    CHECK(Button_IsReleased() == 1);
}

TEST(test_release_ignored_within_debounce_window)
{
    Button_Init();
    advance(100);

    press_edge();
    /* Pin reads high only 10 ms after the press edge (mechanical chatter):
     * the release must NOT latch until the debounce window has elapsed. */
    release_pin();
    advance(10);
    Button_Update();
    CHECK(Button_IsPressed() == 1);      /* still debouncing */

    advance(BUTTON_DEBOUNCE_MS);
    Button_Update();
    CHECK(Button_IsReleased() == 1);     /* now accepted */
    CHECK(Button_WasPressed() == 1);
}

TEST(test_exti_for_other_line_ignored)
{
    Button_Init();
    advance(100);

    /* Pending bit for a different line on the same EXTI15_10 group */
    exti_pending |= GPIO_PIN_12;
    EXTI15_10_IRQHandler();
    CHECK(Button_IsPressed() == 0);      /* no press latched */

    /* Same for the EXTI2 handler with nothing pending */
    exti_pending = 0;
    EXTI2_IRQHandler();
    CHECK(Button2_WasPressed() == 0);
    exti_pending = 0;
}

TEST(test_button2_fires_on_press_edge)
{
    Button_Init();
    advance(100);

    press2_edge();
    CHECK(Button2_WasPressed() == 1);    /* instant: no release wait */
    CHECK(Button2_WasPressed() == 0);    /* one-shot */

    /* Bounce edge within the debounce window: rejected */
    exti_pending |= BUTTON2_PIN;
    advance(5);
    EXTI2_IRQHandler();
    CHECK(Button2_WasPressed() == 0);

    /* Release and settle */
    release2_pin();
    advance(BUTTON_DEBOUNCE_MS + 1);
    Button_Update();

    /* A second real press is accepted after release + debounce */
    advance(100);
    press2_edge();
    CHECK(Button2_WasPressed() == 1);
    release2_pin();
    advance(BUTTON_DEBOUNCE_MS + 1);
    Button_Update();
}

TEST(test_button2_stuck_pressed_blocks_retrigger)
{
    Button_Init();
    advance(100);

    press2_edge();
    CHECK(Button2_WasPressed() == 1);

    /* Still held: another edge long after debounce must NOT re-fire
     * (state guard - the debounced state is still PRESSED) */
    exti_pending |= BUTTON2_PIN;
    advance(500);
    EXTI2_IRQHandler();
    CHECK(Button2_WasPressed() == 0);

    release2_pin();
    advance(BUTTON_DEBOUNCE_MS + 1);
    Button_Update();
}

TEST(test_rapid_press_sequence)
{
    Button_Init();
    advance(100);

    /* Three clean short presses in a row */
    for (int i = 0; i < 3; i++) {
        do_short_press(100);
        CHECK(Button_WasPressed() == 1);
    }
    CHECK(Button_WasPressed() == 0);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    Test_SetTick(now_ms);

    run_test_init_and_raw_state();
    run_test_short_press_fires_at_release();
    run_test_long_press_fires_while_held();
    run_test_press_bounce_rejected_by_time_gate();
    run_test_release_bounce_rejected_by_state_guard();
    run_test_release_ignored_within_debounce_window();
    run_test_exti_for_other_line_ignored();
    run_test_button2_fires_on_press_edge();
    run_test_button2_stuck_pressed_blocks_retrigger();
    run_test_rapid_press_sequence();

    return TEST_SUMMARY();
}
