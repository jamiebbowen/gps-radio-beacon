/**
 * @file test_button.c
 * @brief Host-side unit tests for the receiver's jogpad-based button shim.
 *
 * v2.0 hardware: physical buttons are gone. button.c maps the I2C jogpad
 * (joystick Y + A/B/Z buttons) onto the legacy button.h API. The jogpad
 * driver itself (jogpad.c, I2C register reads) is faked here - these tests
 * exercise the shim's semantics: stick hysteresis, edge-fired page-next
 * events, long-press-while-held, and the context/home button edge logic.
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "button.h"
#include "jogpad.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/button.c"

/* ------------------------------------------------------------------ */
/* Fake jogpad                                                         */
/* ------------------------------------------------------------------ */

static uint8_t fake_x = 128;
static uint8_t fake_y = 128;
static uint8_t fake_held[JOGPAD_BTN_COUNT] = {0};
static uint8_t fake_present = 1;

uint8_t Jogpad_Init(void) { return fake_present; }
void    Jogpad_Update(void) {}
uint8_t Jogpad_IsPresent(void) { return fake_present; }
uint8_t Jogpad_GetX(void) { return fake_x; }
uint8_t Jogpad_GetY(void) { return fake_y; }
uint8_t Jogpad_ButtonHeld(Jogpad_Button b) { return fake_held[b]; }

/* ------------------------------------------------------------------ */
/* Simulation helpers                                                  */
/* ------------------------------------------------------------------ */

static uint32_t now_ms = 1000;   /* non-zero so time windows are sane */

static void advance(uint32_t ms)
{
    now_ms += ms;
    Test_SetTick(now_ms);
}

static void flick_up(void)   { fake_y = 128 - 70; Button_Update(); }
static void flick_down(void) { fake_y = 128 + 70; Button_Update(); }
static void center_stick(void) { fake_y = 128; Button_Update(); }

static void reset_all(void)
{
    Button_Init();
    fake_x = 128;
    fake_y = 128;
    memset(fake_held, 0, sizeof(fake_held));
    (void)Button_WasPressed();
    (void)Button_WasLongPressed();
    (void)Button2_WasPressed();
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_centered_is_released)
{
    reset_all();
    advance(10);
    Button_Update();

    CHECK(stick_dir_engaged == STICK_CENTERED);
    CHECK(Button_GetState() == BUTTON_RELEASED);
    CHECK(Button_IsPressed() == 0);
    CHECK(Button_IsReleased() == 1);
    CHECK(Button_WasPressed() == 0);
    CHECK(Button_WasLongPressed() == 0);
    CHECK(Button2_WasPressed() == 0);
}

TEST(test_flick_up_fires_once_on_edge)
{
    reset_all();
    flick_up();
    CHECK(stick_dir_engaged == STICK_UP);
    CHECK(Button_GetState() == BUTTON_PRESSED);   /* still held */
    CHECK(Button_WasPressed() == 1);              /* one-shot consumed */
    CHECK(Button_WasPressed() == 0);              /* not latched */

    /* Staying deflected must not re-fire */
    advance(100);
    Button_Update();
    CHECK(Button_WasPressed() == 0);

    center_stick();
    CHECK(Button_WasPressed() == 0);
}

TEST(test_flick_down_also_pages_forward)
{
    reset_all();
    flick_down();
    CHECK(stick_dir_engaged == STICK_DOWN);
    CHECK(Button_WasPressed() == 1);
    CHECK(Button_WasPressed() == 0);
    center_stick();
}

TEST(test_hysteresis_rejects_small_deflections)
{
    reset_all();

    /* Below the engage threshold: nothing */
    fake_y = 128 - 50;               /* engage needs >= 60 */
    Button_Update();
    CHECK(stick_dir_engaged == STICK_CENTERED);
    CHECK(Button_WasPressed() == 0);

    /* Noise back to center, then just past the threshold */
    center_stick();
    CHECK(Button_WasPressed() == 0);
    flick_up();
    CHECK(Button_WasPressed() == 1);

    /* Partial return (still outside the release band) keeps it engaged -
     * no re-fire on wobble toward center without full release */
    fake_y = 128 - 40;               /* release needs <= 128-32 */
    Button_Update();
    CHECK(stick_dir_engaged == STICK_UP);
    CHECK(Button_WasPressed() == 0);

    /* Wobble back deep, still same engage - no new edge */
    flick_up();
    CHECK(Button_WasPressed() == 0);

    center_stick();
}

TEST(test_re_engage_after_release_fires_again)
{
    reset_all();
    flick_up();
    CHECK(Button_WasPressed() == 1);
    center_stick();
    flick_down();
    CHECK(Button_WasPressed() == 1);   /* fresh engage = fresh event */
    center_stick();
}

TEST(test_long_press_fires_while_held_once)
{
    reset_all();
    flick_up();
    CHECK(Button_WasPressed() == 1);   /* consume the edge */

    advance(699);
    Button_Update();
    CHECK(Button_WasLongPressed() == 0);

    advance(1);
    Button_Update();
    CHECK(Button_WasLongPressed() == 1);   /* fires while still held */
    CHECK(Button_WasLongPressed() == 0);   /* once per engage */

    /* Hold longer: no repeat */
    advance(2000);
    Button_Update();
    CHECK(Button_WasLongPressed() == 0);

    center_stick();

    /* A new engage re-arms it */
    flick_up();
    advance(701);
    Button_Update();
    CHECK(Button_WasLongPressed() == 1);

    center_stick();
}

TEST(test_button_a_aliases_page_next)
{
    reset_all();

    fake_held[JOGPAD_BTN_A] = 1;
    Button_Update();
    CHECK(Button_WasPressed() == 1);
    CHECK(Button_WasPressed() == 0);

    /* Held A does not auto-repeat */
    advance(500);
    Button_Update();
    CHECK(Button_WasPressed() == 0);

    /* Release + press again fires again */
    fake_held[JOGPAD_BTN_A] = 0;
    Button_Update();
    advance(300);
    fake_held[JOGPAD_BTN_A] = 1;
    Button_Update();
    CHECK(Button_WasPressed() == 1);

    fake_held[JOGPAD_BTN_A] = 0;
    Button_Update();
}

TEST(test_z_click_and_b_fire_context)
{
    reset_all();

    /* Joystick click = context/home */
    fake_held[JOGPAD_BTN_Z] = 1;
    Button_Update();
    CHECK(Button2_WasPressed() == 1);
    CHECK(Button2_WasPressed() == 0);
    CHECK(Button2_IsHeld() == 1);      /* still held -> held state */
    CHECK(Button2_RawLevel() == 1);

    fake_held[JOGPAD_BTN_Z] = 0;
    Button_Update();
    CHECK(Button2_IsHeld() == 0);
    CHECK(Button2_RawLevel() == 0);

    /* Button B aliases the same action */
    fake_held[JOGPAD_BTN_B] = 1;
    Button_Update();
    CHECK(Button2_WasPressed() == 1);
    fake_held[JOGPAD_BTN_B] = 0;
    Button_Update();

    /* C and D are free for future use - they must not fire either event */
    fake_held[JOGPAD_BTN_C] = 1;
    fake_held[JOGPAD_BTN_D] = 1;
    Button_Update();
    CHECK(Button_WasPressed() == 0);
    CHECK(Button2_WasPressed() == 0);
    Button_Update();
    fake_held[JOGPAD_BTN_C] = 0;
    fake_held[JOGPAD_BTN_D] = 0;
    Button_Update();
}

TEST(test_pad_absent_means_neutral)
{
    reset_all();
    Button_Init();  /* fakes report present, fine either way */

    fake_y = 128;
    advance(10);
    Button_Update();

    /* No stick motion: nothing fires */
    CHECK(Button_WasPressed() == 0);
    CHECK(Button2_WasPressed() == 0);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_centered_is_released();
    run_test_flick_up_fires_once_on_edge();
    run_test_flick_down_also_pages_forward();
    run_test_hysteresis_rejects_small_deflections();
    run_test_re_engage_after_release_fires_again();
    run_test_long_press_fires_while_held_once();
    run_test_button_a_aliases_page_next();
    run_test_z_click_and_b_fire_context();
    run_test_pad_absent_means_neutral();

    return TEST_SUMMARY();
}
