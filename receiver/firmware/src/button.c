/**
 ******************************************************************************
 * @file    button.c
 * @brief   Button input via the NULLLAB/emakefun I2C jogpad (v2.0 hardware)
 ******************************************************************************
 * Physical buttons are gone: "button 1" (page cycle) is a joystick flick
 * (up OR down - forward-only page model) or button A; "button 2" (context /
 * home) is the joystick click (Z) or button B. The jogpad's onboard MCU does
 * the electrical debouncing, so the vibration-immunity machinery the old
 * EXTI+debounce scheme carried here is replaced by stick hysteresis gating:
 * a direction only engages past 60 counts off-center and releases inside 32,
 * which movement contact and pad knocks can't fake for a full sample.
 ******************************************************************************
 */

#include "button.h"
#include "jogpad.h"

/* Stick deflection thresholds on the 0..255 axis (top = 0 for Y):
 * engage at |deflection| >= 60, release inside 32. */
#define JOG_ENGAGE_DELTA   60
#define JOG_RELEASE_DELTA  32

/* Hold duration that turns "button 1" into a long press (fires while held,
 * matching the old PB10 semantics main.c relies on). */
#define BUTTON_LONG_PRESS_MS 700

typedef enum { STICK_CENTERED = 0, STICK_UP, STICK_DOWN } StickDir;

static uint8_t stick_dir_engaged = STICK_CENTERED;
static uint32_t stick_engaged_ms = 0;

static uint8_t button_press_detected = 0;      /* one-shot: page next */
static uint8_t button_long_press_detected = 0; /* one-shot while held */
static uint8_t long_press_done = 0;            /* this engage already long-fired */

static uint8_t prev_a_held = 0;
static uint8_t prev_b_held = 0;
static uint8_t prev_z_held = 0;
static uint8_t button2_press_detected = 0;

void Button_Init(void)
{
  (void)Jogpad_Init();   /* absent pad is fine - receiver just has no UI input */
  stick_dir_engaged = STICK_CENTERED;
  button_press_detected = 0;
  button_long_press_detected = 0;
  long_press_done = 0;
  prev_a_held = prev_b_held = prev_z_held = 0;
  button2_press_detected = 0;
}

/**
 * @brief Get current "button 1" state (stick flicked up/down, or A held)
 * @retval Button_State_t Current state
 */
Button_State_t Button_GetState(void)
{
  if (stick_dir_engaged != STICK_CENTERED || Jogpad_ButtonHeld(JOGPAD_BTN_A)) {
    return BUTTON_PRESSED;
  }
  return BUTTON_RELEASED;
}

uint8_t Button_IsPressed(void) { return Button_GetState() == BUTTON_PRESSED; }
uint8_t Button_IsReleased(void) { return Button_GetState() == BUTTON_RELEASED; }

/**
 * @brief Poll the pad and turn stick motion / button edges into events.
 * @retval None
 * @note Called from the main loop. The pad itself holds state - no IRQs.
 */
void Button_Update(void)
{
  Jogpad_Update();
  uint32_t now = HAL_GetTick();
  uint8_t y = Jogpad_GetY();

  /* --- Hysteresis gate on the Y axis (up = low counts) --- */
  if (stick_dir_engaged == STICK_CENTERED) {
    if (y <= (128 - JOG_ENGAGE_DELTA)) {
      stick_dir_engaged = STICK_UP;
      stick_engaged_ms = now;
      long_press_done = 0;
      button_press_detected = 1;   /* edge = short press (old: at release) */
    } else if (y >= (128 + JOG_ENGAGE_DELTA)) {
      stick_dir_engaged = STICK_DOWN;
      stick_engaged_ms = now;
      long_press_done = 0;
      button_press_detected = 1;
    }
  } else if (stick_dir_engaged == STICK_UP) {
    if (y >= (128 - JOG_RELEASE_DELTA)) stick_dir_engaged = STICK_CENTERED;
  } else { /* STICK_DOWN */
    if (y <= (128 + JOG_RELEASE_DELTA)) stick_dir_engaged = STICK_CENTERED;
  }

  /* --- Long press: fires while still held, once per engage --- */
  if (stick_dir_engaged != STICK_CENTERED && !long_press_done &&
      (now - stick_engaged_ms) >= BUTTON_LONG_PRESS_MS) {
    long_press_done = 1;
    button_long_press_detected = 1;
  }

  /* --- Physical pad buttons (module is already debounced) --- */
  uint8_t a = Jogpad_ButtonHeld(JOGPAD_BTN_A);
  uint8_t b = Jogpad_ButtonHeld(JOGPAD_BTN_B);
  uint8_t z = Jogpad_ButtonHeld(JOGPAD_BTN_Z);

  if (a && !prev_a_held) button_press_detected = 1;   /* A = page next alias */
  if ((b && !prev_b_held) || (z && !prev_z_held)) {
    button2_press_detected = 1;                        /* B or Z = context/home */
  }
  prev_a_held = a;
  prev_b_held = b;
  prev_z_held = z;
}

/**
 * @brief Page-next event (stick flick up/down or button A)
 * @retval 1 if detected since last call; clears the flag
 */
uint8_t Button_WasPressed(void)
{
  if (button_press_detected) {
    button_press_detected = 0;
    return 1;
  }
  return 0;
}

/**
 * @brief Long-press event (stick held in one direction >= 700 ms)
 * @retval 1 if detected since last call; clears the flag
 */
uint8_t Button_WasLongPressed(void)
{
  if (button_long_press_detected) {
    button_long_press_detected = 0;
    return 1;
  }
  return 0;
}

/**
 * @brief Context/home event (joystick click Z or button B)
 * @retval 1 if detected since last call; clears the flag
 */
uint8_t Button2_WasPressed(void)
{
  if (button2_press_detected) {
    button2_press_detected = 0;
    return 1;
  }
  return 0;
}

/* Diagnostics for the SD breadcrumb paths in main.c: "raw level" now means
 * "context action currently held" (1 = pressed), matching the old polarity
 * where 1 = pressed. */
uint8_t Button2_RawLevel(void) { return prev_z_held || prev_b_held; }
uint8_t Button2_IsHeld(void)   { return prev_z_held || prev_b_held; }
