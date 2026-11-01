/**
 * @file jogpad.c
 * @brief NULLLAB/emakefun Mini I2C Jogpad driver (registers at 0x5A)
 */

#include "jogpad.h"
#include "i2c_bus.h"

#define JOGPAD_ADDR_8BIT   (0x5A << 1)

/* Register map (emakefun JoystickHandle reference) */
#define JOGPAD_REG_AXES    0x10   /* X, Y */
#define JOGPAD_REG_BTN_BASE 0x20  /* Z, C, A, B, D (this order on the wire) */
#define JOGPAD_BTN_STATE_NONE  8
#define JOGPAD_BTN_STATE_BAD   0xFF

/* Wires to Jogpad_Button index order: [Z, C, A, B, D] -> our enum */
static const uint8_t btn_reg_offset[JOGPAD_BTN_COUNT] = { 2, 3, 1, 4, 0 };

/* Poll at most every 20 ms: the module debounces internally, and the bus is
 * shared with the compass - every transaction here is latency it can't use. */
#define JOGPAD_POLL_MS  20

static uint8_t pad_present = 0;
static uint8_t pad_x = 128;
static uint8_t pad_y = 128;
static uint8_t btn_held[JOGPAD_BTN_COUNT];
static uint32_t last_poll_ms = 0;

uint8_t Jogpad_Init(void)
{
  if (!I2CBus_IsOnline()) {
    pad_present = 0;
    return 0;
  }
  uint8_t probe = 0;
  pad_present = (I2CBus_MemRead(JOGPAD_ADDR_8BIT, JOGPAD_REG_AXES, &probe, 1)
                 == HAL_OK) ? 1 : 0;
  if (pad_present) pad_x = probe;   /* free first axis sample from the probe */
  return pad_present;
}

uint8_t Jogpad_IsPresent(void) { return pad_present; }

void Jogpad_Update(void)
{
  if (!pad_present) return;
  uint32_t now = HAL_GetTick();
  if (now - last_poll_ms < JOGPAD_POLL_MS) return;
  last_poll_ms = now;

  /* Two block reads: axes at 0x10-0x11, button states at 0x20-0x24. A failed
   * read keeps the last known values - one missed poll must not look like a
   * released button (would fire spurious "press finished" events). */
  uint8_t axes[2];
  if (I2CBus_MemRead(JOGPAD_ADDR_8BIT, JOGPAD_REG_AXES, axes, 2) == HAL_OK) {
    pad_x = axes[0];
    pad_y = axes[1];
  }

  uint8_t states[JOGPAD_BTN_COUNT];
  if (I2CBus_MemRead(JOGPAD_ADDR_8BIT, JOGPAD_REG_BTN_BASE,
                     states, JOGPAD_BTN_COUNT) == HAL_OK) {
    for (uint8_t i = 0; i < JOGPAD_BTN_COUNT; i++) {
      uint8_t s = states[btn_reg_offset[i]];
      btn_held[i] = (s != JOGPAD_BTN_STATE_NONE && s != JOGPAD_BTN_STATE_BAD);
    }
  }
}

uint8_t Jogpad_GetX(void) { return pad_x; }
uint8_t Jogpad_GetY(void) { return pad_y; }

uint8_t Jogpad_ButtonHeld(Jogpad_Button b)
{
  if (b >= JOGPAD_BTN_COUNT) return 0;
  return btn_held[b];
}
