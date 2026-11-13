/**
 * @file test_jogpad.c
 * @brief Host-side unit tests for the NULLLAB jogpad driver (jogpad.c).
 *
 * The pad is the menu interface on the ground unit. The driver's contract
 * being verified here:
 *   - presence probe at boot (reg 0x10), absent pad never polls
 *   - 20 ms poll throttle (the module debounces; the bus is shared with
 *     the compass and every transaction is latency it can't use)
 *   - wire order [Z,C,A,B,D] remapped to the enum order [A,B,C,D,Z]
 *   - a failed block read keeps the last state (one missed poll must not
 *     look like a button release and fire spurious press events)
 *   - state values NONE(8)/BAD(0xFF) mean "released"
 *
 * Build & run:  make -C receiver/tests
 */

#include "test_harness.h"
#include <string.h>
#include "jogpad.h"
#include "i2c_bus.h"

/* ------------------------------------------------------------------ */
/* I2C bus fakes (jogpad talks through i2c_bus.c, not the HAL)         */
/* ------------------------------------------------------------------ */

static uint8_t bus_online    = 1;
static HAL_StatusTypeDef read_status = HAL_OK;
static uint32_t memread_calls = 0;

/* Virtual registers: 0x10-0x11 axes, 0x20-0x24 button states */
static uint8_t reg_axes[2] = { 128, 128 };
static uint8_t reg_btns[5] = { 8, 8, 8, 8, 8 };   /* JOGPAD_BTN_STATE_NONE */

uint8_t I2CBus_IsOnline(void) { return bus_online; }
uint32_t I2CBus_ClockKHz(void) { return 400; }

HAL_StatusTypeDef I2CBus_MemRead(uint16_t addr8, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    memread_calls++;
    if (addr8 != (0x5A << 1) || read_status != HAL_OK) return read_status;
    if (reg == 0x10 && len <= 2) { memcpy(buf, reg_axes, len); return HAL_OK; }
    if (reg == 0x20 && len <= 5) { memcpy(buf, reg_btns, len); return HAL_OK; }
    return HAL_ERROR;
}
HAL_StatusTypeDef I2CBus_Transmit(uint16_t addr8, const uint8_t *buf,
                                  uint16_t len)
{ (void)addr8; (void)buf; (void)len; return HAL_OK; }

/* ------------------------------------------------------------------ */

#include "../firmware/src/jogpad.c"

static void reset_all(void)
{
    bus_online = 1;
    read_status = HAL_OK;
    memread_calls = 0;
    reg_axes[0] = reg_axes[1] = 128;
    for (int i = 0; i < 5; i++) reg_btns[i] = 8;
    pad_present = 0; pad_x = 128; pad_y = 128; last_poll_ms = 0;
    memset(btn_held, 0, sizeof(btn_held));
}

/* ------------------------------------------------------------------ */

TEST(init_absent_when_bus_offline)
{
    reset_all();
    bus_online = 0;
    CHECK(Jogpad_Init() == 0);
    CHECK(Jogpad_IsPresent() == 0);
    CHECK(memread_calls == 0);          /* no probe attempted at all */
}

TEST(init_absent_when_probe_fails)
{
    reset_all();
    read_status = HAL_ERROR;
    CHECK(Jogpad_Init() == 0);
    CHECK(Jogpad_IsPresent() == 0);
    /* absent pad: Update() is a no-op, axes stay centered */
    Jogpad_Update();
    CHECK(Jogpad_GetX() == 128 && Jogpad_GetY() == 128);
}

TEST(init_present_reads_first_axis_free)
{
    reset_all();
    reg_axes[0] = 200;                  /* probe answer carries X too */
    CHECK(Jogpad_Init() == 1);
    CHECK(Jogpad_IsPresent() == 1);
    CHECK(Jogpad_GetX() == 200);        /* free first axis sample */
}

TEST(update_polls_axes_and_throttles_to_20ms)
{
    reset_all();
    Jogpad_Init();
    memread_calls = 0;

    Test_SetTick(1000);
    reg_axes[0] = 10; reg_axes[1] = 240;
    Jogpad_Update();
    CHECK(Jogpad_GetX() == 10 && Jogpad_GetY() == 240);
    CHECK(memread_calls == 2);          /* axes + buttons block reads */

    /* 15 ms later: still inside the throttle window - no bus traffic */
    Test_SetTick(1015);
    reg_axes[0] = 99;
    Jogpad_Update();
    CHECK(memread_calls == 2);
    CHECK(Jogpad_GetX() == 10);         /* stale, not repolled */

    /* 20 ms boundary: fresh poll */
    Test_SetTick(1020);
    Jogpad_Update();
    CHECK(memread_calls == 4);
    CHECK(Jogpad_GetX() == 99);
}

TEST(button_wire_order_remaps_to_enum)
{
    /* Wire order at 0x20-0x24 is [Z,C,A,B,D]; enum is [A,B,C,D,Z].
     * Set exactly one wire slot held and check it surfaces on the right
     * enum button - the classic swap bug this driver exists to hide. */
    reset_all();
    Jogpad_Init();
    Test_SetTick(100);

    /* wire: [Z,C,A,B,D] = [8, 8, 0, 8, 8] -> only A held (reg 0x22) */
    reg_btns[2] = 0;
    Jogpad_Update();
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_A) == 1);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_B) == 0);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_C) == 0);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_D) == 0);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_Z) == 0);

    /* all five held at once */
    memset(reg_btns, 1, sizeof(reg_btns));
    Test_SetTick(200);
    Jogpad_Update();
    for (int b = 0; b < JOGPAD_BTN_COUNT; b++)
        CHECK(Jogpad_ButtonHeld((Jogpad_Button)b) == 1);
}

TEST(failed_read_keeps_previous_button_state)
{
    /* The recovery killer this guards: a NACK poll mid-press used to
     * report the button released, firing a phantom "press finished"
     * edge in the menu layer. */
    reset_all();
    Jogpad_Init();
    Test_SetTick(100);
    memset(reg_btns, 0, sizeof(reg_btns));      /* all held */
    reg_axes[0] = 42;
    Jogpad_Update();
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_A) == 1);
    CHECK(Jogpad_GetX() == 42);

    read_status = HAL_ERROR;
    memset(reg_btns, 8, sizeof(reg_btns));      /* wire would say NONE */
    reg_axes[0] = 77;
    Test_SetTick(200);
    Jogpad_Update();
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_A) == 1);  /* kept, not released */
    CHECK(Jogpad_GetX() == 42);                   /* axes kept too */
}

TEST(none_and_bad_states_mean_released)
{
    reset_all();
    Jogpad_Init();
    Test_SetTick(100);
    /* 0,1,2,3,4,5,6 = held; 8 = none; 0xFF = invalid */
    reg_btns[0] = 8;      /* Z: none     */
    reg_btns[1] = 0xFF;   /* C: invalid  */
    reg_btns[2] = 6;      /* A: long-hold*/
    reg_btns[3] = 1;      /* B: up event */
    reg_btns[4] = 4;      /* D: dblclick */
    Jogpad_Update();
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_Z) == 0);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_C) == 0);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_A) == 1);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_B) == 1);
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_D) == 1);
    /* out-of-range enum is safe */
    CHECK(Jogpad_ButtonHeld(JOGPAD_BTN_COUNT) == 0);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_init_absent_when_bus_offline();
    run_init_absent_when_probe_fails();
    run_init_present_reads_first_axis_free();
    run_update_polls_axes_and_throttles_to_20ms();
    run_button_wire_order_remaps_to_enum();
    run_failed_read_keeps_previous_button_state();
    run_none_and_bad_states_mean_released();
    return TEST_SUMMARY();
}
