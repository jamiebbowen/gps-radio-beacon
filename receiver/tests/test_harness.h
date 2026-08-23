/**
 * @file test_harness.h
 * @brief Minimal shared unit-test harness for host-side firmware tests.
 *
 * Provides CHECK / CHECK_NEAR assertion macros, a TEST() declaration helper,
 * and the fake HAL tick used by the stm32f4xx_hal.h stub. Include once per
 * test executable and finish main() with TEST_SUMMARY().
 */

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(cond) do {                                                   \
    tests_run++;                                                           \
    if (!(cond)) {                                                         \
        tests_failed++;                                                    \
        printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                      \
} while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

#define TEST(name) static void name(void); \
    static void run_##name(void) { printf("%s\n", #name); name(); } \
    static void name(void)

/* Returns the process exit code: 0 = all passed. */
#define TEST_SUMMARY() \
    (printf("\n%d checks, %d failed\n", tests_run, tests_failed), \
     tests_failed ? 1 : 0)

/* Fake HAL tick backing the stubs/stm32f4xx_hal.h declarations. Defined
 * here (once per test executable) so every test binary links cleanly. */
static uint32_t fake_tick_ms = 0;
/* On real hardware the systick ISR advances the tick even while code spins
 * in a wait loop making no HAL calls. Tests that exercise such loops set
 * this to the per-call quantum (e.g. 25) so timeouts can fire; default 0
 * keeps the tick fully deterministic for everything else. */
static uint32_t fake_tick_auto_ms = 0;
uint32_t HAL_GetTick(void)
{
    uint32_t t = fake_tick_ms;
    fake_tick_ms += fake_tick_auto_ms;
    return t;
}
void Test_SetTick(uint32_t tick_ms) { fake_tick_ms = tick_ms; }
void Test_SetTickAutoAdvance(uint32_t ms_per_call) { fake_tick_auto_ms = ms_per_call; }

#endif /* TEST_HARNESS_H */
