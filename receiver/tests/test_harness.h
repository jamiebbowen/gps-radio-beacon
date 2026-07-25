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
uint32_t HAL_GetTick(void) { return fake_tick_ms; }
void Test_SetTick(uint32_t tick_ms) { fake_tick_ms = tick_ms; }

#endif /* TEST_HARNESS_H */
