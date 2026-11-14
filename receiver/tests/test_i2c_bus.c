/**
 * @file test_i2c_bus.c
 * @brief Host-side unit tests for the shared I2C1 bus manager (i2c_bus.c).
 *
 * The manager is the firmware's wedge-recovery brain: a proven error
 * streak drops the bus 400 -> 100 kHz, a second streak takes it offline,
 * a rate-limited probe loop (one 50 ms probe per 10 s) brings it back,
 * and a 2-minute clean window at the slow rung re-tests the fast rung.
 * Every transition is scripted here through the HAL fake.
 *
 * Build & run:  make -C receiver/tests
 */

#include "test_harness.h"
#include <string.h>
#include "i2c_bus.h"

/* ------------------------------------------------------------------ */
/* HAL fakes                                                           */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef read_result  = HAL_OK;
static HAL_StatusTypeDef init_result  = HAL_OK;
static uint32_t init_calls   = 0;
static uint32_t deinit_calls = 0;
static uint32_t probe_calls  = 0;
static uint16_t probe_addr   = 0;
static uint32_t read_calls   = 0;
static uint8_t  rx1[1];

/* 7-bit addresses that ACK IsDeviceReady; -1 terminates (0 = nobody).
 * ready_only_at_100k gates the ACK on the CURRENT bus clock - models a
 * marginal device that only answers when the bus runs slow. */
static uint8_t ready_addrs[8] = { 0 };
static uint8_t ready_only_at_100k = 0;

static uint8_t addr_ready(uint16_t addr8)
{
    if (ready_only_at_100k && hi2c_display.Init.ClockSpeed != 100000u)
        return 0;
    for (unsigned i = 0; i < sizeof(ready_addrs); i++) {
        if (!ready_addrs[i]) break;
        if ((uint16_t)(ready_addrs[i] << 1) == addr8) return 1;
    }
    return 0;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h)
{
    (void)h; init_calls++;
    return init_result;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h)
{
    (void)h; deinit_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr,
                                   uint16_t mem_addr, uint16_t mem_size,
                                   uint8_t *data, uint16_t size,
                                   uint32_t timeout)
{
    (void)h; (void)addr; (void)mem_addr; (void)mem_size; (void)data;
    (void)size; (void)timeout;
    read_calls++;
    return read_result;
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
                                        uint32_t trials, uint32_t timeout)
{
    (void)h; (void)trials; (void)timeout;
    probe_calls++;
    probe_addr = addr;
    return addr_ready(addr) ? HAL_OK : HAL_ERROR;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a,
                                    uint16_t m, uint16_t ms, uint8_t *d,
                                    uint16_t s, uint32_t t)
{ (void)h;(void)a;(void)m;(void)ms;(void)d;(void)s;(void)t; return HAL_OK; }

/* ------------------------------------------------------------------ */

#include "../firmware/src/i2c_bus.c"

static void reset_all(void)
{
    read_result = init_result = HAL_OK;
    init_calls = deinit_calls = probe_calls = read_calls = 0;
    probe_addr = 0;
    memset(ready_addrs, 0, sizeof(ready_addrs));
    ready_only_at_100k = 0;
    hi2c_display.State = HAL_I2C_STATE_READY;
    /* module statics */
    bus_offline = 0;
    bus_error_streak = 0;
    bus_khz = I2C_BUS_KHZ_DEFAULT;
    bus_slow_since_ms = 0;
    bus_last_probe_ms = 0;
}

/* ------------------------------------------------------------------ */
/* Init behavior                                                       */
/* ------------------------------------------------------------------ */

TEST(init_fast_probe_ok_stays_400k)
{
    reset_all();
    ready_addrs[0] = 0x5A;              /* jogpad ACKs */
    I2CBus_Init();
    CHECK(I2CBus_IsOnline() == 1);
    CHECK(bus_khz == 400);
}

TEST(init_fast_fail_falls_back_to_100k)
{
    reset_all();
    /* nobody answers at 400k; the BNO085 ACKs once the bus slows */
    probe_calls = 0;
    I2CBus_Init();
    CHECK(bus_khz == 100 || I2CBus_IsOnline() == 0);
    /* with nobody answering at all the bus is offline at the slow rung */
    CHECK(I2CBus_IsOnline() == 0);
    CHECK(bus_khz == 100);
}

TEST(init_slow_probe_ok_online_100k)
{
    reset_all();
    /* Device ACKs only when the bus actually runs at 100 kHz - models a
     * marginal slave the fast edge rate leaves behind. */
    ready_addrs[0] = 0x5A;
    ready_only_at_100k = 1;
    I2CBus_Init();
    CHECK(I2CBus_IsOnline() == 1);
    CHECK(bus_khz == 100);
}

TEST(init_hal_failure_goes_offline_not_hang)
{
    reset_all();
    init_result = HAL_ERROR;
    I2CBus_Init();
    CHECK(I2CBus_IsOnline() == 0);
}

TEST(init_probe_ok_on_bno085_addresses)
{
    /* bus_probe_ok short-circuits on the jogpad (0x5A) - a bus where only
     * the compass answers exercises the BNO085 primary and alt probes. */
    reset_all();
    ready_addrs[0] = 0x28;              /* BNO085 primary only */
    I2CBus_Init();
    CHECK(I2CBus_IsOnline() == 1);

    reset_all();
    ready_addrs[0] = 0x29;              /* BNO085 alt only */
    I2CBus_Init();
    CHECK(I2CBus_IsOnline() == 1);
}

/* ------------------------------------------------------------------ */
/* Error streak -> slow -> offline -> probe recovery                   */
/* ------------------------------------------------------------------ */

TEST(error_streak_drops_bus_to_100k)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();
    CHECK(bus_khz == 400);

    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 100);    /* first streak = slow rung */
    CHECK(I2CBus_IsOnline() == 1);      /* not offline yet */

    /* A clean transaction resets the streak */
    read_result = HAL_OK;
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 100);    /* still slow */
    CHECK(I2CBus_IsOnline() == 0);      /* second streak = offline */
}

TEST(offline_probe_recovers_when_bus_acks)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();

    /* Drive it offline */
    read_result = HAL_ERROR;
    for (int i = 0; i < 64; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(I2CBus_IsOnline() == 0);

    /* Calls while offline short-circuit (no bus traffic), and probes are
     * rate-limited to one per 10 s. */
    read_calls = 0;
    probe_calls = 0;
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(read_calls == 0);               /* offline: no bus transaction */
    uint32_t probes_first = probe_calls;
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(probe_calls == probes_first); /* within 10 s: no second probe */

    /* Still answering 10 s later -> recovered, streak cleared */
    Test_SetTick(HAL_GetTick() + 10001);
    ready_addrs[0] = 0x5A;
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(probe_calls == probes_first + 1);
}

TEST(wedged_state_reinits_before_probe)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();
    read_result = HAL_ERROR;
    for (int i = 0; i < 64; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(I2CBus_IsOnline() == 0);

    /* A wedge leaves hi2c->State stuck BUSY: the recovery probe must
     * DeInit+Init first, or the probe would return HAL_BUSY forever. */
    hi2c_display.State = 0x24;          /* HAL_I2C_STATE_BUSY_TX-ish  */
    uint32_t dinits = deinit_calls;
    uint32_t inits  = init_calls;
    Test_SetTick(HAL_GetTick() + 11000);
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(deinit_calls == dinits + 1);
    CHECK(init_calls == inits + 1);
    CHECK(I2CBus_IsOnline() == 1);      /* recovered via the jogpad ACK */
}

/* ------------------------------------------------------------------ */
/* Hesitant ramp-up                                                    */
/* ------------------------------------------------------------------ */

TEST(slow_bus_ramps_up_after_two_clean_minutes)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();

    /* Drop to the slow rung */
    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 100);
    read_result = HAL_OK;

    /* 119 s of quiet: still slow */
    Test_SetTick(HAL_GetTick() + 119000);
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 100);

    /* past the 2-minute window: one probe, fast rung restored */
    Test_SetTick(HAL_GetTick() + 2000);
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 400);
}

TEST(rampup_probe_failure_stays_slow)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();
    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    read_result = HAL_OK;
    CHECK(bus_khz == 100);

    /* Two quiet minutes, but the fast-rung probe NACKs: stay slow and
     * restart the quiet window instead of ping-ponging. */
    memset(ready_addrs, 0, sizeof(ready_addrs));
    Test_SetTick(HAL_GetTick() + 121000);
    I2CBus_MemRead(0x5A << 1, 0x00, rx1, 1);
    CHECK(bus_khz == 100);
}

/* ------------------------------------------------------------------ */
/* MemRead: same offline guard + streak bookkeeping as Transmit         */
/* ------------------------------------------------------------------ */

TEST(memread_online_passes_through_and_offline_short_circuits)
{
    reset_all();
    ready_addrs[0] = 0x5A;
    I2CBus_Init();

    uint8_t buf[4] = {0};
    CHECK(I2CBus_MemRead(0x5A << 1, 0x10, buf, sizeof(buf)) == HAL_OK);

    /* A failed MemRead streak feeds the same ladder as Transmit */
    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x10, buf, sizeof(buf));
    CHECK(bus_khz == 100);
    read_result = HAL_ERROR;
    for (int i = 0; i < 32; i++)
        I2CBus_MemRead(0x5A << 1, 0x10, buf, sizeof(buf));
    CHECK(I2CBus_IsOnline() == 0);

    /* Offline: MemRead is refused without bus traffic, and it still
     * runs the rate-limited recovery probe. */
    CHECK(I2CBus_MemRead(0x5A << 1, 0x10, buf, sizeof(buf)) == HAL_ERROR);
    uint32_t probes = probe_calls;
    I2CBus_MemRead(0x5A << 1, 0x10, buf, sizeof(buf));
    CHECK(probe_calls == probes);        /* still inside the 10 s window */
}

int main(void)
{
    run_init_fast_probe_ok_stays_400k();
    run_init_fast_fail_falls_back_to_100k();
    run_init_slow_probe_ok_online_100k();
    run_init_hal_failure_goes_offline_not_hang();
    run_init_probe_ok_on_bno085_addresses();
    run_error_streak_drops_bus_to_100k();
    run_offline_probe_recovers_when_bus_acks();
    run_wedged_state_reinits_before_probe();
    run_slow_bus_ramps_up_after_two_clean_minutes();
    run_rampup_probe_failure_stays_slow();
    run_memread_online_passes_through_and_offline_short_circuits();
    return TEST_SUMMARY();
}
