/**
 * @file i2c_bus.c
 * @brief Shared I2C1 bus manager with wedge detection and speed ladder
 *
 * Ported from the old SSD1309 display.c: proven error streaks drop the bus
 * from 400 kHz to 100 kHz; a sustained clean window at the slow rung re-tests
 * fast (one shot); going offline is a last resort with a 10 s probe loop.
 * Recovery probing targets the jogpad (0x5A), falling back to the BNO085
 * (0x28/0x29 strap) - any ACK means the bus is healthy again.
 */

#include "i2c_bus.h"

#define I2C_BUS_TX_TIMEOUT_MS   10

/* Bus speed ladder */
#define I2C_BUS_KHZ_DEFAULT     400
#define I2C_BUS_KHZ_SLOW        100
static uint32_t bus_khz = I2C_BUS_KHZ_DEFAULT;
static uint32_t bus_slow_since_ms = 0;      /* entered-slow timestamp */
#define I2C_BUS_RAMPUP_QUIET_MS 120000u     /* 2 min clean -> try fast */

/* Offline/backoff state */
static uint8_t  bus_offline = 0;
static uint32_t bus_error_streak = 0;
static uint32_t bus_last_probe_ms = 0;
#define I2C_BUS_STREAK_LIMIT    32
#define I2C_BUS_PROBE_MS        10000

/* Probe addresses: jogpad then BNO085 (both ADR strap values) */
#define JOGPAD_ADDR_7BIT        0x5A
#define BNO085_ADDR_7BIT        0x28
#define BNO085_ADDR_ALT_7BIT    0x29

I2C_HandleTypeDef hi2c_display; /* legacy global name - shared with compass */

uint8_t I2CBus_IsOnline(void) { return !bus_offline; }
static void bus_set_clock(uint32_t khz)
{
  hi2c_display.Init.ClockSpeed = khz * 1000u;
  (void)HAL_I2C_DeInit(&hi2c_display);
  (void)HAL_I2C_Init(&hi2c_display);
  bus_khz = khz;
}

/** Was anything on the bus still responding at the current clock? */
static uint8_t bus_probe_ok(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c_display, JOGPAD_ADDR_7BIT << 1, 1, 50) == HAL_OK)
    return 1;
  if (HAL_I2C_IsDeviceReady(&hi2c_display, BNO085_ADDR_7BIT << 1, 1, 50) == HAL_OK)
    return 1;
  if (HAL_I2C_IsDeviceReady(&hi2c_display, BNO085_ADDR_ALT_7BIT << 1, 1, 50) == HAL_OK)
    return 1;
  return 0;
}

static void bus_note_result(HAL_StatusTypeDef st)
{
  if (st == HAL_OK) {
    bus_error_streak = 0;
    return;
  }
  if (++bus_error_streak >= I2C_BUS_STREAK_LIMIT) {
    /* First failure point when fast: try slower before giving up. */
    if (bus_khz > I2C_BUS_KHZ_SLOW) {
      bus_set_clock(I2C_BUS_KHZ_SLOW);
      bus_slow_since_ms = HAL_GetTick();
      bus_error_streak = 0;
      return;   /* slower rung comes up; next errors re-streak there */
    }
    /* Already slow (or tried both) and still wedged: go offline */
    bus_offline = 1;
  }
}

/* Called while offline. Rate-limited so a permanently wedged bus costs at
 * most one 50 ms probe per 10 s. Re-inits the HAL peripheral first: a wedge
 * often leaves hi2c->State stuck at BUSY, which would make the probe return
 * HAL_BUSY instantly and never recover. */
static void bus_try_recovery(void)
{
  uint32_t now = HAL_GetTick();
  if (now - bus_last_probe_ms < I2C_BUS_PROBE_MS) return;
  bus_last_probe_ms = now;

  if (hi2c_display.State != HAL_I2C_STATE_READY) {
    (void)HAL_I2C_DeInit(&hi2c_display);
    (void)HAL_I2C_Init(&hi2c_display);
  }
  if (bus_probe_ok()) {
    bus_offline = 0;
    bus_error_streak = 0;
  }
}

/* Hesitant ramp-up: only after a long enough uninterrupted clean run at the
 * slow speed, probe the fast speed once. Any new trouble re-drops us. */
static void bus_service_rampup(void)
{
  if (bus_khz == I2C_BUS_KHZ_SLOW &&
      bus_error_streak == 0 &&
      (HAL_GetTick() - bus_slow_since_ms) >= I2C_BUS_RAMPUP_QUIET_MS) {
    bus_set_clock(I2C_BUS_KHZ_DEFAULT);
    if (!bus_probe_ok()) {
      bus_set_clock(I2C_BUS_KHZ_SLOW);
      bus_slow_since_ms = HAL_GetTick();  /* new quiet window */
    }
  }
}

void I2CBus_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* I2C1 GPIO: PB6 = SCL, PB7 = SDA (external 4.7k pull-ups on the board) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c_display.Instance = I2C1;
  hi2c_display.Init.ClockSpeed = I2C_BUS_KHZ_DEFAULT * 1000u;
  hi2c_display.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c_display.Init.OwnAddress1 = 0;
  hi2c_display.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_display.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c_display.Init.OwnAddress2 = 0;
  hi2c_display.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c_display.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

  if (HAL_I2C_Init(&hi2c_display) != HAL_OK) {
    bus_offline = 1;  /* run degraded - never hang the receiver */
    return;
  }
  bus_khz = I2C_BUS_KHZ_DEFAULT;

  /* Try fast mode first; if nobody answers, fall back to always-good 100k. */
  if (!bus_probe_ok()) {
    bus_set_clock(I2C_BUS_KHZ_SLOW);
    if (!bus_probe_ok()) {
      bus_offline = 1;
      bus_khz = I2C_BUS_KHZ_SLOW;
    }
  }
}

HAL_StatusTypeDef I2CBus_MemRead(uint16_t addr8, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
  if (bus_offline) {
    bus_try_recovery();
    return HAL_ERROR;
  }
  bus_service_rampup();
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(&hi2c_display, addr8, reg, I2C_MEMADD_SIZE_8BIT,
                       buf, len, I2C_BUS_TX_TIMEOUT_MS);
  bus_note_result(st);
  return st;
}
