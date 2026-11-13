/**
 * @file eink.c
 * @brief SSD1680 2.9" tri-color eInk driver (Adafruit 4778 FeatherWing)
 *
 * Init/window/update sequence is a faithful port of Adafruit_EPD's
 * Adafruit_SSD1680 (default/FPC-A005 panel, OTP waveform LUT, full refresh
 * only - partial refresh LUTs don't exist for this panel).
 *
 * Framebuffer mapping (matches Adafruit's THINKINK_STANDARD rotation-0):
 *   addr = ((WIDTH-1 - x) * HEIGHT + y) / 8      (HEIGHT already 8-aligned)
 *   bit  = 0x80 >> (y % 8)
 *   black RAM (0x24): 1 = white, 0 = black   (black plane is "inverted")
 *   red   RAM (0x26): 1 = red,   0 = not red
 * Data entry mode 0x03, RAM X window 1..HEIGHT/8, RAM Y window 0..WIDTH-1.
 */

#include "eink.h"
#include "main.h"
#include "font.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;  /* shared SPI1 handle (main.c) */

/* SSD1680 command opcodes */
#define SSD1680_DRIVER_CONTROL  0x01
#define SSD1680_GATE_VOLTAGE    0x03
#define SSD1680_SOURCE_VOLTAGE  0x04
#define SSD1680_DEEP_SLEEP      0x10
#define SSD1680_DATA_MODE       0x11
#define SSD1680_SW_RESET        0x12
#define SSD1680_MASTER_ACTIVATE 0x20
#define SSD1680_DISP_CTRL2      0x22
#define SSD1680_WRITE_RAM_BW    0x24
#define SSD1680_WRITE_RAM_RED   0x26
#define SSD1680_WRITE_VCOM      0x2C
#define SSD1680_WRITE_BORDER    0x3C
#define SSD1680_SET_RAMXPOS     0x44
#define SSD1680_SET_RAMYPOS     0x45
#define SSD1680_SET_RAMXCOUNT   0x4E
#define SSD1680_SET_RAMYCOUNT   0x4F

#define EINK_PLANE_SIZE   ((EINK_WIDTH * EINK_HEIGHT) / 8)  /* 4736 */
#define EINK_XRAM_OFFSET  1     /* Adafruit uses colstart = 1 for this panel */
#define EINK_SPI_TIMEOUT_MS 25
#define EINK_REFRESH_TIMEOUT_MS 40000u   /* tri-color can take ~25 s */
#define EINK_BUSY_WAIT_TIMEOUT_MS 2000u  /* boot/update bounded waits */

static uint8_t eink_bw[EINK_PLANE_SIZE];
static uint8_t eink_red[EINK_PLANE_SIZE];

typedef enum { EINK_IDLE = 0, EINK_REFRESHING } Eink_State;
static Eink_State eink_state = EINK_IDLE;
static uint32_t refresh_start_ms = 0;
static uint8_t eink_error = 0;

/* --- low-level ---------------------------------------------------------- */

static void eink_set_bus_speed(void)
{
  /* APB2 = 21 MHz; /8 = 2.6 MHz - fine for the ribbon-on-breadboard case. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  HAL_SPI_Init(&hspi1);
}

static void eink_cs(uint8_t on)
{
  HAL_GPIO_WritePin(EINK_CS_GPIO_PORT, EINK_CS_PIN,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);   /* active low */
}

static void eink_dc_command(void) { HAL_GPIO_WritePin(EINK_DC_GPIO_PORT, EINK_DC_PIN, GPIO_PIN_RESET); }
static void eink_dc_data(void)    { HAL_GPIO_WritePin(EINK_DC_GPIO_PORT, EINK_DC_PIN, GPIO_PIN_SET); }

static uint8_t eink_busy(void)
{
  return HAL_GPIO_ReadPin(EINK_BUSY_GPIO_PORT, EINK_BUSY_PIN) == GPIO_PIN_SET;
}

static void eink_spi(const uint8_t *buf, uint16_t len)
{
  HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, EINK_SPI_TIMEOUT_MS);
}

static void eink_command1(uint8_t cmd)
{
  eink_cs(1);
  eink_dc_command();
  eink_spi(&cmd, 1);
  eink_cs(0);
}

static void eink_command(uint8_t cmd, const uint8_t *data, uint16_t len)
{
  eink_cs(1);
  eink_dc_command();
  eink_spi(&cmd, 1);
  if (len > 0) {
    eink_dc_data();
    eink_spi(data, len);
  }
  eink_cs(0);
}

/** Bounded busy wait (BUSY high = working). Returns 1 if cleared in time. */
static uint8_t eink_wait_not_busy(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (eink_busy()) {
    if (HAL_GetTick() - start > timeout_ms) {
      eink_error = 1;
      return 0;
    }
  }
  return 1;
}

static void eink_hardware_reset(void)
{
  HAL_GPIO_WritePin(EINK_RST_GPIO_PORT, EINK_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(EINK_RST_GPIO_PORT, EINK_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(EINK_RST_GPIO_PORT, EINK_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
}

/** Power-up + full init + RAM windows (every refresh, like the reference). */
static void eink_power_up(void)
{
  /* SPI1 is shared: an SD transaction may have left the bus at the
   * negotiated /2 (~10 MHz), far past what the panel's ribbon tolerates.
   * Re-pin the eInk's own clock every power-up instead of inheriting. */
  eink_set_bus_speed();
  eink_hardware_reset();
  HAL_Delay(100);
  eink_wait_not_busy(EINK_BUSY_WAIT_TIMEOUT_MS);

  /* Default SSD1680 init for FPC-A005 2.9" tri-color (Adafruit_EPD):
   * panel loads its waveform LUT from OTP during the update (Ctrl2=0xF4). */
  eink_command1(SSD1680_SW_RESET);
  eink_wait_not_busy(EINK_BUSY_WAIT_TIMEOUT_MS);
  HAL_Delay(20);

  eink_command(SSD1680_DATA_MODE, (const uint8_t[]){0x03}, 1);
  eink_command(SSD1680_WRITE_BORDER, (const uint8_t[]){0x05}, 1);
  eink_command(SSD1680_WRITE_VCOM, (const uint8_t[]){0x36}, 1);
  eink_command(SSD1680_GATE_VOLTAGE, (const uint8_t[]){0x17}, 1);
  eink_command(SSD1680_SOURCE_VOLTAGE, (const uint8_t[]){0x41, 0x00, 0x32}, 3);
  eink_command(SSD1680_SET_RAMXCOUNT, (const uint8_t[]){EINK_XRAM_OFFSET}, 1);
  eink_command(SSD1680_SET_RAMYCOUNT, (const uint8_t[]){0x00, 0x00}, 2);

  /* RAM windows: X = 1..HEIGHT/8(+offset), Y = 0..WIDTH-1 */
  eink_command(SSD1680_SET_RAMXPOS,
               (const uint8_t[]){EINK_XRAM_OFFSET,
                                 (uint8_t)(EINK_HEIGHT / 8 - 1 + EINK_XRAM_OFFSET)},
               2);
  {
    uint16_t ymax = EINK_WIDTH - 1;   /* 295 */
    eink_command(SSD1680_SET_RAMYPOS,
                 (const uint8_t[]){0x00, 0x00,
                                   (uint8_t)(ymax & 0xFF),
                                   (uint8_t)(ymax >> 8)}, 4);
  }

  /* Driver output control: MUX = WIDTH-1 gates */
  eink_command(SSD1680_DRIVER_CONTROL,
               (const uint8_t[]){(uint8_t)((EINK_WIDTH - 1) & 0xFF),
                                 (uint8_t)((EINK_WIDTH - 1) >> 8), 0x00}, 3);
}

static void eink_write_plane(uint8_t cmd, const uint8_t *plane)
{
  /* Re-seat the address counters before each plane dump. */
  eink_command(SSD1680_SET_RAMXCOUNT, (const uint8_t[]){EINK_XRAM_OFFSET}, 1);
  eink_command(SSD1680_SET_RAMYCOUNT, (const uint8_t[]){0x00, 0x00}, 2);
  eink_cs(1);
  eink_dc_command();
  eink_spi(&cmd, 1);
  eink_dc_data();
  eink_spi(plane, EINK_PLANE_SIZE);
  eink_cs(0);
}

static void eink_deep_sleep(void)
{
  eink_command(SSD1680_DEEP_SLEEP, (const uint8_t[]){0x01}, 1);
}

/* --- public API ---------------------------------------------------------- */

uint8_t Eink_Init(void)
{
  eink_set_bus_speed();
  Eink_Clear();
  eink_power_up();
  eink_deep_sleep();
  eink_state = EINK_IDLE;
  return !eink_error;
}

void Eink_Clear(void)
{
  memset(eink_bw, 0xFF, EINK_PLANE_SIZE);  /* 1 = white */
  memset(eink_red, 0x00, EINK_PLANE_SIZE); /* 0 = not red */
}

void Eink_DrawPixel(uint16_t x, uint16_t y, uint8_t color)
{
  if (x >= EINK_WIDTH || y >= EINK_HEIGHT) return;

  uint16_t idx = (uint16_t)(((uint32_t)(EINK_WIDTH - 1 - x) * EINK_HEIGHT + y) / 8);
  uint8_t mask = (uint8_t)(0x80 >> (y & 7));

  switch (color) {
  case EINK_BLACK: eink_bw[idx] &= (uint8_t)~mask;  eink_red[idx] &= (uint8_t)~mask; break;
  case EINK_RED:   eink_bw[idx] |= mask;            eink_red[idx] |= mask;           break;
  default:         eink_bw[idx] |= mask;            eink_red[idx] &= (uint8_t)~mask; break; /* WHITE */
  }
}

void Eink_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
  for (uint16_t j = 0; j < h; j++) {
    for (uint16_t i = 0; i < w; i++) {
      Eink_DrawPixel(x + i, y + j, color);
    }
  }
}

void Eink_DrawText(uint16_t x, uint16_t y, uint8_t scale,
                   const char *text, uint8_t color)
{
  if (scale < 1) scale = 1;
  if (scale > 3) scale = 3;
  uint16_t cx = x;
  while (*text) {
    char c = *text++;
    if (c < 32 || c > 126) c = '?';
    const uint8_t *char_data = &font_5x7[(c - 32) * 5];
    for (uint8_t i = 0; i < 5; i++) {
      uint8_t col = char_data[i];
      for (uint8_t j = 0; j < 7; j++) {
        if (col & (1 << j)) {
          for (uint8_t sy = 0; sy < scale; sy++) {
            for (uint8_t sx = 0; sx < scale; sx++) {
              Eink_DrawPixel(cx + i * scale + sx, y + j * scale + sy, color);
            }
          }
        }
      }
    }
    cx += 6 * scale;
    if (cx + 5 * scale > EINK_WIDTH) break;
  }
}

uint8_t Eink_RequestRefresh(void)
{
  if (eink_state != EINK_IDLE || eink_error) return 0;

  eink_power_up();
  eink_write_plane(SSD1680_WRITE_RAM_BW, eink_bw);
  eink_write_plane(SSD1680_WRITE_RAM_RED, eink_red);

  /* Full (tri-color) update: Ctrl2 = 0xF4 (use OTP waveform), then activate */
  eink_command(SSD1680_DISP_CTRL2, (const uint8_t[]){0xF4}, 1);
  eink_command1(SSD1680_MASTER_ACTIVATE);

  eink_state = EINK_REFRESHING;
  refresh_start_ms = HAL_GetTick();
  return 1;
}

uint8_t Eink_IsBusy(void)
{
  return eink_state == EINK_REFRESHING;
}

void Eink_Service(void)
{
  if (eink_state != EINK_REFRESHING) return;

  if (!eink_busy()) {
    /* Physical refresh done - park the panel in deep sleep. */
    eink_deep_sleep();
    eink_state = EINK_IDLE;
  } else if (HAL_GetTick() - refresh_start_ms > EINK_REFRESH_TIMEOUT_MS) {
    /* BUSY stuck (wire off the pad? panel dead?): give up this refresh and
     * mark the error; a reboot-side Eink_Init is the recovery path. */
    eink_error = 1;
    eink_state = EINK_IDLE;
  }
}
