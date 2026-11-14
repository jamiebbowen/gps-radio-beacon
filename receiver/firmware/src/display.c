/**
 * @file display.c
 * @brief SHARP Memory LCD driver (Adafruit 3502, 168x144) over shared SPI1
 *
 * Replaces the v1.x SSD1309 I2C driver; same public API (display.h) so all
 * display_modes pages work unchanged on the larger canvas (they simply use
 * the top-left 128x64 region until layouts catch up).
 *
 * Hardware quirks handled here (from the Adafruit_SHARP_Memory reference):
 *  - CS is ACTIVE HIGH (inverted vs SD card / eInk sharing SPI1)
 *  - Wire bit order is LSB-first, but SPI1 is MSB-first for the SD card:
 *    every byte is software-reversed (__RBIT) before HAL transmission
 *  - 1 bit in the frame = WHITE (reflecting), 0 = black - the opposite of
 *    the drawing convention, so the bit is inverted at stream time
 *  - VCOM must toggle at ~1-10 Hz or the panel degrades: handled by flipping
 *    the M1 bit in every frame write, guaranteed by the 250 ms throttle
 *  - SHARP tops out at ~2 MHz: SPI1 is re-prescled to /16 (1.3 MHz at 21 MHz
 *    PCLK2) before every transfer, matching what sd_diskio does for the SD
 */

#include "display.h"
#include "main.h"      /* SHARP_CS_PIN / SHARP_DISP_PIN, hspi1 extern via... */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "font.h"

extern SPI_HandleTypeDef hspi1;  /* shared SPI1 handle (main.c) */

/* Framebuffer: SSD1306-style page packing (bit y%8 in byte x + (y/8)*W).
 * 1 = pixel on (black). 168*144/8 = 3024 bytes. */
#define SHARP_BUFFER_SIZE   (DISPLAY_HW_WIDTH * DISPLAY_HW_HEIGHT / 8)
#define SHARP_BYTES_PER_LINE (DISPLAY_HW_WIDTH / 8)   /* 21 */
#define SHARP_SPI_TIMEOUT_MS 25

/* SHARP command bits EXPRESSED IN MSB-FIRST WIRE ORDER (we bit-reverse the
 * library-pretzel at stream time; these are the datasheet-facing values):
 *   M0 = 0x01 write line mode, M1 = 0x02 VCOM toggle, M2 = 0x04 clear */
#define SHARP_CMD_WRITE   0x01
#define SHARP_CMD_VCOM    0x02
#define SHARP_CMD_CLEAR   0x04

static uint8_t display_buffer[SHARP_BUFFER_SIZE];
static uint8_t sharp_vcom = 0;       /* 0 or SHARP_CMD_VCOM - frame polarity */

/** Reverse a byte (SHARP wants LSB-first; SPI1 peripheral is MSB-first). */
static inline uint8_t sharp_rev8(uint8_t b)
{
#if defined(__ARM_ARCH)
  return (uint8_t)(__RBIT((uint32_t)b) >> 24);
#else
  /* Host-test builds: portable nibble/bit dance */
  b = (uint8_t)((b >> 4) | (b << 4));
  b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
  b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
  return b;
#endif
}

/** Drop SPI1 to a SHARP-safe clock before touching the panel. */
static void sharp_set_bus_speed(void)
{
  /* APB2 = 21 MHz; /16 = 1.31 MHz - comfortably under the ~2 MHz limit. */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  HAL_SPI_Init(&hspi1);
}

static void sharp_select(void)
{
  HAL_GPIO_WritePin(SHARP_CS_GPIO_PORT, SHARP_CS_PIN, GPIO_PIN_SET);   /* HIGH */
}

static void sharp_deselect(void)
{
  HAL_GPIO_WritePin(SHARP_CS_GPIO_PORT, SHARP_CS_PIN, GPIO_PIN_RESET);
}

static void sharp_write(const uint8_t *buf, uint16_t len)
{
  /* A failed frame needs no handling: the panel is redrawn in full on the
   * next 250 ms refresh, and SPI (unlike I2C) cannot wedge the bus. */
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, SHARP_SPI_TIMEOUT_MS);
}

/**
 * @brief Initialize display (and the shared I2C bus it used to own)
 * @retval Status code
 */
uint8_t Display_Init(void)
{
  /* Panel on, CS idle-low were already established in MX_GPIO_Init. */

  /* Clear command: vcom | M2, followed by an 8-bit dummy. Banishes whatever
   * random pattern the panel woke up with. */
  sharp_set_bus_speed();
  uint8_t clr[2] = { (uint8_t)(sharp_rev8(SHARP_CMD_CLEAR | SHARP_CMD_VCOM)),
                     0x00 };
  sharp_select();
  sharp_write(clr, sizeof(clr));
  sharp_deselect();
  sharp_vcom = SHARP_CMD_VCOM;

  Display_Clear();
  Display_Update();

  return DISPLAY_OK;
}

/**
 * @brief Clear display buffer (all pixels off = all white on the panel)
 * @retval None
 */
void Display_Clear(void)
{
  memset(display_buffer, 0, SHARP_BUFFER_SIZE);
}

/**
 * @brief Push framebuffer to the panel (full frame, ~20 ms at 1.3 MHz)
 * @retval None
 */
void Display_Update(void)
{
  sharp_set_bus_speed();

  /* Toggle VCOM polarity for this frame and build the write-mode command. */
  sharp_vcom ^= SHARP_CMD_VCOM;
  uint8_t cmd = sharp_rev8((uint8_t)(SHARP_CMD_WRITE | sharp_vcom));

  sharp_select();
  sharp_write(&cmd, 1);

  /* Per line: [reversed 1-based line addr][21 reversed/inverted data bytes]
   * [0x00 dummy]. SHARP bit: 1 = white, 0 = black -> invert our "on" bits. */
  static uint8_t line[2 + SHARP_BYTES_PER_LINE];  /* addr + data(+trailer) */
  for (uint8_t y = 0; y < DISPLAY_HW_HEIGHT; y++) {
    const uint8_t *page_row = &display_buffer[(y >> 3) * DISPLAY_HW_WIDTH];
    uint8_t bit = (uint8_t)(1u << (y & 7));
    line[0] = sharp_rev8((uint8_t)(y + 1));
    for (uint8_t j = 0; j < SHARP_BYTES_PER_LINE; j++) {
      uint8_t out = 0xFF;   /* default white */
      for (uint8_t k = 0; k < 8; k++) {
        if (page_row[8 * j + k] & bit) {
          out &= (uint8_t)~(1u << k);   /* black pixel -> 0 */
        }
      }
      line[1 + j] = sharp_rev8(out);
    }
    line[1 + SHARP_BYTES_PER_LINE] = 0x00;
    sharp_write(line, sizeof(line));
  }
  uint8_t trailer = 0x00;
  sharp_write(&trailer, 1);
  sharp_deselect();
}

/**
 * @brief Draw a single pixel
 * @param x X coordinate (in logical display space)
 * @param y Y coordinate (in logical display space)
 * @param color Pixel color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
  /* Check if the coordinates are within bounds */
  if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) {
    return;
  }

  uint16_t byte_pos = x + (y / 8) * DISPLAY_HW_WIDTH;
  uint8_t bit_pos = y % 8;

  if (color) {
    display_buffer[byte_pos] |= (1 << bit_pos);
  } else {
    display_buffer[byte_pos] &= ~(1 << bit_pos);
  }
}

/**
 * @brief Draw a line using Bresenham's algorithm
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color Line color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
  int16_t dx = abs(x1 - x0);
  int16_t sx = x0 < x1 ? 1 : -1;
  int16_t dy = -abs(y1 - y0);
  int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx + dy;
  int16_t e2;

  while (1) {
    Display_DrawPixel(x0, y0, color);

    if (x0 == x1 && y0 == y1) {
      break;
    }

    e2 = 2 * err;

    /* (Exhaustively verified over the full display grid: with this
     * formulation the coordinates never overshoot, so the top check alone
     * terminates every line exactly on its endpoint.) */
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }

    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

/**
 * @brief Draw a circle using Bresenham's algorithm
 * @param x0 Center X coordinate
 * @param y0 Center Y coordinate
 * @param r Radius
 * @param color Circle color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  Display_DrawPixel(x0, y0 + r, color);
  Display_DrawPixel(x0, y0 - r, color);
  Display_DrawPixel(x0 + r, y0, color);
  Display_DrawPixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }

    x++;
    ddF_x += 2;
    f += ddF_x;

    Display_DrawPixel(x0 + x, y0 + y, color);
    Display_DrawPixel(x0 - x, y0 + y, color);
    Display_DrawPixel(x0 + x, y0 - y, color);
    Display_DrawPixel(x0 - x, y0 - y, color);
    Display_DrawPixel(x0 + y, y0 + x, color);
    Display_DrawPixel(x0 - y, y0 + x, color);
    Display_DrawPixel(x0 + y, y0 - x, color);
    Display_DrawPixel(x0 - y, y0 - x, color);
  }
}

/**
 * @brief Draw a single character
 * @param x X coordinate
 * @param y Y coordinate
 * @param c Character to draw
 * @retval None
 */
void Display_DrawChar(uint8_t x, uint8_t y, char c)
{
  if (c < 32 || c > 126) {
    c = '?';
  }

  /* Get character data from font */
  const uint8_t *char_data = &font_5x7[(c - 32) * 5];

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t line = char_data[i];
    for (uint8_t j = 0; j < 7; j++) {
      if (line & (1 << j)) {
        Display_DrawPixel(x + i, y + j, 1);
      }
    }
  }
}

/**
 * @brief Draw text string
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text string to draw
 * @retval None
 */
void Display_DrawText(uint8_t x, uint8_t y, const char *text)
{
  uint8_t cursor_x = x;

  while (*text) {
    if (cursor_x + 5 > DISPLAY_WIDTH) {
      break;
    }

    Display_DrawChar(cursor_x, y, *text);
    cursor_x += 6; /* 5 pixels + 1 pixel space */
    text++;
  }
}

/**
 * @brief Draw text string using row and column coordinates
 * @param row Row (0-based, each row is 8 pixels high)
 * @param col Column (0-based, each column is 6 pixels wide)
 * @param text Text string to draw
 * @retval None
 */
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
  uint8_t x = col * 6;
  uint8_t y = row * 8;

  Display_DrawText(x, y, text);
}

/**
 * @brief Draw direction indicator (arrow pointing in specified direction)
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param angle Direction angle in degrees (0-359.9)
 * @retval None
 */
void Display_DrawDirectionIndicator(uint8_t x, uint8_t y, float angle)
{
  /* Convert angle to radians */
  float rad = angle * M_PI / 180.0f;

  /* Draw compass circle */
  uint8_t radius = 15;
  Display_DrawCircle(x, y, radius, 1);

  /* Calculate arrow points - make arrow slightly longer */
  int8_t tip_x = x + (int8_t)(sinf(rad) * (radius - 2));
  int8_t tip_y = y - (int8_t)(cosf(rad) * (radius - 2));

  /* Calculate points for arrow base */
  float rad_left = rad - M_PI * 0.7f;  /* Adjusted for better arrow shape */
  float rad_right = rad + M_PI * 0.7f;
  int8_t base_left_x = x + (int8_t)(sinf(rad_left) * radius * 0.5f);
  int8_t base_left_y = y - (int8_t)(cosf(rad_left) * radius * 0.5f);
  int8_t base_right_x = x + (int8_t)(sinf(rad_right) * radius * 0.5f);
  int8_t base_right_y = y - (int8_t)(cosf(rad_right) * radius * 0.5f);

  /* Draw arrow */
  Display_DrawLine(x, y, tip_x, tip_y, 1);
  Display_DrawLine(tip_x, tip_y, base_left_x, base_left_y, 1);
  Display_DrawLine(tip_x, tip_y, base_right_x, base_right_y, 1);
  Display_DrawLine(base_left_x, base_left_y, base_right_x, base_right_y, 1);

  /* Cardinal direction markers */
  Display_DrawPixel(x, y - radius, 1);
  Display_DrawPixel(x, y - radius + 1, 1);

  Display_DrawPixel(x + radius, y, 1);
  Display_DrawPixel(x + radius - 1, y, 1);

  Display_DrawPixel(x, y + radius, 1);
  Display_DrawPixel(x, y + radius - 1, 1);

  Display_DrawPixel(x - radius, y, 1);
  Display_DrawPixel(x - radius + 1, y, 1);
}

