/**
 * @file display.h
 * @brief Display driver header - SHARP Memory LCD (Adafruit 3502, v2.0 hw)
 *
 * The v2.0 receiver replaced the SSD1309 I2C OLED with a SHARP 168x144
 * memory LCD on SPI1 (see docs/hardware-receiver.md). The drawing API below
 * is unchanged so all display_modes code keeps working; only the physical
 * size constants changed. SPI1 is a shared bus - the driver re-prescaler
 * switches per use and never assumes the bus is idle at its speed.
 */

#ifndef __DISPLAY_H
#define __DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Display status codes */
#define DISPLAY_OK      0
#define DISPLAY_ERROR   1
#define DISPLAY_TIMEOUT 2

/* Display dimensions - SHARP Memory LCD (LS013B7DH06-family, 3502 breakout) */
#define DISPLAY_HW_WIDTH   168  /* Hardware width */
#define DISPLAY_HW_HEIGHT  144  /* Hardware height */
#define DISPLAY_WIDTH      168  /* Logical width (physical) */
#define DISPLAY_HEIGHT     144  /* Logical height (physical) */

/* Display is fixed in 90 degree orientation */

/* Function prototypes */
uint8_t Display_Init(void);
void Display_Clear(void);
void Display_Update(void);
/* No rotation functions - display is fixed in 90 degree orientation */
void Display_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void Display_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void Display_DrawChar(uint8_t x, uint8_t y, char c);
void Display_DrawText(uint8_t x, uint8_t y, const char *text);
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text);
void Display_DrawDirectionIndicator(uint8_t x, uint8_t y, float angle);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_H */
