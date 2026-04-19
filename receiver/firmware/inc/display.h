/**
 * @file display.h
 * @brief Display driver header for SSD1309 OLED
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

/* Display dimensions */
/* Hardware is configured as 128x64, but physically mounted as 64x128 */
#define DISPLAY_HW_WIDTH   128  /* Hardware width */
#define DISPLAY_HW_HEIGHT  64   /* Hardware height */
#define DISPLAY_WIDTH      128  /* Logical width (physical) */
#define DISPLAY_HEIGHT     64   /* Logical height (physical) */

/* Display commands */
#define SSD1309_SETCONTRAST         0x81
#define SSD1309_DISPLAYALLON_RESUME 0xA4
#define SSD1309_DISPLAYALLON        0xA5
#define SSD1309_NORMALDISPLAY       0xA6
#define SSD1309_INVERTDISPLAY       0xA7
#define SSD1309_DISPLAYOFF          0xAE
#define SSD1309_DISPLAYON           0xAF
#define SSD1309_SETDISPLAYOFFSET    0xD3
#define SSD1309_SETCOMPINS          0xDA
#define SSD1309_SETVCOMDETECT       0xDB
#define SSD1309_SETDISPLAYCLOCKDIV  0xD5
#define SSD1309_SETPRECHARGE        0xD9
#define SSD1309_SETMULTIPLEX        0xA8
#define SSD1309_SETLOWCOLUMN        0x00
#define SSD1309_SETHIGHCOLUMN       0x10
#define SSD1309_SETSTARTLINE        0x40
#define SSD1309_MEMORYMODE          0x20
#define SSD1309_COLUMNADDR          0x21
#define SSD1309_PAGEADDR            0x22
#define SSD1309_COMSCANINC          0xC0
#define SSD1309_COMSCANDEC          0xC8
#define SSD1309_SEGREMAP            0xA0
#define SSD1309_CHARGEPUMP          0x8D
#define SSD1309_EXTERNALVCC         0x01
#define SSD1309_SWITCHCAPVCC        0x02

/* Display is fixed in 90 degree orientation */

/* Function prototypes */
uint8_t Display_Init(void);
void Display_Clear(void);
void Display_Update(void);
/* No rotation functions - display is fixed in 90 degree orientation */
void Display_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void Display_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void Display_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void Display_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
void Display_DrawChar(uint8_t x, uint8_t y, char c);
void Display_DrawText(uint8_t x, uint8_t y, const char *text);
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text);
void Display_DrawDirectionIndicator(uint8_t x, uint8_t y, float angle);
void Display_ShowBootScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_H */
