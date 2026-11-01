/**
 * @file eink.h
 * @brief SSD1680 2.9" tri-color eInk (Adafruit 4778 FeatherWing) driver
 *
 * Persistent "summary" screen (v2.0 hardware): lives on shared SPI1 with its
 * own CS/DC/RST/BUSY. Framebuffers are kept in MCU RAM (2 x 4736 bytes) -
 * the FeatherWing's onboard SPI SRAM is left unselected; RAM is cheaper
 * than the transactions and the F401CC has 64 KB to spend.
 *
 * Register/init sequence is a port of Adafruit_EPD's Adafruit_SSD1680
 * (FPC-A005 2.9" tri-color path, OTP waveform, full updates only).
 * Refresh is asynchronous for the caller: Eink_RequestRefresh() does the
 * SPI transfers (~30 ms), then Eink_Service() polls BUSY from the main loop
 * until the ~15-25 s physical refresh finishes and re-sleeps the panel.
 * Refreshes are therefore only rate-appropriate at Human cadence (seconds),
 * never inside packet-dense paths.
 */

#ifndef EINK_H
#define EINK_H

#include <stdint.h>

#define EINK_WIDTH    296
#define EINK_HEIGHT   128

typedef enum {
  EINK_WHITE = 0,
  EINK_BLACK = 1,
  EINK_RED   = 2
} Eink_Color;

uint8_t Eink_Init(void);                 /* reset+init+deep sleep; 1 = busy pin responded at least once during boot... */
void    Eink_Clear(void);                /* framebuffers to white */
void    Eink_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
void    Eink_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);
void    Eink_DrawText(uint16_t x, uint16_t y, uint8_t scale,
                      const char *text, uint8_t color);

uint8_t Eink_RequestRefresh(void);       /* 1 = started/transferred; 0 = busy or uninit */
uint8_t Eink_IsBusy(void);               /* physical refresh in progress */
void    Eink_Service(void);              /* call every main-loop iteration */

#endif /* EINK_H */
