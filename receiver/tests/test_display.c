/**
 * @file test_display.c
 * @brief Host-side unit tests for the SHARP Memory LCD driver (v2.0).
 *
 * A capturing SPI fake records every byte pushed to the "panel" along with
 * the CS pin state, so tests can verify:
 *   - the init clear command and its format
 *   - full-frame structure (144 lines x 23 bytes + cmd + trailer)
 *   - CS ACTIVE-HIGH polarity (inverted vs every other SPI1 device)
 *   - software bit reversal (SHARP is LSB-first on an MSB-first bus)
 *   - 1 = white / 0 = black data polarity
 *   - per-frame VCOM toggling (panel lifetime requirement)
 *   - SPI1 prescaler switched to /16 (~1.3 MHz) before transfer
 *
 * Framebuffer correctness (pixels, lines, rects, circles, text) is asserted
 * directly against the driver's page-packed internal buffer - that packing
 * did not change from the SSD1309 driver.
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "display.h"
#include "main.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/display.c"

/* ------------------------------------------------------------------ */
/* Fake SPI bus + GPIO                                                 */
/* ------------------------------------------------------------------ */

SPI_HandleTypeDef hspi1;   /* satisfies the extern in display.c */

#define SPI_LOG_CAP       (128 * 1024)
#define SPI_TXN_CAP       2048

static uint8_t  spi_log[SPI_LOG_CAP];
static uint32_t spi_log_len = 0;
static uint32_t txn_off[SPI_TXN_CAP];       /* byte offset of each transfer */
static uint32_t txn_len[SPI_TXN_CAP];
static uint8_t  txn_cs[SPI_TXN_CAP];        /* CS level during transfer */
static uint32_t txn_count = 0;

static uint8_t  fake_spi_fail = 0;
static uint32_t last_prescaler = 0;
static uint8_t  sharp_cs_level = 0;         /* tracked level on PB10 */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    (void)port;
    if (pin == SHARP_CS_PIN) sharp_cs_level = (st == GPIO_PIN_SET) ? 1 : 0;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
    last_prescaler = hspi->Init.BaudRatePrescaler;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    (void)hspi; (void)timeout;
    if (fake_spi_fail) return HAL_ERROR;
    if (txn_count < SPI_TXN_CAP && spi_log_len + size < SPI_LOG_CAP) {
        txn_off[txn_count] = spi_log_len;
        txn_len[txn_count] = size;
        txn_cs[txn_count]  = sharp_cs_level;
        txn_count++;
        memcpy(&spi_log[spi_log_len], data, size);
        spi_log_len += size;
    }
    return HAL_OK;
}

static void spi_log_reset(void)
{
    spi_log_len = 0;
    txn_count = 0;
}

/** Host-side reference bit reversal (display.c has its own copy) */
static uint8_t rev8(uint8_t b)
{
    b = (uint8_t)((b >> 4) | (b << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

/** Byte pointer for transfer n's payload */
static const uint8_t *txn_bytes(uint32_t n) { return &spi_log[txn_off[n]]; }

/** Total payload bytes transmitted since the last log reset */
static uint32_t spi_total_bytes(void) { return spi_log_len; }

/* ------------------------------------------------------------------ */
/* Framebuffer helpers (page-packed buffer did not change)             */
/* ------------------------------------------------------------------ */

static uint8_t get_pixel(uint8_t x, uint8_t y)
{
    uint16_t byte_pos = x + (y / 8) * DISPLAY_HW_WIDTH;
    return (display_buffer[byte_pos] >> (y % 8)) & 1;
}

static uint32_t count_lit_pixels(void)
{
    uint32_t n = 0;
    for (uint16_t i = 0; i < SHARP_BUFFER_SIZE; i++) {
        uint8_t b = display_buffer[i];
        while (b) { n += b & 1; b >>= 1; }
    }
    return n;
}

/** Locate the transfer for display line y (1 cmd + per-line + trailer
 *  layout: cmd is txn k, line y (0-based) is txn k+1+y) */
static uint32_t txn_of_line(uint32_t first_frame_txn, uint8_t y)
{
    return first_frame_txn + 1 + y;
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_clears_panel_and_pushes_blank_frame)
{
    spi_log_reset();
    sharp_cs_level = 0;
    CHECK(Display_Init() == DISPLAY_OK);

    /* First transfer: 2-byte clear command (M2|VCOM, then dummy) */
    CHECK(txn_len[0] == 2);
    CHECK(txn_bytes(0)[0] == rev8(SHARP_CMD_CLEAR | SHARP_CMD_VCOM));
    CHECK(txn_bytes(0)[1] == 0x00);
    CHECK(txn_cs[0] == 1);                     /* CS was ACTIVE HIGH */

    /* Then a full blank frame: 1 cmd + 144 lines + trailer */
    CHECK(txn_count == 1 + 1 + DISPLAY_HW_HEIGHT + 1);
    CHECK(txn_len[1] == 1);                    /* write-mode cmd byte */

    /* All transfers did their slowing-to-SHARP-safe clock first... the
     * most recent full-frame push leaves the bus at /16. */
    CHECK(last_prescaler == SPI_BAUDRATEPRESCALER_16);

    /* CS idles LOW after the frame (everything else on SPI1 expects that) */
    CHECK(sharp_cs_level == 0);
}

TEST(test_update_frame_structure)
{
    CHECK(Display_Init() == DISPLAY_OK);
    Display_Clear();
    uint32_t blank_first_txn = 1;   /* after the init clear cmd */
    spi_log_reset();
    Display_Update();

    /* Structure: 1 cmd + 144 x (addr+21data+trailer) + 1 final trailer */
    CHECK(txn_count == 1 + DISPLAY_HW_HEIGHT + 1);
    CHECK(txn_len[0] == 1);                     /* cmd */
    uint32_t total = 1 + 1;                     /* cmd + final trailer */
    for (uint8_t y = 0; y < DISPLAY_HW_HEIGHT; y++) {
        uint32_t t = txn_of_line(0, y);
        CHECK(txn_len[t] == 2 + SHARP_BYTES_PER_LINE);
        CHECK(txn_bytes(t)[0] == rev8((uint8_t)(y + 1)));   /* 1-based addr */
        CHECK(txn_bytes(t)[txn_len[t] - 1] == 0x00);        /* EOL trailer */
        CHECK(txn_cs[t] == 1);                              /* CS high */
        total += txn_len[t];
    }
    CHECK(spi_total_bytes() == total);
    (void)blank_first_txn;
}

TEST(test_vcom_toggles_every_frame)
{
    CHECK(Display_Init() == DISPLAY_OK);

    spi_log_reset();
    Display_Update();
    uint8_t c1 = txn_bytes(0)[0];
    spi_log_reset();
    Display_Update();
    uint8_t c2 = txn_bytes(0)[0];
    spi_log_reset();
    Display_Update();
    uint8_t c3 = txn_bytes(0)[0];

    /* Write-mode bit always set, VCOM bit alternates */
    CHECK(c1 == rev8(SHARP_CMD_WRITE | SHARP_CMD_VCOM));
    CHECK(c2 == rev8(SHARP_CMD_WRITE));
    CHECK(c3 == rev8(SHARP_CMD_WRITE | SHARP_CMD_VCOM));
}

TEST(test_pixel_polatiity_and_bit_reversal_on_wire)
{
    CHECK(Display_Init() == DISPLAY_OK);

    /* Blank frame: every data byte is white (0xFF - self-reversing) */
    Display_Clear();
    spi_log_reset();
    Display_Update();
    for (uint8_t j = 0; j < SHARP_BYTES_PER_LINE; j++) {
        CHECK(txn_bytes(txn_of_line(0, 0))[1 + j] == 0xFF);
    }

    /* Black pixel at (0,0): byte 0 bit 0 goes 0 -> out 0xFE -> wire 0x7F */
    Display_DrawPixel(0, 0, 1);
    spi_log_reset();
    Display_Update();
    CHECK(txn_bytes(txn_of_line(0, 0))[1] == rev8(0xFE));   /* == 0x7F */

    /* Black pixel at (7,0): byte 0 bit 7 -> out 0x7F -> wire 0xFE */
    Display_Clear();
    Display_DrawPixel(7, 0, 1);
    spi_log_reset();
    Display_Update();
    CHECK(txn_bytes(txn_of_line(0, 0))[1] == rev8(0x7F));   /* == 0xFE */

    /* Black pixel at (8,10): line 11... y=10 -> data byte j=1, bit 0 */
    Display_Clear();
    Display_DrawPixel(8, 10, 1);
    spi_log_reset();
    Display_Update();
    uint32_t t = txn_of_line(0, 10);
    CHECK(txn_bytes(t)[0] == rev8(11));
    CHECK(txn_bytes(t)[1 + 0] == 0xFF);      /* byte 0: untouched */
    CHECK(txn_bytes(t)[1 + 1] == rev8(0xFE));/* byte 1 bit 0 cleared */
    CHECK(txn_bytes(t)[1 + 2] == 0xFF);
}

TEST(test_spi_failures_are_not_fatal)
{
    CHECK(Display_Init() == DISPLAY_OK);

    fake_spi_fail = 1;
    Display_Clear();
    Display_Update();          /* every chunk errors */
    fake_spi_fail = 0;
    Display_Clear();
    Display_Update();

    CHECK(sharp_cs_level == 0);   /* frame still terminates deselected */
}

TEST(test_framebuffer_packing)
{
    Display_Clear();
    Display_DrawPixel(0, 0, 1);
    Display_DrawPixel(DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1, 1);
    CHECK(get_pixel(0, 0) == 1);
    CHECK(get_pixel(DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1) == 1);
    CHECK(count_lit_pixels() == 2);

    Display_DrawPixel(0, 0, 0);
    CHECK(get_pixel(0, 0) == 0);
    CHECK(count_lit_pixels() == 1);

    /* Out-of-bounds writes must be ignored, not corrupt memory */
    Display_DrawPixel(DISPLAY_WIDTH, 0, 1);
    Display_DrawPixel(0, DISPLAY_HEIGHT, 1);
    Display_DrawPixel(255, 255, 1);
    CHECK(count_lit_pixels() == 1);
}

TEST(test_draw_line_all_octants)
{
    Display_Clear();
    Display_DrawLine(5, 5, 14, 5, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(5, 5) == 1 && get_pixel(14, 5) == 1);

    Display_Clear();
    Display_DrawLine(3, 10, 3, 19, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(3, 10) == 1 && get_pixel(3, 19) == 1);

    Display_Clear();
    Display_DrawLine(0, 0, 9, 9, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(0, 0) == 1 && get_pixel(9, 9) == 1);

    Display_Clear();
    Display_DrawLine(10, 20, 12, 5, 1);      /* steep up   */
    Display_DrawLine(30, 5, 20, 8, 1);       /* shallow left */
    CHECK(get_pixel(10, 20) == 1 && get_pixel(12, 5) == 1);
    CHECK(get_pixel(30, 5) == 1 && get_pixel(20, 8) == 1);

    Display_Clear();
    Display_DrawLine(50, 50, 50, 50, 1);
    CHECK(count_lit_pixels() == 1);
}

TEST(test_draw_and_fill_rect_and_circle)
{
    Display_Clear();
    Display_DrawRect(10, 10, 5, 4, 1);
    CHECK(get_pixel(10, 10) == 1);
    CHECK(get_pixel(14, 10) == 1);
    CHECK(get_pixel(10, 13) == 1);
    CHECK(get_pixel(14, 13) == 1);
    CHECK(get_pixel(12, 11) == 0);

    Display_Clear();
    Display_DrawCircle(84, 72, 10, 1);
    CHECK(get_pixel(84, 82) == 1);
    CHECK(get_pixel(84, 62) == 1);
    CHECK(get_pixel(94, 72) == 1);
    CHECK(get_pixel(74, 72) == 1);
    CHECK(get_pixel(84, 72) == 0);
}

TEST(test_draw_char_and_text)
{
    Display_Clear();
    Display_DrawChar(0, 0, 'A');
    CHECK(count_lit_pixels() > 0);

    /* Non-printable characters render as '?' rather than indexing
     * outside the font table */
    Display_Clear();
    Display_DrawChar(0, 0, (char)7);
    uint32_t bell_pixels = count_lit_pixels();
    Display_Clear();
    Display_DrawChar(0, 0, '?');
    CHECK(bell_pixels == count_lit_pixels());

    Display_Clear();
    Display_DrawChar(0, 0, 'A');
    uint32_t one = count_lit_pixels();
    Display_Clear();
    Display_DrawText(0, 0, "AA");
    CHECK(count_lit_pixels() == 2 * one);      /* no overlap at 6 px pitch */

    /* Text clipped at the right edge instead of wrapping */
    Display_Clear();
    Display_DrawText(DISPLAY_WIDTH - 4, 0, "AA");
    CHECK(count_lit_pixels() == 0);

    /* Row/col addressing maps to 6x8 cells */
    Display_Clear();
    Display_DrawChar(6, 8, 'A');
    uint32_t direct = count_lit_pixels();
    Display_Clear();
    Display_DrawTextRowCol(1, 1, "A");
    CHECK(count_lit_pixels() == direct);
}

TEST(test_direction_indicator_draws_arrow)
{
    Display_Clear();
    Display_DrawDirectionIndicator(84, 72, 0.0f);
    uint32_t north = count_lit_pixels();
    CHECK(north > 20);

    Display_Clear();
    Display_DrawDirectionIndicator(84, 72, 135.0f);
    CHECK(count_lit_pixels() > 20);
}

TEST(test_draw_line_sweeps_endpoints)
{
    for (uint8_t dx = 0; dx <= 10; dx++) {
        for (uint8_t dy = 0; dy <= 10; dy++) {
            Display_Clear();
            Display_DrawLine(0, 0, dx, dy, 1);
            Display_DrawLine(10, 10, 10 - dx, 10 - dy, 1);
        }
    }
    Display_Clear();
    Display_DrawLine(2, 3, 5, 3, 1);
    CHECK(get_pixel(2, 3) == 1 && get_pixel(5, 3) == 1);
    Display_Clear();
    Display_DrawLine(7, 1, 7, 6, 1);
    CHECK(get_pixel(7, 1) == 1 && get_pixel(7, 6) == 1);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_clears_panel_and_pushes_blank_frame();
    run_test_update_frame_structure();
    run_test_vcom_toggles_every_frame();
    run_test_pixel_polatiity_and_bit_reversal_on_wire();
    run_test_spi_failures_are_not_fatal();
    run_test_framebuffer_packing();
    run_test_draw_line_all_octants();
    run_test_draw_and_fill_rect_and_circle();
    run_test_draw_char_and_text();
    run_test_direction_indicator_draws_arrow();
    run_test_draw_line_sweeps_endpoints();

    return TEST_SUMMARY();
}
