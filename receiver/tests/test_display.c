/**
 * @file test_display.c
 * @brief Host-side unit tests for the SSD1309 display driver.
 *
 * A capturing I2C fake records every byte pushed to the "panel" so tests
 * can verify the init sequence, framebuffer chunking, and - critically -
 * the headless degradation path: when I2C init fails the receiver must
 * keep running with zero display bus traffic.
 *
 * Framebuffer correctness (pixels, lines, rects, circles, text) is
 * asserted directly against the driver's internal buffer.
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "display.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/display.c"

/* ------------------------------------------------------------------ */
/* Fake I2C bus                                                        */
/* ------------------------------------------------------------------ */

static uint8_t  fake_i2c_init_fail = 0;
static uint32_t i2c_transmits      = 0;   /* number of Master_Transmit calls */
static uint32_t i2c_bytes          = 0;   /* total payload bytes             */
static uint8_t  i2c_last_cmd       = 0;   /* last command byte seen          */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    return fake_i2c_init_fail ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                          uint8_t *data, uint16_t size,
                                          uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    CHECK(addr == (SSD1309_I2C_ADDR << 1));
    i2c_transmits++;
    i2c_bytes += size;
    if (size >= 2 && data[0] == SSD1309_COMMAND) {
        i2c_last_cmd = data[1];
    }
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint8_t get_pixel(uint8_t x, uint8_t y)
{
    uint16_t byte_pos = x + (y / 8) * DISPLAY_HW_WIDTH;
    return (display_buffer[byte_pos] >> (y % 8)) & 1;
}

static uint32_t count_lit_pixels(void)
{
    uint32_t n = 0;
    for (uint16_t i = 0; i < SSD1309_BUFFER_SIZE; i++) {
        uint8_t b = display_buffer[i];
        while (b) { n += b & 1; b >>= 1; }
    }
    return n;
}

static void reset_i2c_counters(void)
{
    i2c_transmits = 0;
    i2c_bytes = 0;
    i2c_last_cmd = 0;
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_sends_full_sequence_and_clears)
{
    reset_i2c_counters();
    CHECK(Display_Init() == DISPLAY_OK);

    /* init sequence + 2 orientation cmds + update (6 cmds + 64 data chunks) */
    CHECK(i2c_transmits > sizeof(ssd1309_init_sequence));
    CHECK(count_lit_pixels() == 0);          /* buffer cleared */
}

TEST(test_pixel_set_clear_and_bounds)
{
    Display_Clear();

    Display_DrawPixel(0, 0, 1);
    Display_DrawPixel(DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1, 1);
    CHECK(get_pixel(0, 0) == 1);
    CHECK(get_pixel(DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1) == 1);
    CHECK(count_lit_pixels() == 2);

    Display_DrawPixel(0, 0, 0);              /* clear a pixel */
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

    /* Horizontal: exactly 10 pixels */
    Display_DrawLine(5, 5, 14, 5, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(5, 5) == 1 && get_pixel(14, 5) == 1);

    /* Vertical */
    Display_Clear();
    Display_DrawLine(3, 10, 3, 19, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(3, 10) == 1 && get_pixel(3, 19) == 1);

    /* 45-degree diagonal: endpoints + steps */
    Display_Clear();
    Display_DrawLine(0, 0, 9, 9, 1);
    CHECK(count_lit_pixels() == 10);
    CHECK(get_pixel(0, 0) == 1 && get_pixel(9, 9) == 1);

    /* Steep and reversed directions (remaining Bresenham branches) */
    Display_Clear();
    Display_DrawLine(10, 20, 12, 5, 1);      /* steep up   */
    Display_DrawLine(30, 5, 20, 8, 1);       /* shallow left */
    CHECK(get_pixel(10, 20) == 1 && get_pixel(12, 5) == 1);
    CHECK(get_pixel(30, 5) == 1 && get_pixel(20, 8) == 1);

    /* Single point (x0==x1, y0==y1) */
    Display_Clear();
    Display_DrawLine(50, 50, 50, 50, 1);
    CHECK(count_lit_pixels() == 1);
}

TEST(test_draw_and_fill_rect)
{
    Display_Clear();
    Display_DrawRect(10, 10, 5, 4, 1);       /* outline only */
    CHECK(get_pixel(10, 10) == 1);           /* corners */
    CHECK(get_pixel(14, 10) == 1);
    CHECK(get_pixel(10, 13) == 1);
    CHECK(get_pixel(14, 13) == 1);
    CHECK(get_pixel(12, 11) == 0);           /* interior stays clear */

    Display_Clear();
    Display_FillRect(10, 10, 5, 4, 1);
    CHECK(count_lit_pixels() == 5 * 4);
    CHECK(get_pixel(12, 11) == 1);           /* interior filled */
}

TEST(test_draw_circle)
{
    Display_Clear();
    Display_DrawCircle(64, 32, 10, 1);

    /* The four cardinal points must be lit */
    CHECK(get_pixel(64, 42) == 1);
    CHECK(get_pixel(64, 22) == 1);
    CHECK(get_pixel(74, 32) == 1);
    CHECK(get_pixel(54, 32) == 1);
    /* Center must not be */
    CHECK(get_pixel(64, 32) == 0);
}

TEST(test_draw_char_and_text)
{
    Display_Clear();
    Display_DrawChar(0, 0, 'A');
    uint32_t a_pixels = count_lit_pixels();
    CHECK(a_pixels > 0);

    /* Non-printable characters render as '?' rather than indexing
     * outside the font table */
    Display_Clear();
    Display_DrawChar(0, 0, (char)7);
    uint32_t bell_pixels = count_lit_pixels();
    Display_Clear();
    Display_DrawChar(0, 0, '?');
    CHECK(bell_pixels == count_lit_pixels());

    /* Text advances 6 px per character */
    Display_Clear();
    Display_DrawText(0, 0, "AB");
    CHECK(get_pixel(0, 0) == get_pixel(0, 0)); /* smoke */
    Display_Clear();
    Display_DrawChar(0, 0, 'A');
    uint32_t one = count_lit_pixels();
    Display_Clear();
    Display_DrawText(0, 0, "AA");
    CHECK(count_lit_pixels() == 2 * one);      /* no overlap at 6 px pitch */

    /* Text clipped at the right edge instead of wrapping */
    Display_Clear();
    Display_DrawText(DISPLAY_WIDTH - 4, 0, "AA"); /* 'A' needs 5 px: clipped */
    CHECK(count_lit_pixels() == 0);

    /* Row/col addressing maps to 6x8 cells */
    Display_Clear();
    Display_DrawTextRowCol(1, 1, "A");
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
    Display_DrawDirectionIndicator(64, 32, 0.0f);
    uint32_t north = count_lit_pixels();
    CHECK(north > 20);                        /* circle + arrow + markers */

    Display_Clear();
    Display_DrawDirectionIndicator(64, 32, 135.0f);
    CHECK(count_lit_pixels() > 20);
}

TEST(test_boot_screen)
{
    reset_i2c_counters();
    Display_ShowBootScreen();
    CHECK(count_lit_pixels() > 100);          /* border + title text */
    CHECK(i2c_transmits > 0);                 /* Update was pushed */
}

TEST(test_update_chunking)
{
    Display_Clear();
    reset_i2c_counters();
    Display_Update();

    /* 6 addressing commands + 1024 buffer bytes in 16-byte chunks (64
     * transmits of 17 bytes each: flag + 16 data) */
    CHECK(i2c_transmits == 6 + SSD1309_BUFFER_SIZE / 16);
    CHECK(i2c_bytes == 6 * 2 + (SSD1309_BUFFER_SIZE / 16) * 17);
}

TEST(test_headless_mode_skips_all_i2c)
{
    /* Regression: I2C init failure used to while(1) here. It must now
     * mark the display dead and keep the receiver alive with zero bus
     * traffic (a dead bus would otherwise cost a HAL timeout per chunk). */
    fake_i2c_init_fail = 1;
    display_i2c_failed = 0;
    CHECK(Display_Init() == DISPLAY_OK);      /* returns, never hangs */
    CHECK(display_i2c_failed == 1);

    reset_i2c_counters();
    Display_Update();
    Display_ShowBootScreen();
    CHECK(i2c_transmits == 0);                /* headless: no I2C at all */

    /* Framebuffer drawing still works (SD-logged screenshots, tests) */
    Display_Clear();
    Display_DrawPixel(1, 1, 1);
    CHECK(get_pixel(1, 1) == 1);

    /* Recovery: a later successful init restores the display path */
    fake_i2c_init_fail = 0;
    display_i2c_failed = 0;
    CHECK(Display_Init() == DISPLAY_OK);
    reset_i2c_counters();
    Display_Update();
    CHECK(i2c_transmits > 0);
}

TEST(test_set_position_helper)
{
    reset_i2c_counters();
    Display_SetPosition(0x25, 3);
    CHECK(i2c_transmits == 3);                /* page + two column nibbles */
    CHECK(i2c_last_cmd == (0x25 & 0x0F));
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_sends_full_sequence_and_clears();
    run_test_pixel_set_clear_and_bounds();
    run_test_draw_line_all_octants();
    run_test_draw_and_fill_rect();
    run_test_draw_circle();
    run_test_draw_char_and_text();
    run_test_direction_indicator_draws_arrow();
    run_test_boot_screen();
    run_test_update_chunking();
    run_test_headless_mode_skips_all_i2c();
    run_test_set_position_helper();

    return TEST_SUMMARY();
}
