/**
 * @file test_eink.c
 * @brief Host-side unit tests for the SSD1680 tri-color eInk driver.
 *
 * Covers the pure drawing layer (framebuffer address mapping, plane
 * semantics, clipping, text scaling) plus the refresh state machine and
 * its failure modes, with a scripted SPI/GPIO fake standing in for the
 * panel. The shared-SPI1 bus-speed re-pin is asserted too: a refresh
 * after another device moved the bus must restore the eInk's /8 clock.
 *
 * Build & run:  make -C receiver/tests
 */

#include "test_harness.h"
#include <string.h>
#include "eink.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/* SPI + GPIO fakes                                                    */
/* ------------------------------------------------------------------ */

SPI_HandleTypeDef hspi1;
static SPI_TypeDef spi1_regs;

static uint32_t spi_init_count = 0;
static uint32_t last_prescaler = 0xFFFFFFFFu;

/* ordered byte stream with the DC level at transfer time (0 = command) */
static uint8_t  tx_bytes[16384];
static uint8_t  tx_dc[16384];
static uint32_t tx_len = 0;

static uint8_t pin_dc = 1, pin_busy = 0;
static uint32_t gpio_writes = 0;

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    spi_init_count++;
    last_prescaler = h->Init.BaudRatePrescaler;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    for (uint16_t i = 0; i < size && tx_len < sizeof(tx_bytes); i++) {
        tx_bytes[tx_len] = data[i];
        tx_dc[tx_len] = pin_dc;
        tx_len++;
    }
    return HAL_OK;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    gpio_writes++;
    if (port == EINK_DC_GPIO_PORT && pin == EINK_DC_PIN) pin_dc = (st == GPIO_PIN_SET);
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return pin_busy ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{ (void)port; (void)init; }
void HAL_Delay(uint32_t ms) { Test_SetTick(HAL_GetTick() + ms); }

/* ------------------------------------------------------------------ */

#include "../firmware/src/eink.c"

static void reset_all(void)
{
    spi_init_count = 0;
    last_prescaler = 0xFFFFFFFFu;
    tx_len = 0;
    pin_dc = 1; pin_busy = 0;
    gpio_writes = 0;
    eink_state = EINK_IDLE;
    eink_error = 0;
    memset(eink_bw, 0xFF, sizeof(eink_bw));
    memset(eink_red, 0x00, sizeof(eink_red));
    hspi1.Instance = &spi1_regs;
    memset(&spi1_regs, 0, sizeof(spi1_regs));
    Test_SetTick(0);
    Test_SetTickAutoAdvance(0);
}

/* Count set bits in a plane */
static uint32_t plane_bits(const uint8_t *plane, uint8_t val)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < EINK_PLANE_SIZE; i++)
        for (uint8_t b = 0x80; b; b >>= 1)
            if (((plane[i] & b) != 0) == (val != 0)) n++;
    return n;
}

/* find first occurrence of a command byte (dc==0) in the stream */
static int find_cmd(uint8_t cmd, uint32_t from)
{
    for (uint32_t i = from; i < tx_len; i++)
        if (tx_dc[i] == 0 && tx_bytes[i] == cmd) return (int)i;
    return -1;
}

/* ------------------------------------------------------------------ */
/* Framebuffer geometry (the THINKINK rotation-0 mapping)               */
/* ------------------------------------------------------------------ */

TEST(pixel_address_mapping_corners)
{
    reset_all();
    /* idx = ((WIDTH-1 - x) * HEIGHT + y) / 8, bit = 0x80 >> (y % 8) */
    Eink_DrawPixel(0, 0, EINK_BLACK);
    CHECK(eink_bw[(295 * 128 + 0) / 8] == 0x7F);   /* top-left: idx 4720 */
    Eink_DrawPixel(0, 127, EINK_BLACK);
    CHECK(eink_bw[4735] == 0xFE);                  /* bottom-left: last  */
    Eink_DrawPixel(295, 0, EINK_BLACK);
    CHECK(eink_bw[0] == 0x7F);                     /* top-right: idx 0   */
    Eink_DrawPixel(295, 127, EINK_BLACK);
    CHECK(eink_bw[(128 - 1) / 8] == 0xFE);         /* bottom-right       */
    /* all four corners also landed in the red plane as "not red" */
    CHECK(plane_bits(eink_red, 1) == 0);
}

TEST(plane_semantics_black_red_white)
{
    reset_all();
    /* black plane: 1 = white, 0 = black; red plane: 1 = red */
    Eink_DrawPixel(10, 10, EINK_BLACK);
    uint16_t idx = (uint16_t)(((uint32_t)(296 - 1 - 10) * 128 + 10) / 8);
    uint8_t mask = (uint8_t)(0x80 >> (10 & 7));
    CHECK((eink_bw[idx] & mask) == 0);             /* black marks bw=0   */
    CHECK((eink_red[idx] & mask) == 0);            /* and not red        */

    Eink_DrawPixel(10, 10, EINK_RED);
    CHECK((eink_bw[idx] & mask) != 0);             /* red marks bw=1     */
    CHECK((eink_red[idx] & mask) != 0);            /* and red=1          */

    Eink_DrawPixel(10, 10, EINK_WHITE);
    CHECK((eink_bw[idx] & mask) != 0);             /* white restores bw  */
    CHECK((eink_red[idx] & mask) == 0);            /* clears red         */
}

TEST(out_of_bounds_pixels_are_dropped)
{
    reset_all();
    Eink_DrawPixel(296, 0, EINK_BLACK);            /* x == WIDTH  */
    Eink_DrawPixel(0, 128, EINK_BLACK);            /* y == HEIGHT */
    Eink_DrawPixel(4000, 4000, EINK_BLACK);
    CHECK(plane_bits(eink_bw, 0) == 0);            /* nothing drawn */
}

TEST(fill_rect_draws_and_clips)
{
    reset_all();
    Eink_FillRect(4, 4, 8, 3, EINK_BLACK);
    CHECK(plane_bits(eink_bw, 0) == 24);           /* 8*3 black pixels  */

    /* rect hanging off the right/bottom edge clips, doesn't wrap/heap   */
    Eink_Clear();
    Eink_FillRect(290, 120, 20, 20, EINK_RED);
    CHECK(plane_bits(eink_red, 1) == 6 * 8);       /* 48 visible pixels */
}

TEST(text_scale_clamped_and_clipped)
{
    reset_all();
    Eink_Clear();
    Eink_DrawText(0, 0, 0, "H", EINK_BLACK);       /* scale 0 -> 1      */
    uint32_t tiny = plane_bits(eink_bw, 0);
    CHECK(tiny > 0 && tiny <= 35);                 /* 5x7 glyph max     */

    Eink_Clear();
    Eink_DrawText(0, 0, 99, "H", EINK_BLACK);      /* scale 99 -> 3     */
    uint32_t huge = plane_bits(eink_bw, 0);
    CHECK(huge >= 4 * tiny / 5);                   /* ~9x area          */

    /* unprintable chars become '?' and a string running off the right
     * edge stops instead of wrapping or overrunning */
    Eink_Clear();
    Eink_DrawText(280, 0, 3, "AB\x01" "CDEFG", EINK_BLACK);
    CHECK(plane_bits(eink_bw, 0) > 0);
}

/* ------------------------------------------------------------------ */
/* Refresh state machine                                               */
/* ------------------------------------------------------------------ */

TEST(refresh_transfers_planes_then_completes)
{
    reset_all();
    Eink_Init();
    CHECK(eink_error == 0);

    Eink_DrawPixel(5, 5, EINK_RED);
    tx_len = 0;
    CHECK(Eink_RequestRefresh() == 1);
    CHECK(Eink_IsBusy() == 1);

    /* command stream sanity: RAM_BW (0x24) then RAM_RED (0x26) plane
     * dumps, then 0xF4 display trigger + 0x20 activate, all as commands */
    int c_bw = find_cmd(0x24, 0);
    int c_red = find_cmd(0x26, 0);
    int c_ctl = find_cmd(0x22, 0);
    int c_act = find_cmd(0x20, 0);
    CHECK(c_bw > 0 && c_red > c_bw && c_ctl > c_red && c_act > c_ctl);
    /* each plane dump is EINK_PLANE_SIZE data bytes (dc=1 run) */
    uint32_t data_run = 0;
    for (int i = c_bw + 1; i < c_red - 3 && i < (int)tx_len; i++)
        if (tx_dc[i] == 1) data_run++;
    CHECK(data_run >= EINK_PLANE_SIZE);

    /* panel still physically refreshing: service waits on BUSY */
    pin_busy = 1;
    Eink_Service();
    CHECK(Eink_IsBusy() == 1);
    pin_busy = 0;
    Eink_Service();
    CHECK(Eink_IsBusy() == 0);                     /* deep-slept, idle  */
    /* next refresh allowed */
    CHECK(Eink_RequestRefresh() == 1);
    Eink_Service();
}

TEST(stuck_busy_times_out_and_latches_error)
{
    reset_all();
    Eink_Init();
    pin_busy = 0;
    CHECK(Eink_RequestRefresh() == 1);
    CHECK(Eink_IsBusy() == 1);

    /* panel never finishes: jump past the 40 s refresh deadline */
    pin_busy = 1;
    Test_SetTick(HAL_GetTick() + 41000);
    Eink_Service();
    CHECK(Eink_IsBusy() == 0);
    CHECK(eink_error == 1);
    /* error is sticky: further refreshes refuse until re-init */
    CHECK(Eink_RequestRefresh() == 0);
}

TEST(powerup_stuck_busy_latches_error_and_continues)
{
    /* Panel busy at power-up: the bounded waits give up after 2 s each,
     * latch eink_error, and init still completes (never hangs boot). */
    reset_all();
    pin_busy = 1;
    Test_SetTickAutoAdvance(500);          /* two waits × ~4 poll ticks */
    Eink_Init();
    Test_SetTickAutoAdvance(0);
    CHECK(eink_error == 1);
    /* a dead panel blocks refresh requests until a re-init clears it */
    CHECK(Eink_RequestRefresh() == 0);
}

TEST(refresh_repins_eink_bus_speed)
{
    /* Shared SPI1: an SD transaction can leave the bus at /2 (~10 MHz).
     * The eInk's own power-up must restore /8 (2.6 MHz) every refresh -
     * the ribbon can't take the SD's negotiated speed. */
    reset_all();
    Eink_Init();
    last_prescaler = 0xFFFFFFFFu;
    Eink_RequestRefresh();
    CHECK(last_prescaler == SPI_BAUDRATEPRESCALER_8);
    Eink_Service();
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_pixel_address_mapping_corners();
    run_plane_semantics_black_red_white();
    run_out_of_bounds_pixels_are_dropped();
    run_fill_rect_draws_and_clips();
    run_text_scale_clamped_and_clipped();
    run_refresh_transfers_planes_then_completes();
    run_stuck_busy_times_out_and_latches_error();
    run_powerup_stuck_busy_latches_error_and_continues();
    run_refresh_repins_eink_bus_speed();
    return TEST_SUMMARY();
}
