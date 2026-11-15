/**
 * test_main.c - host build of the receiver's main.c: boot and main loop
 *
 * main.c had no host build. Its boot sequence (watchdog, init retries,
 * degraded modes, the reset-cause and crash breadcrumbs) and the loop's
 * glue - which radio event becomes which SD row, button routing, the
 * display and e-ink cadence, the local-GPS breadcrumbs - were covered by
 * the firmware compile alone, while the modules it calls are host-tested.
 *
 * The real main.c runs here, renamed receiver_main(), over:
 *  - the real sd_card.c + lfs_sd_bd.c + LittleFS on a RAM card faked at the
 *    sd_diskio sector API, with the time a transfer takes at the 656 kHz
 *    data clock; assertions read the log back off the card, the way
 *    `make extract` does (block b at byte b * 4096);
 *  - the real logic modules it composes (beacon_track, rx_channel,
 *    log_policy, preflight_eval, eink_summary, math_utils, rf_parser);
 *  - fakes at the API of the hardware-facing modules (rf_receiver, gps,
 *    compass, display, eink, button, the display pages) that keep those
 *    modules' contracts: one-shot getters consume, the GPS fix latch
 *    expires after 5 s of silence, the quat lock takes only 1..8, a button
 *    event is read once.
 *
 * Each boot runs in its own process (fork), so main()'s one-shot statics
 * start fresh, as a reset re-runs .data/.bss init. The card and the crash
 * record live in shared memory and survive into the next boot, the way the
 * SD card and the NOLOAD .crash_record RAM do. Inside a boot the firmware
 * runs on its own thread and hands control back to the script whenever
 * the simulated clock reaches the point the script asked for, so scripts
 * read top to bottom: inject, run, check.
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 */
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "test_harness.h"

/* The firmware under test; its main() becomes receiver_main() */
#define main receiver_main
#include "../firmware/src/main.c"
#undef main

#include "lfs.h"
#include "lfs_sd_bd.h"

/* ------------------------------------------------------------------ */
/* State that survives a reset: the card and the crash-record RAM      */
/* ------------------------------------------------------------------ */

#define CARD_SECTORS (8u * 2048u)          /* 8 MiB card: 2048 LittleFS blocks */
#define CARD_BLOCKS  (CARD_SECTORS / 8u)

static struct shared {
    int         run, failed;               /* CHECK counts from boot processes */
    uint8_t     reset_seen;                /* the last boot ended in a reset   */
    uint8_t     crash_valid;               /* crash RAM carried into next boot */
    CrashRecord crash;
    uint8_t     card[CARD_SECTORS * 512u];
} *sh;

/* ------------------------------------------------------------------ */
/* Simulated clock and the independent watchdog                        */
/* ------------------------------------------------------------------ */

IWDG_TypeDef Test_IWDG;
RCC_TypeDef  Test_RCC;
SCB_Type     Test_SCB;

static uint8_t  iwdg_running = 0;
static uint32_t iwdg_last_feed = 0, iwdg_max_gap = 0, iwdg_feeds = 0;

/* Time only moves in HAL_Delay, card transfers and between loop passes,
 * so a start or feed written since the last step happened "now". */
static void sim_advance(uint32_t ms)
{
    if (Test_IWDG.KR == 0xCCCCu) iwdg_running = 1;
    if (Test_IWDG.KR == 0xCCCCu || Test_IWDG.KR == 0xAAAAu) {
        iwdg_last_feed = fake_tick_ms;
        iwdg_feeds++;
        Test_IWDG.KR = 0;
    }
    fake_tick_ms += ms;
    if (iwdg_running && fake_tick_ms - iwdg_last_feed > iwdg_max_gap)
        iwdg_max_gap = fake_tick_ms - iwdg_last_feed;
}

/* IWDG timeout at the fastest LSI the STM32F401 datasheet allows (47 kHz;
 * typical 32 kHz): the longest gap between feeds must stay under this. */
static uint32_t iwdg_worst_timeout_ms(void)
{
    return (uint32_t)(((uint64_t)(Test_IWDG.RLR + 1u) * (4u << Test_IWDG.PR)) / 47u);
}

/* ------------------------------------------------------------------ */
/* Firmware thread <-> script handoff                                  */
/* ------------------------------------------------------------------ */

static pthread_mutex_t sim_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  sim_cv = PTHREAD_COND_INITIALIZER;
static int      fw_turn = 0;               /* 1: the firmware thread runs */
static uint32_t sim_until = 0;             /* firmware yields at this tick */
static uint32_t loop_ms = 10;              /* one main-loop pass           */
static uint32_t loops = 0, loop_entry_tick = 0;
static uint8_t  in_loop = 0;
static void   (*on_pass)(void) = NULL;     /* script hook, every pass      */

static void fw_yield(void)
{
    pthread_mutex_lock(&sim_mu);
    fw_turn = 0;
    pthread_cond_broadcast(&sim_cv);
    while (!fw_turn) pthread_cond_wait(&sim_cv, &sim_mu);
    pthread_mutex_unlock(&sim_mu);
}

static void sim_resume(void)
{
    pthread_mutex_lock(&sim_mu);
    fw_turn = 1;
    pthread_cond_broadcast(&sim_cv);
    while (fw_turn) pthread_cond_wait(&sim_cv, &sim_mu);
    pthread_mutex_unlock(&sim_mu);
}

static void *fw_thread(void *arg)
{
    (void)arg;
    receiver_main();                       /* never returns */
    return NULL;
}

/* Power on: boot runs until the first main-loop pass (or a reset) */
static void sim_start(void)
{
    pthread_t t;
    sim_until = 0;
    pthread_mutex_lock(&sim_mu);
    fw_turn = 1;
    pthread_mutex_unlock(&sim_mu);
    pthread_create(&t, NULL, fw_thread, NULL);
    pthread_detach(t);
    pthread_mutex_lock(&sim_mu);
    while (fw_turn) pthread_cond_wait(&sim_cv, &sim_mu);
    pthread_mutex_unlock(&sim_mu);
}

static void run_ms(uint32_t ms)
{
    sim_until = fake_tick_ms + ms;
    sim_resume();
}

/* One main-loop pass reached Button_Update (called once per pass) */
static void sim_loop_pass(void)
{
    if (!in_loop) { in_loop = 1; loop_entry_tick = fake_tick_ms; }
    loops++;
    if (fake_tick_ms >= sim_until) fw_yield();
    if (on_pass) on_pass();
    sim_advance(loop_ms);
}

/* ------------------------------------------------------------------ */
/* HAL, clock tree, ADC, CMSIS fakes                                   */
/* ------------------------------------------------------------------ */

static RCC_OscInitTypeDef osc_cfg;
static RCC_ClkInitTypeDef clk_cfg;
static uint32_t           clk_latency;
static HAL_StatusTypeDef  osc_status = HAL_OK, clk_status = HAL_OK, spi_status = HAL_OK;
static SPI_HandleTypeDef  spi_seen;

HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_InitTick(uint32_t prio) { (void)prio; return HAL_OK; }
void HAL_IncTick(void) { fake_tick_ms++; }
void HAL_Delay(uint32_t ms) { sim_advance(ms); }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc) { osc_cfg = *osc; return osc_status; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t lat)
{
    clk_cfg = *clk;
    clk_latency = lat;
    return clk_status;
}
/* The real HAL_SPI_Init runs the MSP init (pins, clock) for a fresh handle */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    spi_seen = *h;
    HAL_SPI_MspInit(h);
    return spi_status;
}

static uint8_t  led_on = 0;                /* PC13 is active LOW */
static uint32_t led_pulses = 0;
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    if (port == LED_GPIO_PORT && pin == LED_PIN) {
        uint8_t on = (st == GPIO_PIN_RESET);
        if (on && !led_on) led_pulses++;
        led_on = on;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return GPIO_PIN_RESET; }

/* Battery divider ADC: a script of raw counts, and which conversions time out */
static struct {
    HAL_StatusTypeDef init, config;
    uint16_t raw[8];
    uint8_t  timeout_mask;                 /* bit i: conversion i times out */
    uint8_t  conv, started, deinits;
    uint32_t channel, sampling;
} adc;
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) { (void)h; adc.conv = 0; return adc.init; }
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *h) { (void)h; adc.deinits++; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
    (void)h;
    adc.channel = c->Channel;
    adc.sampling = c->SamplingTime;
    return adc.config;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h) { (void)h; adc.started++; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t t)
{
    (void)h; (void)t;
    return (adc.timeout_mask & (1u << (adc.conv & 7u))) ? (adc.conv++, HAL_TIMEOUT) : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h) { (void)h; return adc.raw[adc.conv++ & 7u]; }

uint32_t __get_MSP(void) { return 0x2000FF80u; }
uint32_t __get_PSP(void) { return 0x2000F000u; }

/* A reset: the crash RAM survives into the next boot, the firmware stops */
void NVIC_SystemReset(void)
{
    sh->crash = g_crash;
    sh->crash_valid = 1;
    sh->reset_seen = 1;
    for (;;) fw_yield();
}

/* ------------------------------------------------------------------ */
/* sd_diskio fakes: the card, with transfer time at the data clock     */
/* ------------------------------------------------------------------ */

uint8_t SD_Type = 0;
uint8_t sd_diag_cmd_resp = 0, sd_diag_data_token = 0, sd_diag_wait_fail = 0;
static struct {
    uint8_t  absent;                       /* no card answers CMD0          */
    uint8_t  read_only;                    /* every write gets an error token */
    uint8_t  div;                          /* SPI prescaler in effect       */
    uint32_t stepdowns, writes, write_errors;
} sd;

#define SD_READ_MS_PER_SECTOR   7u         /* ~530 bytes at 656 kHz */
#define SD_WRITE_MS_PER_SECTOR  8u         /* plus the card's busy time */

DSTATUS SD_initialize(BYTE pdrv)
{
    (void)pdrv;
    sim_advance(200);
    SD_Type = sd.absent ? 0 : 12;
    return sd.absent ? STA_NOINIT : 0;
}
DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    if (cmd != GET_SECTOR_COUNT) return RES_PARERR;
    *(DWORD *)buff = CARD_SECTORS;
    return RES_OK;
}
void    SD_SetFastSpeed(void) { sd.div = 16; }
uint8_t SD_FastSpeedEnabled(void) { return sd.div; }
uint8_t SD_StepSpeedDown(void) { sd.stepdowns++; if (sd.div < 128) sd.div *= 2; return sd.div; }
uint8_t SD_StepSpeedUpTry(void) { if (sd.div > 16) sd.div /= 2; return sd.div; }
DRESULT SD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    sim_advance(SD_READ_MS_PER_SECTOR * count);
    if (sector + count > CARD_SECTORS) return RES_ERROR;
    memcpy(buff, &sh->card[sector * 512u], count * 512u);
    return RES_OK;
}
DRESULT SD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    sim_advance(SD_WRITE_MS_PER_SECTOR * count);
    sd.writes++;
    sd_diag_cmd_resp = 0x00;
    if (sd.read_only || sector + count > CARD_SECTORS) {
        sd.write_errors++;
        sd_diag_data_token = 0x0D;          /* data response: write error */
        return RES_ERROR;
    }
    sd_diag_data_token = 0x05;
    memcpy(&sh->card[sector * 512u], buff, count * 512u);
    return RES_OK;
}

/* ------------------------------------------------------------------ */
/* Display, e-ink, buttons: capture what the operator would see        */
/* ------------------------------------------------------------------ */

#define SCR_ROWS 20
#define SCR_COLS 40
static char     scr[SCR_ROWS][SCR_COLS + 1];
static char     frames[1 << 16];            /* every distinct frame shown */
static size_t   frames_len = 0;
static char     last_frame[SCR_ROWS * (SCR_COLS + 2) + 1];
static uint32_t disp_updates = 0, test_patterns = 0;

uint8_t Display_Init(void) { return 1; }
void Display_Clear(void) { memset(scr, 0, sizeof(scr)); }
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *t)
{
    if (row >= SCR_ROWS) return;
    for (uint8_t c = 0; c < col && c < SCR_COLS; c++)
        if (!scr[row][c]) scr[row][c] = ' ';
    for (size_t i = 0; t[i] && col + i < SCR_COLS; i++) scr[row][col + i] = t[i];
}
void Display_Update(void)
{
    char f[sizeof(last_frame)];
    size_t n = 0;
    disp_updates++;
    for (int r = 0; r < SCR_ROWS; r++) {
        if (!scr[r][0]) continue;
        n += (size_t)snprintf(f + n, sizeof(f) - n, "%s|", scr[r]);
    }
    f[n] = '\0';
    if (strcmp(f, last_frame) == 0) return;
    strcpy(last_frame, f);
    if (frames_len + n + 2 < sizeof(frames)) {
        memcpy(frames + frames_len, f, n);
        frames_len += n;
        frames[frames_len++] = '\n';
        frames[frames_len] = '\0';
    }
}
void Display_ShowTestPattern(void) { test_patterns++; }
static int shown(const char *text) { return strstr(frames, text) != NULL; }

static struct {
    uint8_t  busy, unwired;
    uint32_t refreshes, last_refresh_tick, inits;
    char     page[512], shown[512];
} ink;
uint8_t Eink_Init(void) { ink.inits++; return 1; }
void    Eink_Clear(void) { ink.page[0] = '\0'; }
void    Eink_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c)
{ (void)x; (void)y; (void)w; (void)h; (void)c; }
void    Eink_DrawText(uint16_t x, uint16_t y, uint8_t s, const char *t, uint8_t c)
{
    (void)x; (void)y; (void)s; (void)c;
    size_t n = strlen(ink.page);
    snprintf(ink.page + n, sizeof(ink.page) - n, "%s|", t);
}
uint8_t Eink_RequestRefresh(void)
{
    ink.refreshes++;
    ink.last_refresh_tick = fake_tick_ms;
    strcpy(ink.shown, ink.page);
    return 1;
}
uint8_t Eink_IsBusy(void) { return ink.busy; }
uint8_t Eink_BusyUnwired(void) { return ink.unwired; }
void    Eink_Service(void) {}

/* Button events are one-shot, as button.c's flags clear on read */
static struct { uint8_t b1, b1_long, b2, b2_held; } btn;
void    Button_Init(void) {}
void    Button_Update(void) { sim_loop_pass(); }
uint8_t Button_WasPressed(void) { uint8_t v = btn.b1; btn.b1 = 0; return v; }
uint8_t Button_WasLongPressed(void) { uint8_t v = btn.b1_long; btn.b1_long = 0; return v; }
uint8_t Button2_WasPressed(void) { uint8_t v = btn.b2; btn.b2 = 0; return v; }
uint8_t Button2_IsHeld(void) { return btn.b2_held; }

void I2CBus_Init(void) {}

/* ------------------------------------------------------------------ */
/* Local GPS: a 1 Hz sentence stream with gps.c's fix latch            */
/* ------------------------------------------------------------------ */

static struct {
    uint8_t  init_ok, streaming;
    GPS_Data fix;                          /* what the next sentence carries */
    uint32_t last_sentence, update_calls;
    uint8_t  last_fix;
    uint32_t bytes, sentences, drops, rearm, csum;
    uint16_t uart_errs;
} gps;
uint8_t GPS_Init(void) { return gps.init_ok ? GPS_OK : GPS_ERROR; }
uint8_t GPS_Update(GPS_Data *d)
{
    gps.update_calls++;
    if (!gps.streaming || (gps.last_sentence && fake_tick_ms - gps.last_sentence < 1000u))
        return GPS_ERROR;                  /* no new sentence queued */
    gps.last_sentence = fake_tick_ms;
    *d = gps.fix;                          /* the parser writes fix even for no-fix */
    gps.last_fix = d->fix ? 1 : 0;
    return d->fix ? GPS_OK : GPS_ERROR;
}
uint8_t GPS_IsFixed(void)                  /* expires 5 s after the stream stops */
{
    if (fake_tick_ms - gps.last_sentence > 5000u) return 0;
    return gps.last_fix;
}
void GPS_GetRxStats(uint32_t *b, uint32_t *s, uint32_t *d, uint16_t *e, uint32_t *r)
{ *b = gps.bytes; *s = gps.sentences; *d = gps.drops; *e = gps.uart_errs; *r = gps.rearm; }
uint32_t GPS_Parser_GetChecksumErrors(void) { return gps.csum; }

static void gps_fix_at(float lat, float lon, uint8_t sats)
{
    memset(&gps.fix, 0, sizeof(gps.fix));
    gps.fix.latitude = lat;
    gps.fix.longitude = lon;
    gps.fix.altitude = 1650.0f;
    gps.fix.satellites = sats;
    gps.fix.fix = 1;
    gps.fix.hour = 12; gps.fix.minute = 34; gps.fix.second = 56;
    gps.streaming = 1;
}

/* ------------------------------------------------------------------ */
/* Compass: compass.c's contracts at its API                           */
/* ------------------------------------------------------------------ */

static struct {
    uint8_t      init_ok, update_ok, dev_count, addrs[8];
    Compass_Data data;
    uint8_t      err_code;
    const char  *err_msg;
    int8_t       quat;                     /* the convention lock, 0 = none */
    uint8_t      loc_applied;
    uint32_t     loc_calls, decl_calls, scan_shows, quat_sets;
    float        decl;
} cmp;
uint8_t Compass_Init(void) { return cmp.init_ok ? COMPASS_OK : COMPASS_ERROR; }
uint8_t Compass_Update(Compass_Data *d)
{
    if (!cmp.update_ok) return COMPASS_ERROR;
    *d = cmp.data;
    return COMPASS_OK;
}
uint8_t Compass_DisplayI2CScan(void) { cmp.scan_shows++; return cmp.dev_count; }
uint8_t Compass_GetFoundDeviceCount(void) { return cmp.dev_count; }
const uint8_t *Compass_GetFoundAddresses(void) { return cmp.addrs; }
uint8_t Compass_GetErrorCode(void) { return cmp.err_code; }
const char *Compass_GetErrorMessage(void) { return cmp.err_msg ? cmp.err_msg : ""; }
uint8_t Compass_SetDeclination(float d) { cmp.decl = d; cmp.decl_calls++; return COMPASS_OK; }
uint8_t Compass_UpdateLocation(float lat, float lon)
{
    cmp.loc_calls++;
    if (cmp.loc_applied) return COMPASS_OK;
    if (lat < 18.0f || lat > 50.0f || lon < -130.0f || lon > -60.0f) return COMPASS_ERROR;
    cmp.loc_applied = 1;
    return COMPASS_OK;
}
uint8_t Compass_GetQuatLockConv(void) { return (uint8_t)cmp.quat; }
uint8_t Compass_SetQuatLockConv(uint8_t c)
{
    cmp.quat_sets++;
    if (c < 1 || c > 8) return COMPASS_ERROR;
    cmp.quat = (int8_t)c;
    return COMPASS_OK;
}

/* ------------------------------------------------------------------ */
/* Display pages: each draws its tag; two record what they were given  */
/* ------------------------------------------------------------------ */

static int page_drawn = -1;
static struct { uint8_t local_ok, beacon_ok, verdict; uint32_t pkts; } nav_page;
static struct { uint8_t hb_state, alt_hb, sd_ok, link_ok; uint16_t vdd; } pre_page;
static void page(int p, const char *tag) { page_drawn = p; Display_DrawTextRowCol(0, 0, tag); }

void DisplayMode_Navigation(uint8_t lg, uint8_t rg, GPS_Data *l, GPS_Data *r, float h,
                            uint32_t t, uint32_t n, const Preflight_Result_t *pf)
{
    (void)l; (void)r; (void)h; (void)t;
    nav_page.local_ok = lg; nav_page.beacon_ok = rg; nav_page.pkts = n;
    nav_page.verdict = pf->verdict;
    page(DISPLAY_MODE_NAVIGATION, "[NAV]");
}
void DisplayMode_GPS(GPS_Data *g, GPS_Data *lg, uint8_t h)
{ (void)g; (void)lg; (void)h; page(DISPLAY_MODE_GPS, "[GPS]"); }
void DisplayMode_RF(GPS_Data *g, uint8_t *v, uint32_t *t, uint32_t *n, GPS_Data *lg, uint8_t h)
{ (void)g; (void)v; (void)t; (void)n; (void)lg; (void)h; page(DISPLAY_MODE_RF, "[RF]"); }
void DisplayMode_CompassVisual(Compass_Data *c, float *h)
{ (void)c; (void)h; page(DISPLAY_MODE_COMPASS_VISUAL, "[CVIS]"); }
void DisplayMode_CompassHeading(Compass_Data *c, float *h)
{ (void)c; (void)h; page(DISPLAY_MODE_COMPASS_HEADING, "[CHDG]"); }
void DisplayMode_SDCard(void) { page(DISPLAY_MODE_SD_CARD, "[SD]"); }
void DisplayMode_ChannelSelect(void) { page(DISPLAY_MODE_CHANNEL, "[CHAN]"); }
void DisplayMode_RFStats(void) { page(DISPLAY_MODE_RF_STATS, "[STATS]"); }
void DisplayMode_Altimeter(void) { page(DISPLAY_MODE_ALTIMETER, "[ALT]"); }
void DisplayMode_Preflight(uint8_t link_ok, uint32_t link_age_s, int16_t rssi, uint8_t tx_fix,
                           uint8_t tx_sats, uint8_t hb_state, uint8_t degraded,
                           uint8_t rx_fix, uint8_t rx_sats, uint8_t cmp_ok, uint8_t sd_ok,
                           uint8_t alt_hb, uint16_t vdd, const Preflight_Result_t *pf)
{
    (void)link_age_s; (void)rssi; (void)tx_fix; (void)tx_sats; (void)degraded;
    (void)rx_fix; (void)rx_sats; (void)cmp_ok; (void)pf;
    pre_page.link_ok = link_ok; pre_page.hb_state = hb_state; pre_page.alt_hb = alt_hb;
    pre_page.sd_ok = sd_ok; pre_page.vdd = vdd;
    page(DISPLAY_MODE_PREFLIGHT, "[PRE]");
}

/* ------------------------------------------------------------------ */
/* rf_receiver: scripted state behind its API                          */
/* ------------------------------------------------------------------ */

static struct {
    uint8_t  init_fails, init_code;        /* next N inits fail with init_code */
    uint32_t init_calls, init_tick[4];
    uint8_t  channel, scanning, lock_pending;
    uint32_t scan_starts, scan_from_starts, scan_start_tick;
    uint8_t  sweep_channels;               /* channels NoiseSweep measures */
    uint32_t sweep_tick[LORA_CHANNEL_COUNT], sweeps;
    uint32_t heard_tick;
    int16_t  rssi;
    int8_t   snr;
    uint8_t  latched, bound, testing;
    uint32_t suppressions, wedges, foreign, pings, services;
    uint8_t  hb_new;                       /* one-shot heartbeat */
    HeartbeatPacket_t hb;
    RF_HbInfo_t hbi;
    uint8_t  last_hb_valid;
    uint32_t last_hb_tick;
    uint8_t  t0_new, imu_new, imu_fmt, mx_new, alt_new, hello_new, wreport_new, cmd_new;
    LaunchT0Packet_t  t0;
    ImuTracePacket_t  imu;
    MaximaPacket_t    mx;
    MaximaAltPacket_t alt;
    HelloPacket_t     hello;
    uint32_t wr_count, wr_last, wr_total;
    RF_CmdResult_t cmd;
    FlightEventPacket_t evq[8];
    uint8_t  evq_n;
    uint8_t  crc_len, crc_total, crc_dump[8];
    uint8_t  noise, nf_valid, nfl_valid;
    int16_t  nf, nfl;
    uint32_t irqs, pkts, dups, crc_errors;
    GPS_Data pktq[8];
    uint8_t  pktq_n;
    char     callsign[RF_PARSER_MAX_CALLSIGN_LEN];
} rf;

uint8_t RF_Receiver_Init(void)
{
    if (rf.init_calls < 4) rf.init_tick[rf.init_calls] = fake_tick_ms;
    rf.init_calls++;
    if (rf.init_fails) { rf.init_fails--; return rf.init_code; }
    return RF_OK;
}
void    RF_Receiver_StartScan(void) { rf.scanning = 1; rf.scan_starts++; rf.scan_start_tick = fake_tick_ms; }
void    RF_Receiver_StartScanFrom(uint8_t ch)
{
    rf.scanning = 1; rf.channel = ch; rf.scan_from_starts++; rf.scan_start_tick = fake_tick_ms;
}
void    RF_Receiver_StopScan(void) { rf.scanning = 0; }
uint8_t RF_Receiver_IsScanning(void) { return rf.scanning; }
uint8_t RF_Receiver_ScanUpdate(void)
{
    if (!rf.scanning || !rf.lock_pending) return 0;
    rf.scanning = rf.lock_pending = 0;
    return 1;
}
uint8_t RF_Receiver_GetChannel(void) { return rf.channel; }
/* A channel change is a new airframe context: nothing heard, nothing bound */
uint8_t RF_Receiver_NextChannel(void)
{
    rf.channel = (uint8_t)((rf.channel + 1u) % LORA_CHANNEL_COUNT);
    rf.heard_tick = 0;
    rf.bound = 0xFF;
    rf.pktq_n = 0;
    return rf.channel;
}
uint8_t RF_Receiver_NoiseSweep(int16_t *nf_out)
{
    rf.sweeps++;
    for (uint8_t ch = 0; ch < rf.sweep_channels; ch++) {
        sim_advance(125);                  /* ~1 s for all eight channels */
        rf.sweep_tick[ch] = fake_tick_ms;
        nf_out[ch] = (int16_t)(-120 + ch);
    }
    return rf.sweep_channels;
}
uint32_t RF_Receiver_GetSweepTick(uint8_t ch) { return rf.sweep_tick[ch]; }
uint32_t RF_Receiver_GetBeaconHeardTick(void) { return rf.heard_tick; }
void     RF_Receiver_GetSignalQuality(int16_t *r, int8_t *s) { *r = rf.rssi; *s = rf.snr; }
uint8_t  RF_Receiver_FlightContactLatched(void) { return rf.latched; }
uint32_t RF_Receiver_GetRescanSuppressions(void) { return rf.suppressions; }
uint32_t RF_Receiver_GetWedgesRecovered(void) { return rf.wedges; }
uint8_t  RF_Receiver_GetBoundRocketId(void) { return rf.bound; }
uint32_t RF_Receiver_GetForeignDrops(void) { return rf.foreign; }
void     RF_Receiver_RequestPing(void) { rf.pings++; }
void     RF_Receiver_ServiceCommands(void) { rf.services++; }
uint8_t  RF_Receiver_TestingBuildSuspect(void) { return rf.testing; }
uint8_t  RF_Receiver_GetHeartbeat(HeartbeatPacket_t *hb, RF_HbInfo_t *info)
{
    if (!rf.hb_new) return 0;
    rf.hb_new = 0;
    *hb = rf.hb;
    *info = rf.hbi;
    return 1;
}
uint8_t  RF_Receiver_GetLastHeartbeat(HeartbeatPacket_t *hb, uint32_t *age_ms)
{
    if (!rf.last_hb_valid) return 0;
    if (hb) *hb = rf.hb;
    if (age_ms) *age_ms = fake_tick_ms - rf.last_hb_tick;
    return 1;
}
#define ONE_SHOT(flag, dst, src) do { if (!(flag)) return 0; (flag) = 0; *(dst) = (src); return 1; } while (0)
uint8_t RF_Receiver_GetLaunchT0(LaunchT0Packet_t *t) { ONE_SHOT(rf.t0_new, t, rf.t0); }
uint8_t RF_Receiver_GetMaxima(MaximaPacket_t *m) { ONE_SHOT(rf.mx_new, m, rf.mx); }
uint8_t RF_Receiver_GetAltimeter(MaximaAltPacket_t *a) { ONE_SHOT(rf.alt_new, a, rf.alt); }
uint8_t RF_Receiver_GetHello(HelloPacket_t *h) { ONE_SHOT(rf.hello_new, h, rf.hello); }
uint8_t RF_Receiver_TakeCommandResult(RF_CmdResult_t *c) { ONE_SHOT(rf.cmd_new, c, rf.cmd); }
uint8_t RF_Receiver_GetImuTrace(ImuTracePacket_t *imu, uint8_t *fmt)
{
    if (!rf.imu_new) return 0;
    rf.imu_new = 0;
    *imu = rf.imu;
    if (fmt) *fmt = rf.imu_fmt;
    return 1;
}
uint8_t RF_Receiver_GetFlightEvent(FlightEventPacket_t *e)
{
    if (!rf.evq_n) return 0;
    *e = rf.evq[0];
    memmove(rf.evq, rf.evq + 1, (size_t)(--rf.evq_n) * sizeof(rf.evq[0]));
    return 1;
}
uint8_t RF_Receiver_TakeWedgeReport(uint32_t *c, uint32_t *l, uint32_t *t)
{
    if (!rf.wreport_new) return 0;
    rf.wreport_new = 0;
    *c = rf.wr_count; *l = rf.wr_last; *t = rf.wr_total;
    return 1;
}
uint8_t RF_Receiver_TakeCrcDump(uint8_t *out, uint8_t max, uint8_t *total)
{
    uint8_t n = rf.crc_len < max ? rf.crc_len : max;
    memcpy(out, rf.crc_dump, n);
    *total = rf.crc_total;
    rf.crc_len = 0;
    return n;
}
uint8_t  RF_Receiver_NoiseAlert(void) { return rf.noise; }
uint8_t  RF_Receiver_GetNoiseFloor(int16_t *nf) { *nf = rf.nf; return rf.nf_valid; }
uint8_t  RF_Receiver_GetLiveNoiseFloor(int16_t *nf) { *nf = rf.nfl; return rf.nfl_valid; }
void     RF_Receiver_GetPacketLossDiagnostics(uint32_t *i, uint32_t *p, uint32_t *d)
{ *i = rf.irqs; *p = rf.pkts; *d = rf.dups; }
uint32_t RF_Receiver_GetCrcErrors(void) { return rf.crc_errors; }
uint8_t  RF_Receiver_DataAvailable(void) { return rf.pktq_n > 0; }
uint8_t  RF_Receiver_GetGPSData(GPS_Data *g)
{
    if (!rf.pktq_n) return RF_ERROR;
    *g = rf.pktq[0];
    memmove(rf.pktq, rf.pktq + 1, (size_t)(--rf.pktq_n) * sizeof(rf.pktq[0]));
    return RF_OK;
}
uint8_t  RF_Receiver_GetParsedData(GPS_Data *g, char *cs, uint16_t n, uint8_t *chk)
{
    (void)g; (void)chk;
    snprintf(cs, n, "%s", rf.callsign);
    return 1;
}

/* A position packet from our beacon, heard now */
static void rf_position(float lat, float lon, int16_t rssi)
{
    GPS_Data *p = &rf.pktq[rf.pktq_n++];
    memset(p, 0, sizeof(*p));
    p->latitude = lat;
    p->longitude = lon;
    p->altitude = 1700.0f;
    p->satellites = 9;
    p->fix = 1;
    rf.rssi = rssi;
    rf.snr = 7;
    rf.heard_tick = fake_tick_ms;
}

/* ------------------------------------------------------------------ */
/* Reading the card back, as `make extract` does                       */
/* ------------------------------------------------------------------ */

static int rd_read(const struct lfs_config *c, lfs_block_t b, lfs_off_t off, void *buf, lfs_size_t n)
{ (void)c; memcpy(buf, &sh->card[b * 4096u + off], n); return 0; }
static int rd_prog(const struct lfs_config *c, lfs_block_t b, lfs_off_t off, const void *buf, lfs_size_t n)
{ (void)c; memcpy(&sh->card[b * 4096u + off], buf, n); return 0; }
static int rd_erase(const struct lfs_config *c, lfs_block_t b) { (void)c; (void)b; return 0; }
static int rd_sync(const struct lfs_config *c) { (void)c; return 0; }
static uint8_t rd_rbuf[4096], rd_pbuf[4096], rd_look[128], rd_fbuf[4096];
static const struct lfs_config rd_cfg = {
    .read = rd_read, .prog = rd_prog, .erase = rd_erase, .sync = rd_sync,
    .read_size = 512, .prog_size = 512, .block_size = 4096, .block_count = CARD_BLOCKS,
    .block_cycles = 500, .cache_size = 4096, .lookahead_size = 128,
    .read_buffer = rd_rbuf, .prog_buffer = rd_pbuf, .lookahead_buffer = rd_look,
};
static const struct lfs_file_config rd_fcfg = { .buffer = rd_fbuf };

/* Contents of one file on the card ("" if the card or the file is absent) */
static char filebuf[256 * 1024];
static const char *card_file(const char *name)
{
    lfs_t fs;
    lfs_file_t f;
    filebuf[0] = '\0';
    if (lfs_mount(&fs, &rd_cfg) != 0) return filebuf;
    if (lfs_file_opencfg(&fs, &f, name, LFS_O_RDONLY, &rd_fcfg) == 0) {
        lfs_ssize_t n = lfs_file_read(&fs, &f, filebuf, sizeof(filebuf) - 1);
        filebuf[n > 0 ? n : 0] = '\0';
        lfs_file_close(&fs, &f);
    }
    lfs_unmount(&fs);
    return filebuf;
}

static int card_has_file(const char *name)
{
    lfs_t fs;
    struct lfs_info info;
    if (lfs_mount(&fs, &rd_cfg) != 0) return 0;
    int found = (lfs_stat(&fs, name, &info) == 0);
    lfs_unmount(&fs);
    return found;
}

/* The running session's log, committed first through the firmware's own
 * idle-sync path (the operator's walk back to the car does the same) */
static const char *session_log(void)
{
    SD_Card_Info info;
    sim_advance(500);
    SD_Card_ServiceSync(1);
    if (SD_Card_GetInfo(&info) != SD_CARD_OK || !info.current_log_file[0]) return "";
    return card_file(info.current_log_file);
}

static int count_in(const char *hay, const char *needle)
{
    int n = 0;
    for (const char *p = hay; (p = strstr(p, needle)) != NULL; p += strlen(needle)) n++;
    return n;
}

/* Field k (0-based CSV) of the n-th row (0-based) containing `kind` */
static double row_field(const char *log, const char *kind, int n, int k)
{
    const char *p = log;
    for (int i = 0; p && i <= n; i++) {
        p = strstr(p, kind);
        if (p && i < n) p += strlen(kind);
    }
    if (!p) return -9999.0;
    while (p > log && p[-1] != '\n') p--;  /* start of that row */
    for (int i = 0; i < k && p; i++) {
        p = strchr(p, ',');
        if (p) p++;
    }
    return p ? atof(p) : -9999.0;
}

/* Put a file on the card before a boot, the way a PC would (format first) */
static void card_put(const char *name, const void *data, size_t len)
{
    lfs_t fs;
    lfs_file_t f;
    if (lfs_mount(&fs, &rd_cfg) != 0) {
        lfs_format(&fs, &rd_cfg);
        lfs_mount(&fs, &rd_cfg);
    }
    lfs_file_opencfg(&fs, &f, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &rd_fcfg);
    lfs_file_write(&fs, &f, data, (lfs_size_t)len);
    lfs_file_close(&fs, &f);
    lfs_unmount(&fs);
}

/* ------------------------------------------------------------------ */
/* Boots                                                               */
/* ------------------------------------------------------------------ */

/* A healthy board on the bench: every part answers, nothing heard yet */
static void board_defaults(void)
{
    osc_status = clk_status = spi_status = HAL_OK;
    on_pass = NULL;
    memset(&adc, 0, sizeof(adc));
    for (int i = 0; i < 8; i++) adc.raw[i] = 3100;     /* 3255 mV pack */
    memset(&sd, 0, sizeof(sd));
    memset(&ink, 0, sizeof(ink));
    memset(&btn, 0, sizeof(btn));
    memset(&gps, 0, sizeof(gps));
    gps.init_ok = 1;
    memset(&cmp, 0, sizeof(cmp));
    cmp.init_ok = cmp.update_ok = 1;
    cmp.data.heading = 90.0f;
    memset(&rf, 0, sizeof(rf));
    rf.init_code = RF_ERROR;
    rf.bound = 0xFF;
    rf.sweep_channels = LORA_CHANNEL_COUNT;
    rf.rssi = -90;
    rf.snr = 5;
    memset(&Test_IWDG, 0, sizeof(Test_IWDG));
    memset(&Test_SCB, 0, sizeof(Test_SCB));
    Test_RCC.CSR = RCC_CSR_PORRSTF | RCC_CSR_BORRSTF | RCC_CSR_PINRSTF;   /* power-on */
    memset(&g_crash, 0, sizeof(g_crash));               /* power-on RAM */
}

/* Run one boot in its own process; `script` sets knobs, sim_start()s and checks */
static void boot(void (*script)(void))
{
    fflush(stdout);
    uint8_t after_reset = sh->reset_seen;
    sh->reset_seen = 0;
    pid_t pid = fork();
    if (pid == 0) {
        alarm(60);                         /* a hung boot fails, not blocks */
        tests_run = tests_failed = 0;
        board_defaults();
        if (after_reset) {
            Test_RCC.CSR = RCC_CSR_SFTRSTF | RCC_CSR_PINRSTF;
            if (sh->crash_valid) g_crash = sh->crash;
        }
        script();
        sh->run += tests_run;
        sh->failed += tests_failed;
        fflush(stdout);
        exit(0);
    }
    int st = 0;
    waitpid(pid, &st, 0);
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) {
        tests_run++;
        tests_failed++;
        printf("  FAIL boot process %s %d\n",
               WIFSIGNALED(st) ? "killed by signal" : "exit status",
               WIFSIGNALED(st) ? WTERMSIG(st) : WEXITSTATUS(st));
    }
    if (!sh->reset_seen) sh->crash_valid = 0;   /* powered off: the RAM is gone */
}

static void card_blank(void)
{
    memset(sh->card, 0, sizeof(sh->card));
    sh->crash_valid = 0;
    sh->reset_seen = 0;
}

/* ================================================================== */
/* Scenarios                                                           */
/* ================================================================== */

/* ---- A healthy boot: watchdog, clocks, screens, breadcrumbs --------- */

static void script_healthy_boot(void)
{
    sim_start();

    /* IWDG: /256 prescaler, full reload, started and fed through boot */
    CHECK(iwdg_running && Test_IWDG.PR == 6 && Test_IWDG.RLR == 0x0FFF);
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
    /* Clock tree the SPI/UART timing comments assume: 25 MHz HSE / 25 * 84
     * / 2 = 42 MHz SYSCLK, APB2 = /4, SD init clock 10.5 MHz / 256 = 41 kHz,
     * inside the SD spec's 400 kHz identification-mode ceiling */
    uint32_t sysclk_khz = 25000u / osc_cfg.PLL.PLLM * osc_cfg.PLL.PLLN / osc_cfg.PLL.PLLP;
    CHECK(sysclk_khz == 42000u);
    CHECK(clk_cfg.APB2CLKDivider == RCC_HCLK_DIV4 && clk_latency == FLASH_LATENCY_1);
    CHECK(spi_seen.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256);
    CHECK(sysclk_khz / 4u / 256u <= 400u);

    /* Boot screens, in the order the operator sees them */
    CHECK(test_patterns == 1);
    CHECK(shown("RF Init|") && shown("OK| LoRa 433MHz|"));
    CHECK(shown("SD Card|") && shown("Ready"));
    CHECK(shown("SD Self-Test|") && shown("PASS"));
    CHECK(shown("BNO085 Initialized"));
    CHECK(strstr(frames, "RF Init") < strstr(frames, "SD Self-Test") &&
          strstr(frames, "SD Self-Test") < strstr(frames, "BNO085"));
    CHECK(!shown("Error") && !shown("FAIL"));

    /* Radio up once, hunting; the scan re-armed at loop entry, not the
     * ~5 s-into-boot start whose dwell expired behind the init screens */
    CHECK(rf.init_calls == 1 && rf.scanning && rf.sweeps == 1);
    CHECK(loop_entry_tick - rf.scan_start_tick < 500u);
    CHECK(iwdg_feeds >= 5);

    run_ms(2000);
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
    CHECK(rf.services > 0);                /* the command uplink is serviced */
    const char *log = session_log();
    CHECK(strstr(log, "Timestamp,Type,") == log);            /* the CSV header */
    CHECK(strstr(log, ",EVENT,RF scan start CH0 433.00MHz\n") != NULL);
    CHECK(strstr(log, ",EVENT,BOOT fw=test SF") != NULL);
    CHECK(count_in(log, ",EVENT,NF sweep CH") == LORA_CHANNEL_COUNT);
    char nf_row[80];
    snprintf(nf_row, sizeof(nf_row), ",EVENT,NF sweep CH7 434.75MHz nf=-113dBm at=%lu.%03lu\n",
             (unsigned long)(rf.sweep_tick[7] / 1000), (unsigned long)(rf.sweep_tick[7] % 1000));
    CHECK(strstr(log, nf_row) != NULL && rf.sweep_tick[7] > rf.sweep_tick[0]);
    CHECK(strstr(log, ",EVENT,RESET src=POR vdd=3255mV\n") != NULL);
    CHECK(strstr(log, ",ERROR,") == NULL);
    CHECK(strstr(log, ",EVENT,B2 released\n") && strstr(log, ",EVENT,B2 idle at boot\n"));
    CHECK(strstr(log, ",EVENT,PAGE 0\n") != NULL);
    CHECK(strstr(log, ",EVENT,COMPASS mag_cal=0 heading_INvalid\n") != NULL);
    CHECK(Test_RCC.CSR & RCC_CSR_RMVF);    /* reset flags cleared for next boot */
    CHECK(adc.sampling == ADC_SAMPLETIME_480CYCLES && adc.channel == ADC_CHANNEL_0);
    CHECK(adc.deinits >= 1);               /* the ADC is released after reading */
    CHECK(!led_on && led_pulses == 0);     /* nothing heard, LED dark */
    CHECK(page_drawn == DISPLAY_MODE_NAVIGATION && nav_page.pkts == 0);

    /* The scan locks: logged, remembered for the next boot, shown now */
    uint32_t updates = disp_updates;
    rf.lock_pending = 1;
    run_ms(10);
    CHECK(!rf.scanning && disp_updates == updates + 1);
    CHECK(strstr(session_log(), ",EVENT,RF scan locked CH0 433.00MHz\n") != NULL);
    CHECK(card_has_file("LASTCH.BIN"));
}

TEST(test_healthy_boot)
{
    card_blank();
    boot(script_healthy_boot);
    CHECK(card_has_file("L0001.TXT"));
}

/* ---- Reset causes: priority order, and a hang is an ERROR row -------- */

static const struct { uint32_t csr; const char *row; } reset_cases[] = {
    { RCC_CSR_PORRSTF | RCC_CSR_BORRSTF | RCC_CSR_PINRSTF, ",EVENT,RESET src=POR vdd=" },
    { RCC_CSR_BORRSTF | RCC_CSR_PINRSTF,                   ",EVENT,RESET src=BOR vdd=" },
    { RCC_CSR_PINRSTF,                                     ",EVENT,RESET src=PIN vdd=" },
    { RCC_CSR_SFTRSTF | RCC_CSR_PINRSTF,                   ",EVENT,RESET src=SOFT vdd=" },
    { RCC_CSR_IWDGRSTF | RCC_CSR_PINRSTF,  ",ERROR,RESET src=IWDG vdd=3255mV (firmware hang)" },
    { RCC_CSR_WWDGRSTF | RCC_CSR_PINRSTF,  ",ERROR,RESET src=WWDG vdd=3255mV (firmware hang)" },
    { RCC_CSR_LPWRRSTF,                                    ",EVENT,RESET src=UNK vdd=" },
};
static unsigned reset_case = 0;

static void script_reset_cause(void)
{
    Test_RCC.CSR = reset_cases[reset_case].csr;
    sim_start();
    const char *log = session_log();
    CHECK(strstr(log, reset_cases[reset_case].row) != NULL);
    CHECK(count_in(log, "RESET src=") == 1);
    CHECK(Test_RCC.CSR & RCC_CSR_RMVF);
}

TEST(test_reset_cause_rows)
{
    for (reset_case = 0; reset_case < sizeof(reset_cases) / sizeof(reset_cases[0]); reset_case++) {
        card_blank();
        boot(script_reset_cause);
    }
}

/* ---- Battery reading: 8-sample average over the 100k/330k divider ---- */

static void script_vdd(void)
{
    /* Boot: two of eight conversions time out; the rest average to 3150
     * counts -> 3150 * 3300 * 43 / (4095 * 33) = 3307 mV */
    for (int i = 0; i < 8; i++) adc.raw[i] = (i & 1) ? 3200 : 3100;
    adc.timeout_mask = 0x81;               /* conversions 0 and 7: raws 3100, 3200 */
    sim_start();
    CHECK(strstr(session_log(), "RESET src=POR vdd=3307mV") != NULL);

    /* Every conversion times out: the reading stays what it was */
    adc.timeout_mask = 0xFF;
    run_ms(60000);
    CHECK(strstr(session_log(), " vdd=3307mV loop=") != NULL);   /* RFSTATS */

    /* The ADC will not configure: no conversions at all */
    adc.timeout_mask = 0;
    adc.config = HAL_ERROR;
    uint8_t started = adc.started;
    run_ms(60000);
    CHECK(adc.started == started);
    CHECK(count_in(session_log(), " vdd=3307mV loop=") == 2);
}

static void script_vdd_dead_adc(void)
{
    adc.init = HAL_ERROR;
    sim_start();
    CHECK(strstr(session_log(), "RESET src=POR vdd=0mV\n") != NULL);
    CHECK(adc.started == 0);
}

TEST(test_battery_measurement)
{
    card_blank();
    boot(script_vdd);
    card_blank();
    boot(script_vdd_dead_adc);
}

/* ---- Degraded boots: dead GPS, dead radio, its runtime recovery ------ */

static void script_no_gps_no_radio(void)
{
    gps.init_ok = 0;
    rf.init_fails = 99;
    rf.init_code = RF_ERR_SPI_INIT;
    sim_start();
    CHECK(shown("GPS Init Error|") && shown("RX still works"));
    CHECK(shown("RF Init Err: 0xE1| retrying in bg...|"));
    CHECK(rf.init_calls == 3);             /* three tries, 250 ms apart */
    CHECK(rf.init_tick[1] - rf.init_tick[0] == 250 && rf.init_tick[2] - rf.init_tick[1] == 250);
    CHECK(rf.scan_starts == 0 && rf.sweeps == 0);
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());

    /* The loop retries the radio every 10 s and never polls the dead GPS */
    uint32_t calls = rf.init_calls;
    run_ms(25000);
    CHECK(rf.init_calls == calls + 2 || rf.init_calls == calls + 3);
    CHECK(gps.update_calls == 0);
    rf.bound = 3;                          /* radio events are ignored while dead */
    rf.wedges = 2;
    run_ms(1000);
    CHECK(rf.pings == 0);

    /* It comes back: scan, breadcrumb, then radio rows flow */
    rf.init_fails = 0;
    run_ms(11000);
    CHECK(rf.scanning && rf.scan_starts == 1);
    const char *log = session_log();
    CHECK(strstr(log, ",ERROR,RF radio recovered by runtime re-init\n") != NULL);
    CHECK(strstr(log, ",ERROR,RF wedge recovered (#2)\n") != NULL);
    CHECK(strstr(log, ",EVENT,RF bound rocket_id=3 CH0 433.00MHz\n") != NULL && rf.pings == 1);
    CHECK(strstr(log, "RF scan start") == NULL);   /* the boot-only rows never ran */
}

static void script_radio_recovers_after_launch(void)
{
    rf.init_fails = 3;
    sim_start();
    rf.latched = 1;                        /* launch already seen this session */
    run_ms(11000);
    CHECK(rf.init_calls == 4);
    CHECK(!rf.scanning && rf.scan_starts == 0 && rf.scan_from_starts == 0);
}

static void script_radio_second_try(void)
{
    rf.init_fails = 1;
    sim_start();
    CHECK(rf.init_calls == 2 && shown("RF Init|") && !shown("RF Init Err"));
}

TEST(test_degraded_boots)
{
    card_blank();
    boot(script_no_gps_no_radio);
    card_blank();
    boot(script_radio_recovers_after_launch);
    card_blank();
    boot(script_radio_second_try);
}

/* ---- The card: absent, refusing writes at boot, failing mid-session --- */

static void script_no_card(void)
{
    sd.absent = 1;
    sim_start();
    CHECK(shown("SD Card Error|Stat:1 CMD0:0x00|FS:91 DET:0|"));
    CHECK(Test_RCC.CSR & RCC_CSR_RMVF);    /* cleared even with nothing logged */
    rf.bound = 3;
    rf_position(39.899f, -104.885f, -80);
    run_ms(5000);
    CHECK(sd.writes == 0);
    CHECK(rf.pings == 0);                  /* the binding row (and its PING) needs a card */
}

static void script_card_refuses_writes(void)
{
    sd.read_only = 1;                      /* formatted card gone read-only */
    sim_start();
    CHECK(shown("SD Self-Test FAIL|S:"));
    CHECK(shown("CMD:00 DT:0D T:12|"));
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
    rf_position(39.899f, -104.885f, -80);
    run_ms(5000);
    CHECK(loops > 100);                    /* still running, nothing logged */
}

static void script_selftest_fails_once(void)
{
    sd.read_only = 1;
    sim_start();
    CHECK(shown("SD Self-Test FAIL"));
    sd.read_only = 0;                      /* the card works again */
    uint32_t writes = sd.writes;
    rf_position(39.899f, -104.885f, -80);
    run_ms(15000);
    CHECK(sd.writes == writes);            /* no NAV row, no BEACON.TXT */
}

static void script_card_fails_mid_session(void)
{
    gps_fix_at(39.89f, -104.885f, 8);
    sim_start();
    /* A session's worth of NAV rows: the log outgrows inline storage */
    for (int i = 0; i < 60; i++) {
        rf_position(39.899f, -104.885f, -80);
        run_ms(1000);
    }
    CHECK(count_in(session_log(), ",NAV,") == 60);

    /* The card stops taking writes: the loop keeps running, the errors
     * reach sd_card.c's ladder and the bus steps down */
    sd.read_only = 1;
    uint32_t loops0 = loops;
    for (int i = 0; i < 60; i++) {
        rf_position(39.899f, -104.885f, -80);
        run_ms(1000);
    }
    SD_Card_Info info;
    SD_Card_GetInfo(&info);
    CHECK(info.write_errors > 0 && sd.stepdowns > 0);
    CHECK(loops - loops0 > 3000);
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
    CHECK(sd.write_errors < 400);          /* bounded retries, no relocation storm */

    /* It recovers: logging resumes in the same session file */
    sd.read_only = 0;
    for (int i = 0; i < 40; i++) {
        rf_position(39.899f, -104.885f, -80);
        run_ms(1000);
    }
    CHECK(count_in(session_log(), ",NAV,") > 60);
}

TEST(test_card_failures)
{
    card_blank();
    boot(script_no_card);
    CHECK(!card_has_file("L0001.TXT"));
    card_blank();
    card_put("KEEP.TXT", "x", 1);
    boot(script_card_refuses_writes);
    CHECK(!card_has_file("L0001.TXT") && card_has_file("KEEP.TXT"));
    card_blank();
    card_put("KEEP.TXT", "x", 1);
    boot(script_selftest_fails_once);
    CHECK(!card_has_file("L0001.TXT") && !card_has_file("BEACON.TXT"));
    card_blank();
    boot(script_card_fails_mid_session);
}

/* ---- Compass bring-up failures: the I2C scan screens ----------------- */

static void script_compass_none(void)
{
    cmp.init_ok = 0;
    sim_start();
    CHECK(cmp.scan_shows == 1 && shown("No I2C devices found!"));
    CHECK(led_pulses == 3);                /* three blinks, then carry on */
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
}

static void script_compass_wrong_parts(void)
{
    static const uint8_t found[6] = { 0x10, 0x20, 0x30, 0x3C, 0x50, 0x68 };
    cmp.init_ok = 0;
    cmp.dev_count = 6;
    memcpy(cmp.addrs, found, sizeof(found));
    sim_start();
    CHECK(shown("Found 6 I2C device(s)|"));
    CHECK(shown("0x10 0x20 0x30 0x3C|"));  /* four to a row */
    CHECK(shown("0x50 0x68|"));
    CHECK(cmp.quat_sets == 0 && cmp.decl_calls == 0);   /* no compass, no restore */
    CHECK(iwdg_max_gap < iwdg_worst_timeout_ms());
}

TEST(test_compass_init_failures)
{
    card_blank();
    boot(script_compass_none);
    card_blank();
    boot(script_compass_wrong_parts);
}

/* ---- Crash records: captured, carried over a reset, logged once ------ */

static uint8_t fault_now = 0, errh_now = 0;
static void pass_maybe_crash(void)
{
    static uint32_t frame[8] = { 0x11, 0x22, 0x33, 0x44, 0x1212, 0x08001001u, 0x08004567u, 0x21000000u };
    if (fault_now) {
        fault_now = 0;
        Test_SCB.CFSR = 0x00008200u;       /* precise data bus error, BFAR valid */
        Test_SCB.HFSR = 0x40000000u;
        Test_SCB.BFAR = 0x60000000u;
        HardFault_Capture(frame);          /* what the naked handler branches to */
    }
    if (errh_now) {
        errh_now = 0;
        Error_Handler();
    }
}

static void script_fault_in_loop(void)
{
    on_pass = pass_maybe_crash;
    sim_start();
    run_ms(3000);
    fault_now = 1;
    run_ms(100);
    CHECK(sh->reset_seen && g_crash.magic == CRASH_MAGIC && g_crash.seq == 1);
}

static void script_after_fault_then_errh(void)
{
    on_pass = pass_maybe_crash;
    sim_start();
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,RESET src=SOFT vdd=") != NULL);
    CHECK(strstr(log, ",EVENT,FAULT#1 pc=08004567 lr=08001001 cfsr=00008200 bfar=60000000\n") != NULL);
    CHECK(strstr(log, ",EVENT,FAULT r0=00000011 r1=00000022 r2=00000033 r3=00000044 "
                      "r12=00001212 msp=2000FF80 psp=2000F000 psr=21000000\n") != NULL);
    CHECK(g_crash.magic == 0);             /* consumed */
    errh_now = 1;
    run_ms(100);
    CHECK(sh->reset_seen);
}

static void script_boot_crash(void)
{
    osc_status = HAL_ERROR;                /* the oscillator never comes up */
    sim_start();
    CHECK(sh->reset_seen && g_crash.seq == 2 && g_crash.source == CRASH_SRC_ERROR_HANDLER);
}

static void script_boot_crash_clocks(void)
{
    clk_status = HAL_ERROR;                /* the bus clocks will not switch */
    sim_start();
    CHECK(sh->reset_seen && g_crash.seq == 3);
}

static void script_boot_crash_spi(void)
{
    spi_status = HAL_ERROR;                /* SPI1 (card + displays) will not init */
    sim_start();
    CHECK(sh->reset_seen && g_crash.seq == 4 && test_patterns == 0);
}

static void pass_reset(void) { NVIC_SystemReset(); }

static void script_after_errh(void)
{
    sim_start();
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,ERRH#4 pc=") != NULL);   /* four in a row, nothing logged between */
    CHECK(strstr(log, ",EVENT,FAULT r0=00000000") != NULL);
    on_pass = pass_reset;                  /* a reset with no crash: RAM kept */
    run_ms(100);
}

static void script_consumed_record_stays_quiet(void)
{
    sim_start();
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,RESET src=SOFT vdd=") != NULL);
    CHECK(strstr(log, "FAULT") == NULL && strstr(log, "ERRH") == NULL);
}

TEST(test_crash_records)
{
    card_blank();
    boot(script_fault_in_loop);
    boot(script_after_fault_then_errh);
    boot(script_boot_crash);
    boot(script_boot_crash_clocks);
    boot(script_boot_crash_spi);
    boot(script_after_errh);
    boot(script_consumed_record_stays_quiet);
}

/* ---- Buttons and pages ----------------------------------------------- */

static void script_buttons(void)
{
    sim_start();
    run_ms(1000);
    for (int i = 1; i <= DISPLAY_MODE_COUNT; i++) {
        uint32_t updates = disp_updates;
        btn.b1 = 1;
        run_ms(10);                        /* one pass */
        CHECK(page_drawn == i % DISPLAY_MODE_COUNT);
        CHECK(disp_updates == updates + 1); /* drawn now, not at the next 250 ms frame */
    }
    btn.b1_long = 1;                       /* a long press cycles too */
    run_ms(10);
    CHECK(page_drawn == DISPLAY_MODE_GPS);
    btn.b2 = 1;                            /* B2 off the Rocket Select page: home */
    run_ms(10);
    CHECK(page_drawn == DISPLAY_MODE_NAVIGATION && rf.channel == 0);

    /* B2 on Rocket Select steps the channel, logs it and remembers it */
    for (int i = 0; i < DISPLAY_MODE_CHANNEL; i++) { btn.b1 = 1; run_ms(10); }
    CHECK(page_drawn == DISPLAY_MODE_CHANNEL);
    btn.b2 = 1;
    run_ms(10);
    CHECK(rf.channel == 1 && page_drawn == DISPLAY_MODE_CHANNEL);
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,RF manual CH1 433.25MHz\n") != NULL);
    for (int i = 1; i <= DISPLAY_MODE_COUNT; i++) {
        char row[24];
        snprintf(row, sizeof(row), ",EVENT,PAGE %d\n", i % DISPLAY_MODE_COUNT);
        CHECK(strstr(log, row) != NULL);
    }
    CHECK(card_has_file("LASTCH.BIN"));

    /* The frame cadence without presses: 4 Hz */
    uint32_t updates = disp_updates;
    run_ms(10000);
    CHECK(disp_updates - updates >= 39 && disp_updates - updates <= 41);

    /* 120 s on a page with no press: back to Navigation (the channel
     * step was the last press, ~10.5 s ago) */
    run_ms(105000);
    CHECK(page_drawn == DISPLAY_MODE_CHANNEL);
    run_ms(6000);
    CHECK(page_drawn == DISPLAY_MODE_NAVIGATION);

    /* B2 electrical transitions */
    btn.b2_held = 1;
    run_ms(100);
    btn.b2_held = 0;
    run_ms(100);
    log = session_log();
    CHECK(strstr(log, ",EVENT,B2 press (electrical)\n") != NULL);
    CHECK(count_in(log, ",EVENT,B2 released\n") == 2);

    /* A page index gone bad recovers to the GPS page */
    btn.b1 = 1;
    run_ms(10);
    current_display_mode = (DisplayMode_t)42;
    run_ms(20);
    CHECK(page_drawn == DISPLAY_MODE_GPS && current_display_mode == DISPLAY_MODE_GPS);
}

/* The next boot resumes the scan on the channel the operator picked */
static void script_scan_resumes(void)
{
    sim_start();
    CHECK(rf.scan_from_starts == 1 && rf.channel == 1);
}

static void script_b2_stuck(void)
{
    btn.b2_held = 1;
    sim_start();
    run_ms(1000);
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,B2 press (electrical)\n") != NULL);
    CHECK(strstr(log, ",EVENT,B2 held at boot (stuck key or wiring?)\n") != NULL);
}

TEST(test_buttons_and_pages)
{
    card_blank();
    boot(script_buttons);
    boot(script_scan_resumes);
    card_blank();
    boot(script_b2_stuck);
}

/* ---- Radio breadcrumbs: every one-shot becomes its row ---------------- */

static uint32_t stall_ms = 0;
static void pass_stall_once(void) { sim_advance(stall_ms); stall_ms = 0; }

/* A numeric field of the k-th RFSTATS row from the end */
static long rfstats_field(const char *log, const char *key, int k)
{
    const char *rows[64];
    int n = 0;
    for (const char *p = log; n < 64 && (p = strstr(p, ",EVENT,RFSTATS ")) != NULL; p++) rows[n++] = p;
    if (k >= n) return -1;
    const char *f = strstr(rows[n - 1 - k], key);
    const char *eol = strchr(rows[n - 1 - k], '\n');
    return (f && (!eol || f < eol)) ? atol(f + strlen(key)) : -1;
}

static void script_radio_rows(void)
{
    sim_start();

    rf.bound = 3;                          /* binding: row + PING, once */
    run_ms(100);
    run_ms(100);
    CHECK(rf.pings == 1);
    for (uint32_t n = 1; n <= 5; n++) {    /* foreign: first, then every 4 */
        rf.foreign = n;
        run_ms(20);
    }
    rf.latched = 1;
    rf.suppressions = 1;
    run_ms(20);
    rf.suppressions = 2;
    run_ms(20);
    rf.wedges = 1;
    run_ms(20);

    rf.hb = (HeartbeatPacket_t){ PACKET_TYPE_HEARTBEAT, 3, 0, 7, 1, 120,
                                 HB_GPS_HEALTH(HB_GPS_ACQUIRING, 1), HB_RESET_POR, 0x05 };
    rf.hbi = (RF_HbInfo_t){ 0, RF_HB_RESET_NONE, 0, 0 };
    rf.hb_new = 1;
    run_ms(20);
    rf.hb.uptime_s = 5;                    /* the beacon rebooted */
    rf.hb.reset_info = HB_RESET_WDT;
    rf.hbi = (RF_HbInfo_t){ 0, RF_HB_UPTIME_REGRESSED, 125, HB_RESET_POR };
    rf.hb_new = 1;
    run_ms(20);
    rf.hb.uptime_s = 30;
    rf.hbi = (RF_HbInfo_t){ 0, RF_HB_CAUSE_CHANGED, 25, HB_RESET_POR };
    rf.hb_new = 1;
    run_ms(20);
    rf.hb.uptime_s = 40;
    rf.hb.reset_info = HB_RESET_EXT;
    rf.hbi = (RF_HbInfo_t){ 0, RF_HB_CAUSE_CHANGED, 30, HB_RESET_WDT };
    rf.hb_new = 1;
    run_ms(20);
    rf.hb.rocket_id = 9;                   /* someone else's: logged as such only */
    rf.hb.alt_inputs = 0x3F;
    rf.hbi = (RF_HbInfo_t){ 1, RF_HB_RESET_NONE, 0, 0 };
    rf.hb_new = 1;
    run_ms(20);

    rf.t0 = (LaunchT0Packet_t){ PACKET_TYPE_LAUNCH_T0, 3, 300, 15 };
    rf.t0_new = 1;
    rf.imu = (ImuTracePacket_t){ PACKET_TYPE_IMU, 3, (uint16_t)(IMU_TS_V3_FLAG | 250), 100, -50,
                                 1200, 10, 20, 30, 1300, 0 };
    rf.imu_fmt = 3;
    rf.imu_new = 1;
    rf.evq[0] = (FlightEventPacket_t){ PACKET_TYPE_FLIGHT_EVENT, 3, FLIGHT_EVENT_APOGEE, 310, 2500 };
    rf.evq[1] = (FlightEventPacket_t){ PACKET_TYPE_FLIGHT_EVENT, 3, FLIGHT_EVENT_DROGUE, 311, 0 };
    rf.evq[2] = (FlightEventPacket_t){ PACKET_TYPE_FLIGHT_EVENT, 3, FLIGHT_EVENT_ANOM_TUMBLE, 312, -7 };
    rf.evq_n = 3;
    rf.mx = (MaximaPacket_t){ PACKET_TYPE_MAXIMA, 3, 2500, 310, 21050, 1234, 20 };
    rf.mx_new = 1;
    memset(&rf.alt, 0, sizeof(rf.alt));
    rf.alt.mx = rf.mx;
    rf.alt.alt_inputs = 0x01;                 /* J2: an RRC3, drogue fired */
    rf.alt.alt[0].status = ALT_ST_FMT_RRC3 | ALT_ST_DROGUE;
    rf.alt.alt[0].apogee_4ft = 2050;
    rf.alt_new = 1;
    rf.hello = (HelloPacket_t){ PACKET_TYPE_HELLO, 3, 0x8abc1234u, 0 };
    rf.hello_new = 1;
    rf.wr_count = 2; rf.wr_last = 3200; rf.wr_total = 4100;
    rf.wreport_new = 1;
    static const uint8_t junk[8] = { 0x01, 0x02, 0x03, 0x04, 0xa5, 0x06, 0x07, 0x08 };
    memcpy(rf.crc_dump, junk, 8);
    rf.crc_len = 8; rf.crc_total = 20;
    run_ms(20);
    CHECK(rf.evq_n == 0);                  /* all three events drained in one pass */
    run_ms(5000);
    rf.crc_len = 4;                        /* 5 s on: dropped */
    run_ms(6000);
    rf.crc_len = 4;                        /* 11 s on: logged */
    run_ms(20);

    static const RF_CmdResult_t cmds[] = {
        { RF_CMD_EV_SENT, 0x01, 7, 1, 0, 0, 0 },
        { RF_CMD_EV_ACK, 0x01, 7, 2, 0x5A, -88, 6 },
        { RF_CMD_EV_NO_ACK, 0x01, 8, 3, 0, 0, 0 },
        { RF_CMD_EV_CANCELLED, 0x01, 9, 1, 0, 0, 0 },
        { RF_CMD_EV_SUPPRESSED, 0x01, 10, 0, 0, 0, 0 },
    };
    for (unsigned i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        rf.cmd = cmds[i];
        rf.cmd_new = 1;
        run_ms(20);
    }

    rf.noise = 1; rf.nf_valid = 1; rf.nf = -95;
    run_ms(20);
    rf.noise = 0; rf.nf_valid = 0;         /* cleared by a channel change: no floor */
    run_ms(20);

    const char *log = session_log();
    char want[160];
    CHECK(count_in(log, ",EVENT,RF bound rocket_id=3 CH0 433.00MHz\n") == 1);
    CHECK(strstr(log, ",EVENT,RF foreign pkts=1 dropped CH0 rssi=-90dBm snr=5\n") != NULL);
    CHECK(strstr(log, ",EVENT,RF foreign pkts=5 dropped CH0") != NULL);
    CHECK(count_in(log, ",EVENT,RF foreign pkts=") == 2);
    CHECK(count_in(log, ",EVENT,FLIGHT CONTACT launched - auto-rescan OFF\n") == 1);
    CHECK(strstr(log, ",EVENT,AUTO-RESCAN suppressed x1 (flight contact)\n") != NULL);
    CHECK(strstr(log, ",EVENT,AUTO-RESCAN suppressed x2 (flight contact)\n") != NULL);
    CHECK(strstr(log, ",ERROR,RF wedge recovered (#1)\n") != NULL);

    RF_Parser_FormatPadAltimeters(3, 0x05, want, sizeof(want));
    CHECK(count_in(log, want) == 1);       /* ours, once; the foreign one is not */
    CHECK(strstr(log, "ALTIM pad id=9") == NULL);
    CHECK(strstr(log, ",EVENT,HEARTBEAT id=3 CH0 sats=7 fix=1 up=120s rssi=-90 gps=acq rst=1 rc=POR\n") != NULL);
    snprintf(want, sizeof(want), ",ERROR,TX reset: uptime 125s -> 5s (cause=%s)\n", HB_RESET_NAME(HB_RESET_WDT));
    CHECK(strstr(log, want) != NULL);
    snprintf(want, sizeof(want), ",ERROR,TX boot cause changed: %s(0x01) -> %s(0x20) at up=30s\n",
             HB_RESET_NAME(HB_RESET_POR), HB_RESET_NAME(HB_RESET_WDT));
    CHECK(strstr(log, want) != NULL);
    snprintf(want, sizeof(want), ",ERROR,TX boot cause changed: %s(0x20) -> %s(0x10) at up=40s\n",
             HB_RESET_NAME(HB_RESET_WDT), HB_RESET_NAME(HB_RESET_EXT));
    CHECK(strstr(log, want) != NULL);
    CHECK(strstr(log, ",EVENT,HEARTBEAT FOREIGN id=9 CH0") != NULL);

    CHECK(strstr(log, ",EVENT,LAUNCH T0 id=3 up=300s gps_fix_age=1.5s\n") != NULL);
    CHECK(strstr(log, ",IMU,250,100,-50,1200,10,20,30,1300,0,-90,5,3\n") != NULL);
    snprintf(want, sizeof(want), ",EVENT,%s id=3 up=310s val=2500\n", FLIGHT_EVENT_NAME(FLIGHT_EVENT_APOGEE));
    CHECK(strstr(log, want) != NULL);
    snprintf(want, sizeof(want), ",EVENT,%s id=3 up=312s val=-7\n", FLIGHT_EVENT_NAME(FLIGHT_EVENT_ANOM_TUMBLE));
    CHECK(strstr(log, want) != NULL);
    CHECK(strstr(log, ",EVENT,MAXIMA id=3 alt=2500m@310s v=210.5m/s a=12.3g r=320dps\n") != NULL);
    RF_Parser_FormatAltimeter(&rf.alt, want, sizeof(want));
    CHECK(strstr(log, want) != NULL);
    CHECK(strstr(log, ",EVENT,TXBOOT id=3 fw=abc1234-dirty\n") != NULL);
    CHECK(strstr(log, ",EVENT,WEDGE #2 recovered deaf=3200ms total=4100ms\n") != NULL);
    CHECK(strstr(log, ",EVENT,CRC garbage len=20 head=01020304a5060708\n") != NULL);
    CHECK(count_in(log, ",EVENT,CRC garbage") == 2);   /* 10 s rate limit */

    CHECK(strstr(log, ",EVENT,CMD 0x01 sent seq=7 try=1\n") != NULL);
    CHECK(strstr(log, ",EVENT,CMD ACK cmd=0x01 seq=7 echo=90 rssi=-88 snr=6 try=2\n") != NULL);
    CHECK(strstr(log, ",EVENT,CMD 0x01 seq=8 no ACK after 3 tries\n") != NULL);
    CHECK(strstr(log, ",EVENT,CMD 0x01 seq=9 cancelled (binding lost)\n") != NULL);
    CHECK(strstr(log, ",EVENT,CMD 0x01 suppressed (beacon airborne)\n") != NULL);
    CHECK(strstr(log, ",EVENT,RF NOISE HIGH CH0 433.00MHz nf=-95dBm\n") != NULL);
    CHECK(strstr(log, ",EVENT,RF NOISE clear CH0 433.00MHz\n") != NULL);

    /* The once-a-minute RFSTATS row: the counters it was given, and the
     * worst main-loop pass since the row before (a 3 s stall, then none:
     * the quiet minute still carries the card's commit bursts, ~250 ms) */
    rf.pkts = 42; rf.irqs = 44; rf.crc_errors = 3; rf.nfl_valid = 1; rf.nfl = -118;
    rf.nf_valid = 1; rf.nf = -121;
    on_pass = pass_stall_once;
    stall_ms = 3000;
    run_ms(61000);
    run_ms(60000);
    log = session_log();
    CHECK(strstr(log, ",EVENT,RFSTATS pkts=42 irq=44 crc=3 wedges=1 nf=-121dBm vdd=3255mV loop=") != NULL);
    CHECK(strstr(log, " spidiv=16 rf=") != NULL && strstr(log, " nfl=-118dBm\n") != NULL);
    CHECK(rfstats_field(log, "loop=", 1) >= 3000 && rfstats_field(log, "loop=", 0) < 1000);
    CHECK(rfstats_field(log, "iters=", 0) > 4000);   /* passes per minute */
}

static void script_radio_rows_without_card(void)
{
    sd.absent = 1;
    sim_start();
    rf.cmd = (RF_CmdResult_t){ RF_CMD_EV_SENT, 0x01, 7, 1, 0, 0, 0 };
    rf.cmd_new = 1;
    rf.noise = 1;
    run_ms(100);
    CHECK(rf.cmd_new == 0);                /* still consumed */
    CHECK(sd.writes == 0);
}

TEST(test_radio_breadcrumbs)
{
    card_blank();
    boot(script_radio_rows);
    card_blank();
    boot(script_radio_rows_without_card);
}

/* ---- Beacon packets: NAV rows, BEACON.TXT, navigation, contact ------- */

#define HOME_LAT 39.8900f
#define HOME_LON (-104.8850f)

static void script_beacon_session(void)
{
    gps_fix_at(HOME_LAT, HOME_LON, 8);
    sim_start();
    run_ms(2000);
    CHECK(has_valid_local_gps);
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,Local GPS first fix: sats=8 utc=12:34:56\n") != NULL);
    CHECK(strstr(log, ",EVENT,DECLIN from location 39.8900,-104.8850\n") != NULL);

    /* Two packets, 1 km then 2 km north: each NAV row measures its own
     * packet, not the navigation state left by the one before */
    rf_position(HOME_LAT + 0.009f, HOME_LON, -80);
    snprintf(rf.callsign, sizeof(rf.callsign), "KD0ABC-3 CH0");
    run_ms(300);
    /* The RF pulse, on a packet whose pass is quick (no BEACON.TXT save;
     * see the first packet's above, which commits the file) */
    uint32_t pulses = led_pulses;
    rf_position(HOME_LAT + 0.018f, HOME_LON, -80);
    run_ms(20);
    CHECK(led_on && led_pulses == pulses + 1);
    run_ms(200);
    CHECK(!led_on);
    log = session_log();
    CHECK(fabs(row_field(log, ",NAV,", 0, 15) - 1.0) < 0.01);
    CHECK(fabs(row_field(log, ",NAV,", 1, 15) - 2.0) < 0.01);
    CHECK(fabs(row_field(log, ",NAV,", 1, 16) - 0.0) < 0.5);    /* bearing: north */
    CHECK(fabs(row_field(log, ",NAV,", 1, 17) - 90.0) < 0.01);  /* heading */
    CHECK(fabs(distance_to_tx - 2.0f) < 0.01f && fabs(direction_to_tx) < 0.5f);
    CHECK(fabs(relative_direction - 270.0f) < 0.5f);            /* facing east */
    CHECK(count_in(log, ",EVENT,CALLSIGN KD0ABC-3 CH0 rx-ch CH0\n") == 1);
    CHECK(page_drawn == DISPLAY_MODE_NAVIGATION && nav_page.beacon_ok && nav_page.pkts == 2);
    /* BEACON.TXT: the first packet was saved, the second inside 10 s was not */
    CHECK(fabs(atof(card_file("BEACON.TXT")) - 39.899) < 1e-4);
    run_ms(10000);
    rf_position(HOME_LAT + 0.018f, HOME_LON, -80);
    run_ms(20);
    CHECK(fabs(atof(card_file("BEACON.TXT")) - 39.908) < 1e-4);

    /* Walk-out and walk-back: a weak packet, then a strong one */
    rf_position(HOME_LAT + 0.018f, HOME_LON, -125);
    run_ms(20);
    rf_position(HOME_LAT + 0.018f, HOME_LON, -75);
    run_ms(20);
    log = session_log();
    CHECK(strstr(log, ",EVENT,LINK WEAK rssi=-125dBm snr=7\n") != NULL);
    CHECK(strstr(log, ",EVENT,LINK OK rssi=-75dBm snr=7\n") != NULL);

    /* The operator's own fix goes away: navigation keeps the last good one */
    gps.streaming = 0;
    run_ms(7000);
    CHECK(!has_valid_local_gps && has_last_good_local_gps);
    CHECK(fabs(distance_to_tx - 2.0f) < 0.01f);
    rf_position(HOME_LAT + 0.018f, HOME_LON, -80);
    run_ms(20);
    log = session_log();
    CHECK(strstr(log, ",EVENT,Local GPS fix lost\n") != NULL);
    int nav_rows = count_in(log, ",NAV,");
    CHECK(fabs(row_field(log, ",NAV,", nav_rows - 1, 12) - HOME_LAT) < 1e-4);   /* BaseLat */
    CHECK(fabs(row_field(log, ",NAV,", nav_rows - 1, 15) - 2.0) < 0.01);
    gps.streaming = 1;
    run_ms(2000);

    /* The beacon goes quiet: the position stops counting after 180 s,
     * the arrow follows the compass, the contact row lands after 5 min */
    cmp.data.heading = 180.0f;
    run_ms(185000);
    CHECK(!beacon.has_valid && fabs(relative_direction - 180.0f) < 0.5f);
    run_ms(120000);
    rf_position(HOME_LAT + 0.018f, HOME_LON, -80);
    run_ms(20);
    log = session_log();
    CHECK(strstr(log, ",EVENT,RF LOST (>5min silence)\n") != NULL);
    CHECK(strstr(log, ",EVENT,RF REGAINED after ") != NULL);
    CHECK(count_in(log, ",BASE,") >= 10);  /* the walk, every 30 s */
    CHECK(count_in(log, "Local GPS first fix") == 1 && count_in(log, "DECLIN from location") == 1);
    CHECK(strstr(log, "no fix at 120s") == NULL);
}

static void script_beacon_restored(void)
{
    gps_fix_at(HOME_LAT, HOME_LON, 8);
    sim_start();
    CHECK(shown("Last Beacon|39.90800|-104.88500|Loaded|"));
    CHECK(beacon.has_valid && beacon.has_last_good);
    run_ms(3000);
    CHECK(fabs(distance_to_tx - 2.0f) < 0.01f);  /* navigation works before any packet */
    CHECK(strstr(session_log(), "discarded") == NULL);
}

static void script_beacon_cross_state(void)
{
    gps_fix_at(39.5f, -119.8f, 8);         /* the receiver is in Nevada now */
    sim_start();
    run_ms(3000);
    CHECK(!beacon.has_valid && !beacon.has_last_good);
    CHECK(strstr(session_log(), ",EVENT,Last-beacon self-loc far from beacon - discarded\n") != NULL);
}

static void script_restored_unusable(void)
{
    gps_fix_at(51.4770f, 0.0010f, 8);      /* Greenwich */
    sim_start();
    CHECK(beacon.has_valid);               /* 0.0 longitude loads as in range... */
    run_ms(3000);
    CHECK(has_valid_local_gps && beacon.has_valid);
    CHECK(distance_to_tx == -1.0f && direction_to_tx == -1.0f && relative_direction == -1.0f);
}

TEST(test_beacon_session)
{
    card_blank();
    boot(script_beacon_session);
    boot(script_beacon_restored);
    boot(script_beacon_cross_state);
    card_blank();
    card_put("BEACON.TXT", "51.4770000,0.0000000,30.0\n", 26);   /* ...but is no point */
    boot(script_restored_unusable);
}

/* ---- Local GPS that never fixes: the 120 s diagnosis row ------------- */

static void script_gps_slow(void)
{
    gps.streaming = 1;                     /* sentences, but no fix */
    gps.fix.satellites = 3;
    gps.bytes = 9000; gps.sentences = 240; gps.drops = 2; gps.uart_errs = 1; gps.csum = 4;
    sim_start();
    run_ms(110000 - fake_tick_ms);
    CHECK(strstr(session_log(), "no fix at 120s") == NULL);
    run_ms(15000);
    const char *log = session_log();
    CHECK(strstr(log, ",EVENT,Local GPS no fix at 120s: rx=9000 snt=240 drp=2 csum=4 er=1 ra=0 sats=3\n") != NULL);
    run_ms(60000);
    CHECK(count_in(session_log(), "no fix at 120s") == 1);
    CHECK(!has_valid_local_gps && strstr(session_log(), "Local GPS first fix") == NULL);

    /* A fix from RMC alone carries no satellite count: the first-fix row
     * waits for a GGA instead of logging sats=0 */
    gps_fix_at(HOME_LAT, HOME_LON, 0);
    run_ms(3000);
    CHECK(has_valid_local_gps && strstr(session_log(), "Local GPS first fix") == NULL);
    gps.fix.satellites = 6;
    run_ms(2000);
    CHECK(strstr(session_log(), ",EVENT,Local GPS first fix: sats=6 utc=12:34:56\n") != NULL);

    /* A module claiming a fix at the 0,0 placeholder (cold-start garbage) */
    uint32_t loc_calls = cmp.loc_calls;
    gps_fix_at(0.0f, 0.0f, 4);
    run_ms(5000);
    CHECK(!has_valid_local_gps);
    CHECK(fabs(last_good_local_gps.latitude - HOME_LAT) < 1e-4f);   /* the real fix stays */
    CHECK(cmp.loc_calls == loc_calls);
}

TEST(test_local_gps_without_fix)
{
    card_blank();
    boot(script_gps_slow);
}

/* ---- Compass state: lock persisted once, declination precedence ------ */

static void script_compass_session(void)
{
    gps_fix_at(HOME_LAT, HOME_LON, 8);
    sim_start();
    cmp.quat = 3;                          /* the convention lock forms */
    cmp.data.mag_cal = 3;
    cmp.data.heading_valid = 1;
    run_ms(500);
    cmp.err_code = COMPASS_ERROR_READ_DATA;
    cmp.err_msg = "BNO085 read fail";
    run_ms(500);
    cmp.err_code = 0;
    cmp.data.heading_stale = 1;
    run_ms(500);
    const char *log = session_log();
    CHECK(count_in(log, ",EVENT,QUATLOCK conv=3 saved\n") == 1);
    CHECK(strstr(log, ",EVENT,COMPASS mag_cal=3 heading_valid\n") != NULL);
    CHECK(strstr(log, ",EVENT,COMPASS mag_cal=3 heading_valid stale\n") != NULL);
    CHECK(strstr(log, ",EVENT,COMPASS err 0x05 BNO085 read fail\n") != NULL);
    CHECK(strstr(log, ",EVENT,COMPASS err 0x00 cleared\n") != NULL);
    CHECK(card_has_file("QLOCK.BIN") && cmp.loc_applied);
}

static void script_compass_restored(void)
{
    gps_fix_at(HOME_LAT, HOME_LON, 8);
    sim_start();
    CHECK(cmp.quat == 3);                  /* restored at boot */
    run_ms(3000);
    const char *log = session_log();
    CHECK(strstr(log, "QUATLOCK") == NULL);   /* already durable: no re-save */
    CHECK(strstr(log, ",EVENT,DECLIN override 11.9degE\n") != NULL);
    CHECK(fabs(cmp.decl - 11.9f) < 1e-4f && cmp.loc_calls == 0);   /* file beats table */
    CHECK(strstr(log, "DECLIN from location") == NULL);
}

TEST(test_compass_state)
{
    card_blank();
    boot(script_compass_session);
    card_put("DECLIN.TXT", "11.9\n", 5);
    boot(script_compass_restored);
}

/* ---- e-ink summary cadence; preflight heartbeat gate ----------------- */

static void script_eink_and_preflight(void)
{
    sim_start();
    CHECK(ink.inits == 1 && ink.refreshes == 0);
    if (fake_tick_ms < 9990u) {
        run_ms(9990u - fake_tick_ms);
        CHECK(ink.refreshes == 0);
    }
    run_ms(30);
    CHECK(ink.refreshes == 1 && ink.last_refresh_tick >= 10000u);
    CHECK(strstr(ink.shown, "NO BEACON YET") != NULL);
    ink.busy = 1;                          /* the next one waits out the panel */
    run_ms(65000);
    CHECK(ink.refreshes == 1);
    ink.busy = 0;
    run_ms(20);
    CHECK(ink.refreshes == 2);
    run_ms(59000);
    CHECK(ink.refreshes == 2);
    run_ms(2000);
    CHECK(ink.refreshes == 3);
    ink.unwired = 1;
    run_ms(100);
    CHECK(count_in(session_log(), ",ERROR,EINK BUSY not wired - fixed-time refresh\n") == 1);

    /* PRE-FLIGHT: heartbeat GPS health only while that heartbeat is fresh */
    for (int i = 0; i < DISPLAY_MODE_PREFLIGHT; i++) { btn.b1 = 1; run_ms(10); }
    rf.hb = (HeartbeatPacket_t){ PACKET_TYPE_HEARTBEAT, 3, 0, 7, 1, 120,
                                 HB_GPS_HEALTH(HB_GPS_NO_NMEA, 0), HB_RESET_POR, 0x01 };
    rf.last_hb_valid = 1;
    rf.last_hb_tick = fake_tick_ms;
    rf.heard_tick = fake_tick_ms;
    run_ms(20);
    CHECK(page_drawn == DISPLAY_MODE_PREFLIGHT);
    CHECK(pre_page.hb_state == HB_GPS_NO_NMEA && pre_page.alt_hb == 0x01);
    CHECK(pre_page.sd_ok && pre_page.vdd == 3255 && pre_page.link_ok);
    run_ms(PF_LINK_FRESH_S * 1000u + 1000u);
    CHECK(pre_page.hb_state == 0xFF && pre_page.alt_hb == 0xFF && !pre_page.link_ok);
}

static void script_eink_first_page(void)
{
    sd.absent = 1;                         /* no card screens: a short boot */
    sim_start();
    CHECK(fake_tick_ms < 9000u);
    run_ms(9900u - fake_tick_ms);
    CHECK(ink.refreshes == 0);
    run_ms(200);
    CHECK(ink.refreshes == 1 && ink.last_refresh_tick >= 10000u);
}

TEST(test_eink_and_preflight)
{
    card_blank();
    boot(script_eink_and_preflight);
    card_blank();
    boot(script_eink_first_page);
}

/* ---- The testing-build advisory: one ERROR row ------------------------ */

static void script_testing_build(void)
{
    sim_start();
    rf.testing = 1;
    rf_position(HOME_LAT + 0.009f, HOME_LON, -80);
    run_ms(20);
    rf_position(HOME_LAT + 0.009f, HOME_LON, -80);
    run_ms(20);
    CHECK(count_in(session_log(), ",ERROR,TX testing-build cadence (callsign ~30s) - flashing bench firmware?\n") == 1);
    uint32_t t = fake_tick_ms;
    SysTick_Handler();                     /* the 1 ms tick ISR */
    CHECK(fake_tick_ms == t + 1u);
}

TEST(test_testing_build_advisory)
{
    card_blank();
    boot(script_testing_build);
}

int main(void)
{
    sh = mmap(NULL, sizeof(*sh), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (sh == MAP_FAILED) { perror("mmap"); return 2; }

    run_test_healthy_boot();
    run_test_reset_cause_rows();
    run_test_battery_measurement();
    run_test_degraded_boots();
    run_test_card_failures();
    run_test_compass_init_failures();
    run_test_crash_records();
    run_test_buttons_and_pages();
    run_test_radio_breadcrumbs();
    run_test_beacon_session();
    run_test_local_gps_without_fix();
    run_test_compass_state();
    run_test_eink_and_preflight();
    run_test_testing_build_advisory();

    tests_run += sh->run;
    tests_failed += sh->failed;
    return TEST_SUMMARY();
}
