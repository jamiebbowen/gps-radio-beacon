/**
 * @file test_display_modes.c
 * @brief Host tests for the small display-mode pages (the ones not already
 *        covered by test_navigation_mode.c / test_preflight_mode.c):
 *        channel select, RF stats, RF last-packet, local GPS, SD card,
 *        compass heading + rose, I2C scan, SPI test, IMU test, test pattern.
 *
 * These pages are pure formatting over stubbed sensor/RF getters, so the
 * value pinned here is the *layout contract*: which row carries which
 * datum, fallback markers ("(L)", "*", "R"), and error decode strings.
 *
 * Build & run:  make -C receiver/tests
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "display.h"
#include "display_modes/channel_mode.h"
#include "display_modes/rf_stats_mode.h"
#include "display_modes/gps_mode.h"
#include "display_modes/sd_card_mode.h"
#include "display_modes/rf_mode.h"
#include "display_modes/compass_mode.h"
#include "display_modes/test_pattern.h"
#include "gps.h"
#include "compass.h"
#include "sd_card.h"
#include "rf_receiver.h"
#include "rf_parser.h"
#include "main.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Display capture fake (row/col aware - channels draw two columns)    */
/* ------------------------------------------------------------------ */

#define CAP_ROWS 18
#define CAP_COLS 28
static char cap[CAP_ROWS][CAP_COLS + 1];
static uint32_t cap_updates = 0, cap_clears = 0;
static uint32_t cap_lines = 0, cap_circles = 0, cap_texts = 0, cap_pixels = 0;
static uint32_t cap_chars = 0;

static void cap_clear(void)
{
    memset(cap, 0, sizeof(cap));
    cap_updates = cap_clears = cap_lines = cap_circles = 0;
    cap_texts = cap_pixels = cap_chars = 0;
}

void Display_Clear(void) { cap_clears++; memset(cap, 0, sizeof(cap)); }
void Display_Update(void) { cap_updates++; }
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
    if (row >= CAP_ROWS || !text) return;
    /* Pad with spaces up to col, then copy */
    size_t l = strlen(cap[row]);
    while (l < (size_t)col && l < CAP_COLS) cap[row][l++] = ' ';
    strncpy(cap[row] + l, text, CAP_COLS - l);
    cap[row][CAP_COLS] = '\0';
}
void Display_DrawPixel(uint8_t x, uint8_t y, uint8_t c)
{ (void)x; (void)y; (void)c; cap_pixels++; }
void Display_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t c)
{ (void)x0; (void)y0; (void)x1; (void)y1; (void)c; cap_lines++; }
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t c)
{ (void)x0; (void)y0; (void)r; (void)c; cap_circles++; }
void Display_DrawChar(uint8_t x, uint8_t y, char c)
{ (void)x; (void)y; (void)c; cap_chars++; }
void Display_DrawText(uint8_t x, uint8_t y, const char *t)
{ (void)x; (void)y; (void)t; cap_texts++; }

static const char *row(int r) { return cap[r]; }

/* ------------------------------------------------------------------ */
/* HAL GPIO/SPI fakes (sd_card_mode reads real pins)                 */
/* ------------------------------------------------------------------ */

static uint8_t pin_level[8][16];   /* [port sel][pin idx]: 0/1 */

static int port_idx(GPIO_TypeDef *p)
{
    if (p == GPIOA) return 0;
    if (p == GPIOB) return 1;
    return 7;
}
static int pin_idx(uint16_t pin)
{
    int i = 0;
    while (pin >>= 1) i++;
    return i;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return pin_level[port_idx(port)][pin_idx(pin)] ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    pin_level[port_idx(port)][pin_idx(pin)] = (st == GPIO_PIN_SET);
}
void HAL_Delay(uint32_t ms) { (void)ms; }

SPI_HandleTypeDef hspi1;
static HAL_StatusTypeDef fake_spi_status = HAL_OK;
static uint8_t           fake_spi_rx = 0x00;
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                          uint8_t *rx, uint16_t size,
                                          uint32_t timeout)
{
    (void)h; (void)timeout;
    for (uint16_t i = 0; i < size; i++) rx[i] = fake_spi_rx;
    (void)tx;
    return fake_spi_status;
}

/* ------------------------------------------------------------------ */
/* RF layer fakes                                                     */
/* ------------------------------------------------------------------ */

static uint8_t  fake_channel = 0;
static uint8_t  fake_scanning = 0;
static uint32_t fake_last_pkt_ms = 0;
static uint32_t fake_lora_pkts = 0;
static uint32_t fake_irq_checks = 0;
static uint16_t fake_last_irq = 0;
static uint8_t  fake_dev_mode = 0;
static uint8_t  fake_spi_test = 0xFF;
static uint8_t  fake_busy = 0;
static uint32_t fake_dio1_irqs = 0, fake_dups = 0;
static int16_t  fake_rssi = 0;
static int8_t   fake_snr = 0;
static uint8_t  fake_nf_ok = 0;
static int16_t  fake_nf = 0;
static uint8_t  fake_noise_alert = 0;
static uint32_t fake_foreign = 0;
static uint8_t  fake_bound = 0xFF;
static char     fake_ascii[64] = "";
static uint16_t fake_ascii_len = 0;
static uint8_t  fake_testing = 0;

uint8_t  RF_Receiver_GetChannel(void)        { return fake_channel; }
uint8_t  RF_Receiver_IsScanning(void)        { return fake_scanning; }
uint32_t RF_Receiver_GetLastPacketTime(void) { return fake_last_pkt_ms; }
uint32_t RF_Receiver_GetLoRaPacketCount(void){ return fake_lora_pkts; }
void RF_Receiver_GetIRQDiagnostics(uint32_t *c, uint16_t *irq,
                                   uint8_t *mode, uint8_t *spi, uint8_t *busy)
{
    if (c)    *c    = fake_irq_checks;
    if (irq)  *irq  = fake_last_irq;
    if (mode) *mode = fake_dev_mode;
    if (spi)  *spi  = fake_spi_test;
    if (busy) *busy = fake_busy;
}
void RF_Receiver_GetPacketLossDiagnostics(uint32_t *d, uint32_t *p, uint32_t *dup)
{
    if (d)   *d   = fake_dio1_irqs;
    if (p)   *p   = fake_lora_pkts;
    if (dup) *dup = fake_dups;
}
void RF_Receiver_GetSignalQuality(int16_t *r, int8_t *s)
{
    if (r) *r = fake_rssi;
    if (s) *s = fake_snr;
}
uint8_t  RF_Receiver_GetNoiseFloor(int16_t *nf)
{
    if (!fake_nf_ok || !nf) return 0;
    *nf = fake_nf;
    return 1;
}
uint8_t  RF_Receiver_NoiseAlert(void)        { return fake_noise_alert; }
uint32_t RF_Receiver_GetForeignDrops(void)   { return fake_foreign; }
uint8_t  RF_Receiver_GetBoundRocketId(void)  { return fake_bound; }
uint16_t RF_Receiver_GetLastPacketASCII(char *buf, uint16_t max_len)
{
    if (fake_ascii_len == 0 || !buf || max_len == 0) return 0;
    uint16_t n = fake_ascii_len < (uint16_t)(max_len - 1)
               ? fake_ascii_len : (uint16_t)(max_len - 1);
    memcpy(buf, fake_ascii, n);
    buf[n] = '\0';
    return n;
}
uint8_t  RF_Receiver_TestingBuildSuspect(void) { return fake_testing; }

static uint32_t fake_p_attempts = 0, fake_p_ok = 0, fake_p_fail = 0;
static uint32_t fake_p_null = 0;
void RF_Parser_GetDiagnostics(uint32_t *a, uint32_t *s, uint32_t *f)
{
    if (a) *a = fake_p_attempts;
    if (s) *s = fake_p_ok;
    if (f) *f = fake_p_fail;
}
void RF_Parser_GetDetailedFailures(uint32_t *nul)
{
    if (nul) *nul = fake_p_null;
}

/* ------------------------------------------------------------------ */
/* SD + compass fakes                                                 */
/* ------------------------------------------------------------------ */

static SD_Card_Info fake_sd;
static uint8_t fake_sd_status = SD_CARD_OK;
SD_Card_Status SD_Card_GetInfo(SD_Card_Info *info)
{
    if (info) *info = fake_sd;
    return fake_sd_status;
}

static uint8_t  fake_compass_update = COMPASS_OK;
static uint8_t  fake_cal[4] = {3, 3, 3, 3};
static uint32_t fake_rv_count = 0;
static uint8_t  fake_sys = 5, fake_self = 0x0F, fake_syserr = 0;
static const char *fake_errmsg = "none";
static uint8_t  fake_errcode = 0;

uint8_t Compass_Update(Compass_Data *d) { (void)d; return fake_compass_update; }
uint8_t Compass_GetCalibrationStatus(uint8_t *s, uint8_t *g, uint8_t *a, uint8_t *m)
{
    if (s) *s = fake_cal[0];
    if (g) *g = fake_cal[1];
    if (a) *a = fake_cal[2];
    if (m) *m = fake_cal[3];
    return COMPASS_OK;
}
uint8_t Compass_GetSystemStatus(uint8_t *s, uint8_t *st, uint8_t *e)
{
    if (s)  *s  = fake_sys;
    if (st) *st = fake_self;
    if (e)  *e  = fake_syserr;
    return COMPASS_OK;
}
uint32_t Compass_GetRvSampleCount(void)   { return fake_rv_count; }
const char* Compass_GetErrorMessage(void) { return fake_errmsg; }
uint8_t Compass_GetErrorCode(void)        { return fake_errcode; }

static void reset_fakes(void)
{
    cap_clear();
    memset(pin_level, 0, sizeof(pin_level));
    fake_spi_status = HAL_OK; fake_spi_rx = 0;
    fake_channel = 0; fake_scanning = 0; fake_last_pkt_ms = 0;
    fake_lora_pkts = 0; fake_irq_checks = 0; fake_last_irq = 0;
    fake_dev_mode = 0; fake_spi_test = 0xFF; fake_busy = 0;
    fake_dio1_irqs = 0; fake_dups = 0; fake_rssi = 0; fake_snr = 0;
    fake_nf_ok = 0; fake_nf = 0; fake_noise_alert = 0;
    fake_foreign = 0; fake_bound = 0xFF;
    fake_ascii_len = 0; fake_ascii[0] = '\0'; fake_testing = 0;
    fake_p_attempts = fake_p_ok = fake_p_fail = 0;
    fake_p_null = 0;
    memset(&fake_sd, 0, sizeof(fake_sd));
    fake_sd_status = SD_CARD_OK;
    fake_compass_update = COMPASS_OK;
    fake_cal[0] = fake_cal[1] = fake_cal[2] = fake_cal[3] = 3;
    fake_rv_count = 0;
    fake_sys = 5; fake_self = 0x0F; fake_syserr = 0;
    fake_errmsg = "none"; fake_errcode = 0;
}

/* ------------------------------------------------------------------ */
/* channel_mode                                                        */
/* ------------------------------------------------------------------ */

TEST(channel_select_marks_current_and_layout)
{
    reset_fakes();
    fake_channel = 2;
    fake_scanning = 1;
    fake_lora_pkts = 42;
    Test_SetTick(100000);
    fake_last_pkt_ms = 88000;          /* 12 s ago */

    DisplayMode_ChannelSelect();
    CHECK(strstr(row(0), "ROCKET SELECT") != NULL);
    CHECK(strstr(row(0), "SCAN") != NULL);

    /* channels 0..3 in col 0 (rows 1-4), 4..7 at col 11 */
    CHECK(row(1)[0] == ' ');            /* ch0 not current  */
    CHECK(strstr(row(3), ">2 433.50") != NULL);   /* current marked */
    CHECK(strstr(row(1) + 11, "4 434.00") != NULL);
    CHECK(strstr(row(6), "Pkts:42") != NULL);
    CHECK(strstr(row(6), "Age:12s") != NULL);
    CHECK(strstr(row(7), "B2: next channel") != NULL);
}

TEST(channel_select_fixed_and_silent)
{
    reset_fakes();
    fake_channel = 7;
    fake_scanning = 0;
    fake_lora_pkts = 0;
    fake_last_pkt_ms = 0;

    DisplayMode_ChannelSelect();
    CHECK(strstr(row(0), "ROCKET SELECT") != NULL);
    CHECK(strstr(row(0), "SCAN") == NULL);          /* fixed-channel title */
    CHECK(strstr(row(4) + 11, ">7 434.75") != NULL); /* col 2 row 4 */
    CHECK(strstr(row(6), "Age:--") != NULL);
}

/* ------------------------------------------------------------------ */
/* rf_stats_mode                                                       */
/* ------------------------------------------------------------------ */

TEST(rf_stats_full_layout)
{
    reset_fakes();
    fake_channel = 3;
    fake_lora_pkts = 1234;
    fake_dio1_irqs = 1240;
    fake_rssi = -102; fake_snr = -5;
    fake_nf_ok = 1; fake_nf = -118; fake_noise_alert = 1;
    Test_SetTick(50000);
    fake_last_pkt_ms = 38000;          /* 12 s ago */
    fake_p_ok = 120; fake_p_fail = 1;
    fake_dev_mode = 5;                 /* RX */
    fake_spi_test = 1; fake_busy = 0;
    fake_bound = 3;

    DisplayMode_RFStats();
    CHECK(strstr(row(0), "RF STATS") != NULL);
    CHECK(strstr(row(0), "CH3 433.75") != NULL);
    CHECK(strstr(row(1), "SF") != NULL && strstr(row(1), "CR4/") != NULL);
    CHECK(strstr(row(2), "Pkts:1234") && strstr(row(2), "IRQ:1240"));
    /* noise floor + alert flag variant */
    CHECK(strstr(row(3), "NF:-118!") != NULL);
    CHECK(strstr(row(4), "Last pkt: 12s") != NULL);
    CHECK(strstr(row(5), "OK:120") && strstr(row(5), "F:1"));
    CHECK(strstr(row(6), "Mode:RX") && strstr(row(6), "SPI:OK"));
    CHECK(strstr(row(7), "Bnd:R3") != NULL);
    CHECK(strstr(row(7), "IRQst:0x") != NULL);
}

TEST(rf_stats_no_nf_and_foreign_row)
{
    reset_fakes();
    fake_nf_ok = 0;
    fake_rssi = -80; fake_snr = 8;
    fake_last_pkt_ms = 0;
    fake_dev_mode = 2;                 /* STBY */
    fake_spi_test = 0;                 /* FAIL */
    fake_foreign = 17;
    fake_bound = 0xFF;

    DisplayMode_RFStats();
    /* no NF estimate: falls back to the RSSI/SNR-only row */
    CHECK(strstr(row(3), "RSSI:-80") != NULL && strstr(row(3), "SNR:8") != NULL);
    CHECK(strstr(row(4), "Last pkt: none") != NULL);
    CHECK(strstr(row(6), "Mode:STBY") && strstr(row(6), "SPI:FAIL"));
    /* foreign traffic escalates to the warning form */
    CHECK(strstr(row(7), "FGN:17") != NULL);
    CHECK(strstr(row(7), "Bnd:--") != NULL);
}

/* ------------------------------------------------------------------ */
/* gps_mode                                                            */
/* ------------------------------------------------------------------ */

TEST(gps_mode_live_fix_layout)
{
    reset_fakes();
    GPS_Data g = {0}, last = {0};
    g.fix = 1; g.satellites = 9;
    g.latitude = 39.89444f; g.longitude = -104.88517f;
    g.altitude = 1655.4f;
    g.hour = 13; g.minute = 7; g.second = 42;
    g.day = 5; g.month = 11; g.year = 2026;

    DisplayMode_GPS(&g, &last, 0);
    CHECK(strstr(row(0), "Fix: Y") && strstr(row(0), "Sat: 9"));
    CHECK(strstr(row(1), "Lat: 39.89444") != NULL);
    CHECK(strstr(row(2), "Lon: -104.88517") != NULL);
    CHECK(strstr(row(3), "Alt: 1655.4m") != NULL);
    CHECK(strstr(row(4), "Time: 13:07:42") != NULL);
    CHECK(strstr(row(5), "Date: 05/11/26") != NULL);
}

TEST(gps_mode_last_good_fallback)
{
    reset_fakes();
    GPS_Data g = {0}, last = {0};
    g.fix = 0;                          /* current fix dead */
    last.fix = 1; last.satellites = 7;
    last.latitude = 40.125f; last.longitude = -105.25f;
    last.altitude = 1700.0f;
    last.hour = 1; last.minute = 2; last.second = 3;
    /* no date fields: guard must produce "No Date" */

    DisplayMode_GPS(&g, &last, 1);
    CHECK(strstr(row(0), "Fix: L (was Y)") != NULL);
    CHECK(strstr(row(1), "Lat: 40.12500") != NULL);
    CHECK(strstr(row(2), "Lon: -105.25000") != NULL);
    CHECK(strstr(row(5), "No Date") != NULL);
}

/* ------------------------------------------------------------------ */
/* sd_card_mode                                                        */
/* ------------------------------------------------------------------ */

TEST(sd_card_mode_present_gb)
{
    reset_fakes();
    fake_sd.is_present = 1;
    fake_sd.is_initialized = 1;
    fake_sd.is_mounted = 1;
    fake_sd.total_size_mb = 31999;       /* ~31 GB */
    fake_sd.free_space_mb = 2048;        /* 2 GB */
    fake_sd.files_logged = 7;
    fake_sd.bytes_written = 3u * 1024u * 1024u + 5u; /* MB branch */
    fake_sd.cmd0_response = 0x01;
    pin_level[port_idx(SD_SPI_MISO_GPIO_PORT)][pin_idx(SD_SPI_MISO_PIN)] = 1;
    pin_level[port_idx(SD_DETECT_GPIO_PORT)][pin_idx(SD_DETECT_PIN)] = 1;

    DisplayMode_SDCard();
    CHECK(strstr(row(0), "SD Card: Present") != NULL);
    CHECK(strstr(row(1), "Status: Ready") != NULL);
    CHECK(strstr(row(2), "Size: 31GB") != NULL);
    CHECK(strstr(row(3), "Free: 2GB") != NULL);
    CHECK(strstr(row(4), "Files: 7") != NULL);
    CHECK(strstr(row(5), "Wr: 3MB") != NULL);
    CHECK(strstr(row(6), "CMD0: 0x01") != NULL);
    CHECK(strstr(row(7), "MISO:H") && strstr(row(7), "DET:H"));
}

TEST(sd_card_mode_absent_and_error_decode)
{
    reset_fakes();
    /* absent */
    DisplayMode_SDCard();
    CHECK(strstr(row(0), "Not Found") != NULL);
    CHECK(strstr(row(1), "Insert SD card") != NULL);

    /* present but error code 13 = FORMAT_FAIL decode */
    fake_sd.is_present = 1;
    fake_sd.is_mounted = 0;
    fake_sd.write_errors = 13;
    cap_clear();
    DisplayMode_SDCard();
    CHECK(strstr(row(0), "SD Card: Present") != NULL);
    CHECK(strstr(row(1), "Status: Error") != NULL);
    CHECK(strstr(row(2), "Err: 13") != NULL);
    CHECK(strstr(row(3), "FORMAT_FAIL") != NULL);
}

/* ------------------------------------------------------------------ */
/* rf_mode                                                             */
/* ------------------------------------------------------------------ */

static GPS_Data rf_gps, rf_last;
static uint8_t rf_valid, rf_has_last;
static uint32_t rf_last_time, rf_count;

TEST(rf_mode_live_packet_layout)
{
    reset_fakes();
    memset(&rf_gps, 0, sizeof(rf_gps));
    rf_gps.fix = 1; rf_gps.satellites = 11;
    rf_gps.latitude = 39.9f; rf_gps.longitude = -105.0f;
    rf_gps.altitude = 1600.0f;
    rf_valid = 1; rf_has_last = 0;
    Test_SetTick(5000);
    rf_last_time = 3000;
    fake_lora_pkts = 100;
    fake_p_attempts = 110; fake_p_ok = 100; fake_p_fail = 10;
    fake_p_null = 4;
    fake_dev_mode = 5; fake_last_irq = 0x002;
    strcpy(fake_ascii, "KE0MZS-3 CH2 TELEMETRY");
    fake_ascii_len = 22;
    fake_testing = 0;

    DisplayMode_RF(&rf_gps, &rf_valid, &rf_last_time, &rf_count,
                   &rf_last, rf_has_last);
    CHECK(strstr(row(0), "Last: 2000 ms") != NULL);
    /* row 1: parse counters - successes/attempts, failures, NULL packets */
    CHECK(strstr(row(1), "P: 100/110 F") != NULL);
    CHECK(strstr(row(1), " N4") != NULL);
    CHECK(strstr(row(2), "Lat: 39.90000") != NULL);
    CHECK(strstr(row(3), "Lon: -105.00000") != NULL);
    CHECK(strstr(row(5), "Fix: Y") && strstr(row(5), "Sat: 11"));
    /* ASCII split: first 15 chars on row 6, remainder on row 7 */
    CHECK(strncmp(row(6), "KE0MZS-3 CH2 TE", 15) == 0);
    CHECK(strncmp(row(7), "LEMETRY", 7) == 0);
}

TEST(rf_mode_last_good_and_testing_star)
{
    reset_fakes();
    memset(&rf_gps, 0, sizeof(rf_gps));
    memset(&rf_last, 0, sizeof(rf_last));
    rf_last.fix = 1; rf_last.satellites = 6;
    rf_last.latitude = 40.0f; rf_last.longitude = -104.0f;
    rf_last.altitude = 1500.0f;
    rf_valid = 0; rf_has_last = 1;
    rf_last_time = 1000;
    Test_SetTick(5000);
    strcpy(fake_ascii, "KE0MZS-3 CH2");
    fake_ascii_len = 12;
    fake_testing = 1;                  /* wraps ID line in '*' */

    DisplayMode_RF(&rf_gps, &rf_valid, &rf_last_time, &rf_count,
                   &rf_last, rf_has_last);
    /* last-good markers on all three coordinate rows */
    CHECK(strstr(row(2), "(L)") != NULL);
    CHECK(strstr(row(3), "(L)") != NULL);
    CHECK(strstr(row(4), "(LAST)") != NULL);
    CHECK(strstr(row(5), "Fix:L(was Y)") != NULL);
    /* testing build wraps the ID line */
    CHECK(row(6)[0] == '*');
    CHECK(strchr(row(6) + 1, '*') != NULL);
}

TEST(rf_mode_no_gps_no_packets)
{
    reset_fakes();
    memset(&rf_gps, 0, sizeof(rf_gps));
    rf_valid = 0; rf_has_last = 0;
    rf_last_time = 0;
    fake_ascii_len = 0;
    fake_lora_pkts = 0;
    fake_dev_mode = 5;
    fake_last_irq = 0x40;

    DisplayMode_RF(&rf_gps, &rf_valid, &rf_last_time, &rf_count,
                   &rf_last, rf_has_last);
    /* no packets ever: row 0 shows the chip/IRQ state instead */
    CHECK(strstr(row(0), "L:0") && strstr(row(0), "IRQ:040"));
    CHECK(strstr(row(6), "No RF data received") != NULL);
}

/* ------------------------------------------------------------------ */
/* compass_mode                                                        */
/* ------------------------------------------------------------------ */

static Compass_Data cdata;
static float cd_heading;

TEST(compass_heading_live_layout)
{
    reset_fakes();
    memset(&cdata, 0, sizeof(cdata));
    cdata.heading = 123.4f; cdata.heading_valid = 1; cdata.orientation_valid = 1;
    cdata.pitch = 1.5f; cdata.roll = -2.5f;
    cdata.accel_x = 10; cdata.accel_y = 20; cdata.accel_z = 30;
    cdata.gyro_x = 1; cdata.gyro_y = 2; cdata.gyro_z = 3;
    cdata.mag_x = -100; cdata.mag_y = 200; cdata.mag_z = 300;
    cd_heading = 0;

    DisplayMode_CompassHeading(&cdata, &cd_heading);
    CHECK(cd_heading > 123.0f && cd_heading < 124.0f); /* exported heading */
    CHECK(strstr(row(0), "Hdg:123") != NULL && row(0)[strlen(row(0))-1] == 'd');
    CHECK(strstr(row(1), "P:  1.5") != NULL);
    CHECK(strstr(row(2), "Cal S3 G3 A3 M3") != NULL);
    CHECK(strstr(row(6), "St:05") && strstr(row(6), "Er:00"));
    CHECK(strstr(row(7), "RV#0") != NULL);
}

TEST(compass_heading_stale_marker)
{
    reset_fakes();
    memset(&cdata, 0, sizeof(cdata));
    cdata.heading = 90.0f; cdata.heading_stale = 1;
    fake_rv_count = 4242;

    DisplayMode_CompassHeading(&cdata, &cd_heading);
    CHECK(strstr(row(0), "Hdg: 90d*") != NULL);  /* * = stale */
    CHECK(strstr(row(7), "RV#4242") != NULL);
}

TEST(compass_update_failure_no_response)
{
    reset_fakes();
    memset(&cdata, 0, sizeof(cdata));
    fake_compass_update = COMPASS_ERROR;
    cd_heading = -1;

    DisplayMode_CompassHeading(&cdata, &cd_heading);
    CHECK(strstr(row(3), "Compass no response") != NULL);
    CHECK(cd_heading == 0.0f);          /* default heading assigned */
}

TEST(compass_visual_rose_and_invalid_heading)
{
    reset_fakes();
    memset(&cdata, 0, sizeof(cdata));
    cdata.heading = 45.0f; cdata.heading_valid = 1; cdata.orientation_valid = 1;

    DisplayMode_CompassVisual(&cdata, &cd_heading);
    CHECK(cap_circles == 2);            /* outer ring + centre dot */
    CHECK(cap_lines == 9);              /* 8 ticks + 1 needle */
    CHECK(cap_texts >= 4);              /* at least the N S E W cardinals */

    /* orientation invalid (device vertical): no needle, status text */
    memset(&cdata, 0, sizeof(cdata));
    cdata.orientation_valid = 0;
    cap_clear();
    DisplayMode_CompassVisual(&cdata, &cd_heading);
    CHECK(cap_lines == 8);              /* ticks only, needle suppressed */
}

/* ------------------------------------------------------------------ */
/* test_pattern                                                       */
/* ------------------------------------------------------------------ */

TEST(test_pattern_corner_layout)
{
    reset_fakes();
    Display_ShowTestPattern();
    /* 168x144 panel, 6x8 font: 28 cols x 18 rows */
    CHECK(strstr(row(0), "TL") != NULL);
    CHECK(strstr(row(17), "BL") != NULL);
    CHECK(strstr(row(0) + 26, "TR") != NULL);
    CHECK(strstr(row(17) + 26, "BR") != NULL);
    CHECK(strstr(row(8), "CENTER") != NULL);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_channel_select_marks_current_and_layout();
    run_channel_select_fixed_and_silent();
    run_rf_stats_full_layout();
    run_rf_stats_no_nf_and_foreign_row();
    run_gps_mode_live_fix_layout();
    run_gps_mode_last_good_fallback();
    run_sd_card_mode_present_gb();
    run_sd_card_mode_absent_and_error_decode();
    run_rf_mode_live_packet_layout();
    run_rf_mode_last_good_and_testing_star();
    run_rf_mode_no_gps_no_packets();
    run_compass_heading_live_layout();
    run_compass_heading_stale_marker();
    run_compass_update_failure_no_response();
    run_compass_visual_rose_and_invalid_heading();
    run_test_pattern_corner_layout();
    return TEST_SUMMARY();
}
