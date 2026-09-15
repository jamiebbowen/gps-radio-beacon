/**
 * @file test_preflight_mode.c
 * @brief Host tests for the go/no-go preflight display page.
 *
 * display_modes/preflight_mode.c is compiled unmodified; the display layer
 * is capture-faked and the two RF reads preflight needs are stubbed. Covers
 * every verdict row plus -crucially for launch day- the battery-watchdog
 * gating (RX supply rail sag at the pad must veto "READY TO FLY").
 *
 * Build & run:  make -C receiver/tests
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "display_modes/preflight_mode.h"
#include "gps.h"
#include "packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Display capture fake                                                */
/* ------------------------------------------------------------------ */

#define CAP_ROWS 8
static char cap_row_text[CAP_ROWS][40];

void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
    if (row >= CAP_ROWS) return;
    if (col != 0) {
        size_t l = strlen(cap_row_text[row]);
        if (l == 0) while (l < (size_t)col) cap_row_text[row][l++] = ' ';
        strncpy(cap_row_text[row] + l, text, sizeof(cap_row_text[row]) - 1 - l);
        cap_row_text[row][40 - 1] = '\0';
        return;
    }
    strncpy(cap_row_text[row], text, sizeof(cap_row_text[row]) - 1);
    cap_row_text[row][40 - 1] = '\0';
}

static void cap_clear(void) { memset(cap_row_text, 0, sizeof(cap_row_text)); }

/* ------------------------------------------------------------------ */
/* RF layer stubs (the two reads preflight_mode.c performs from rf)    */
/* ------------------------------------------------------------------ */

static int fake_testing_build = 0;
uint8_t RF_Receiver_TestingBuildSuspect(void) { return fake_testing_build; }

static const char *fake_callsign = "";
uint8_t RF_Receiver_GetParsedData(GPS_Data *out_data, char *out_callsign,
                                  uint32_t callsign_len, uint8_t *out_age)
{
    (void)out_age;
    memset(out_data, 0, sizeof(*out_data));
    if (out_callsign && callsign_len) {
        strncpy(out_callsign, fake_callsign, callsign_len - 1);
        out_callsign[callsign_len - 1] = '\0';
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* Call shortcuts                                                     */
/* ------------------------------------------------------------------ */

static const char *page_row(int r) { return cap_row_text[r]; }

static void render(uint8_t link_ok, uint32_t link_age_s, int16_t rssi,
                   uint8_t tx_fix, uint8_t tx_sats, uint8_t tx_hb_state,
                   uint8_t sensor_deg,
                   uint8_t rx_fix_ok, uint8_t rx_sats,
                   uint8_t compass_ok, uint8_t sd_ok, uint16_t vdd)
{
    cap_clear();
    DisplayMode_Preflight(link_ok, link_age_s, rssi,
                          tx_fix, tx_sats, tx_hb_state, sensor_deg,
                          rx_fix_ok, rx_sats, compass_ok, sd_ok, vdd);
}

/* ------------------------------------------------------------------ */
/* Tests                                                              */
/* ------------------------------------------------------------------ */

TEST(test_preflight_all_green_ready)
{
    /* link <35s, RSSI strong, TX 9 sat fix1, hb state nominal,
     * sensor healthy, RX fixed with 8 sats, compass ok, sd ok, rail 3.30V */
    render(1, 3, -55, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(0), "PRE-FLIGHT CHECK") != NULL);
    CHECK(strstr(page_row(1), "TX LINK  OK") != NULL);
    CHECK(strstr(page_row(2), "TX GPS   OK 9sat") != NULL);
    CHECK(strstr(page_row(3), "RX GPS   OK 8sat") != NULL);
    CHECK(strstr(page_row(4), "COMPASS  OK") != NULL);
    CHECK(strstr(page_row(4), "3.3V") != NULL);
    CHECK(strstr(page_row(5), "SD CARD  OK") != NULL);
    CHECK(strstr(page_row(6), "TX IMU   OK") != NULL);
    CHECK(strstr(page_row(7), "READY TO FLY") != NULL);
}

TEST(test_preflight_link_missing)
{
    render(0, 0, -60, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(1), "CHK no pkts") != NULL);
    CHECK(strstr(page_row(7), "NOT READY") != NULL);
}

TEST(test_preflight_weak_link_rssi)
{
    /* link ok but RSSI under the -75 dBm pad floor: "WEAK", advisory only -
     * the verdict can still be READY (it's a pad-sanity hint, not a gate). */
    render(1, 3, -90, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(1), "WEAK -90dBm") != NULL);
}

TEST(test_preflight_testing_build_flag)
{
    fake_testing_build = 1;
    fake_callsign = "KE0MZS-3";
    render(1, 3, -55, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(1), "*KE0MZS-3*") != NULL);
    fake_testing_build = 0;
}

TEST(test_preflight_testing_build_no_callsign)
{
    fake_testing_build = 1;
    fake_callsign = "";
    render(1, 3, -55, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(1), "TST BUILD?") != NULL);
    fake_testing_build = 0;
}

TEST(test_preflight_tx_gps_states)
{
    /* No fix but heartbeat says ACQUIRING and reports 3 sats */
    render(1, 2, -50, 0, 3, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(2), "ACQ 3sat") != NULL);
    CHECK(strstr(page_row(7), "NOT READY") != NULL);

    /* Heartbeat says wiring fault */
    render(1, 2, -50, 0, 0, HB_GPS_NO_DATA, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(2), "wiring!") != NULL);

    /* Heartbeat says garbled NMEA */
    render(1, 2, -50, 0, 0, HB_GPS_NO_NMEA, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(2), "garbled") != NULL);

    /* No heartbeat at all (0xFF): fall through to the CHK row */
    render(1, 2, -50, 0, 2, 0xFF, 0, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(2), "CHK 2sat f0") != NULL);
}

TEST(test_preflight_rx_gps_missing)
{
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 0, 0, 1, 1, 3304);
    CHECK(strstr(page_row(3), "CHK no fix") != NULL);
    CHECK(strstr(page_row(7), "NOT READY") != NULL);
}

TEST(test_preflight_battery_low_gates_verdict)
{
    /* Everything else healthy, but pack is sagging to 3.1V on pad:
     * must veto READY, and the power row warns*/
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3100);
    CHECK(strstr(page_row(4), "PWR LOW") != NULL);
    CHECK(strstr(page_row(4), "3.10V") != NULL);
    CHECK(strstr(page_row(7), "PWR LOW") != NULL);
    CHECK(strstr(page_row(7), "READY TO FLY") == NULL);
}

TEST(test_preflight_battery_edge)
{
    /* 3200 mV: right AT the warning boundary = still OK */
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3200);
    CHECK(strstr(page_row(4), "COMPASS  OK") != NULL);
    CHECK(strstr(page_row(7), "READY TO FLY") != NULL);

    /* 3199 mV: warning engages */
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 3199);
    CHECK(strstr(page_row(7), "PWR LOW") != NULL);
}

TEST(test_preflight_battery_zero_means_unmeasured)
{
    /* vdd=0 = ADC never ran this boot path (bring-up edge). Gate must NOT
     * veto READY: an unmeasured rail is not evidence of a sag. */
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 1, 0);
    CHECK(strstr(page_row(7), "READY TO FLY") != NULL);
}

TEST(test_preflight_sensor_degraded_advisory)
{
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 1, 1, 8, 1, 1, 3304);
    CHECK(strstr(page_row(6), "TX IMU   CHK dead!") != NULL);
    CHECK(strstr(page_row(7), "READY TO FLY") != NULL);  /* advisory only */
}

TEST(test_preflight_sd_advisory)
{
    render(1, 2, -50, 1, 9, HB_GPS_ACQUIRING, 0, 1, 8, 1, 0, 3304);
    CHECK(strstr(page_row(5), "WARN nolog") != NULL);
    CHECK(strstr(page_row(7), "READY TO FLY") != NULL);  /* sd not a gate */
}

int main(void)
{
    run_test_preflight_all_green_ready();
    run_test_preflight_link_missing();
    run_test_preflight_weak_link_rssi();
    run_test_preflight_testing_build_flag();
    run_test_preflight_testing_build_no_callsign();
    run_test_preflight_tx_gps_states();
    run_test_preflight_rx_gps_missing();
    run_test_preflight_battery_low_gates_verdict();
    run_test_preflight_battery_edge();
    run_test_preflight_battery_zero_means_unmeasured();
    run_test_preflight_sensor_degraded_advisory();
    run_test_preflight_sd_advisory();
    return TEST_SUMMARY();
}
