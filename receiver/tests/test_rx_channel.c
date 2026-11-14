/**
 * test_rx_channel.c - channel selection and its breadcrumbs (rx_channel.c)
 *
 * The boot scan's remembered channel, the manual pick and the scan lock
 * lived inline in main.c (no host build). The radio and the card are
 * faked at their API: this pins what each event asks of them.
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/rx_channel.c"
#include "../firmware/src/beacon_track.c"
#include "test_harness.h"

/* ---- rf_receiver fakes ---------------------------------------------- */
static uint8_t fake_ch = 0, fake_next_ch = 1, fake_scan_lock = 0;
static int     scan_starts = 0, scan_starts_from = 0, scan_stops = 0;
static uint8_t scan_from_ch = 0xFF;
uint8_t RF_Receiver_GetChannel(void)  { return fake_ch; }
uint8_t RF_Receiver_NextChannel(void) { fake_ch = fake_next_ch; return fake_ch; }
void    RF_Receiver_StartScan(void)   { scan_starts++; }
void    RF_Receiver_StartScanFrom(uint8_t ch) { scan_starts_from++; scan_from_ch = ch; }
void    RF_Receiver_StopScan(void)    { scan_stops++; }
uint8_t RF_Receiver_ScanUpdate(void)  { return fake_scan_lock; }

/* ---- sd_card fakes --------------------------------------------------- */
static uint8_t fake_lastch_ok = 0, fake_lastch = 0, saved_ch = 0xFF;
static int     ch_saves = 0, rows = 0, ensures = 0;
static char    last_row[64];
SD_Card_Status SD_Card_LoadLastChannel(uint8_t *ch)
{
    if (!fake_lastch_ok) return SD_CARD_ERROR;
    *ch = fake_lastch;
    return SD_CARD_OK;
}
SD_Card_Status SD_Card_SaveLastChannel(uint8_t ch) { ch_saves++; saved_ch = ch; return SD_CARD_OK; }
SD_Card_Status SD_Card_LogEvent(const char *m)
{
    rows++;
    snprintf(last_row, sizeof(last_row), "%s", m);
    return SD_CARD_OK;
}
SD_Card_Status SD_Card_EnsureLogFile(void) { ensures++; return SD_CARD_OK; }

static void reset(void)
{
    fake_ch = 0; fake_next_ch = 1; fake_scan_lock = 0;
    scan_starts = scan_starts_from = scan_stops = 0; scan_from_ch = 0xFF;
    fake_lastch_ok = 0; fake_lastch = 0; saved_ch = 0xFF;
    ch_saves = rows = ensures = 0; last_row[0] = '\0';
}

static const char *row_for(const char *fmt, uint8_t ch)
{
    static char buf[64];
    snprintf(buf, sizeof(buf), fmt, (unsigned)ch, LORA_CHANNEL_FREQ_MHZ(ch));
    return buf;
}

TEST(test_scan_resumes_on_the_remembered_channel)
{
    reset();
    RxChannel_StartScan(0);                       /* no card */
    CHECK(scan_starts == 1 && scan_starts_from == 0);

    reset();
    RxChannel_StartScan(1);                       /* card, no LASTCH.BIN */
    CHECK(scan_starts == 1 && scan_starts_from == 0);

    reset();
    fake_lastch_ok = 1; fake_lastch = 5;
    RxChannel_StartScan(1);                       /* remembered CH5 */
    CHECK(scan_starts == 0 && scan_starts_from == 1 && scan_from_ch == 5);
}

TEST(test_manual_step_resets_logs_and_remembers)
{
    BeaconTrack_t bt;
    BeaconTrack_Init(&bt);
    BeaconTrack_Restore(&bt, 39.89f, -105.11f, 1650.0f);
    bt.pos_tick = 12345;

    reset();
    fake_ch = 2; fake_next_ch = 3;
    CHECK(RxChannel_ManualStep(&bt, 1) == 1);
    CHECK(scan_stops == 1);                        /* a pick overrides the scan */
    CHECK(bt.has_valid == 0 && bt.has_last_good == 0 && bt.pos_tick == 0);
    CHECK(rows == 1 && strcmp(last_row, row_for("RF manual CH%u %.2fMHz", 3)) == 0);
    CHECK(ch_saves == 1 && saved_ch == 3);

    /* No card: same reset, nothing written */
    BeaconTrack_Restore(&bt, 39.89f, -105.11f, 1650.0f);
    reset();
    fake_ch = 3; fake_next_ch = 4;
    CHECK(RxChannel_ManualStep(&bt, 0) == 1);
    CHECK(bt.has_valid == 0 && rows == 0 && ch_saves == 0);

    /* The step landed on the same channel: nothing belongs elsewhere */
    BeaconTrack_Restore(&bt, 39.89f, -105.11f, 1650.0f);
    reset();
    fake_ch = 4; fake_next_ch = 4;
    CHECK(RxChannel_ManualStep(&bt, 1) == 0);
    CHECK(scan_stops == 1 && bt.has_valid == 1 && rows == 0 && ch_saves == 0);
}

TEST(test_scan_lock_logs_and_remembers)
{
    reset();
    fake_ch = 6;
    CHECK(RxChannel_ServiceScan(1) == 0);          /* still scanning */
    CHECK(rows == 0 && ch_saves == 0 && ensures == 0);

    fake_scan_lock = 1;
    CHECK(RxChannel_ServiceScan(1) == 1);
    CHECK(ensures == 1);                           /* log file exists first */
    CHECK(rows == 1 && strcmp(last_row, row_for("RF scan locked CH%u %.2fMHz", 6)) == 0);
    CHECK(ch_saves == 1 && saved_ch == 6);

    reset();
    fake_scan_lock = 1;
    CHECK(RxChannel_ServiceScan(0) == 1);          /* no card: lock still reported */
    CHECK(rows == 0 && ch_saves == 0 && ensures == 0);
}

int main(void)
{
    run_test_scan_resumes_on_the_remembered_channel();
    run_test_manual_step_resets_logs_and_remembers();
    run_test_scan_lock_logs_and_remembers();
    return TEST_SUMMARY();
}
