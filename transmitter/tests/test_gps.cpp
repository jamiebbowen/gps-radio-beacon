/**
 * @file test_gps.cpp
 * @brief Host-side unit tests for the transmitter GPS driver (gps.cpp).
 *
 * Fakes the UART byte queue and clock to exercise NMEA sentence assembly
 * (GGA + RMC, GN/GP talkers), absolute-position field extraction with
 * empty-field preservation, the EKF feed, gps_get_health()'s three no-fix
 * verdicts, and the GPS watchdog: UART-silent recovery, no-fix cold
 * restart, the 3-attempt limit, and the 10-minute cooldown re-arm added
 * in the resiliency review.
 *
 * Links the real nmea_fields.c (field extraction + coordinate math).
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include "include/gps.h"
#include "include/packet_format.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / UART / nav fakes                                          */
/* ------------------------------------------------------------------ */

FakeSerial Serial;

static unsigned long now_ms = 50000;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }

/* UART RX queue the test feeds bytes into */
static uint8_t  uq[4096];
static size_t   uq_head = 0, uq_tail = 0;

bool uart_data_available(void) { return uq_head != uq_tail; }
uint8_t uart_read_byte(void)
{
    if (uq_head == uq_tail) return 0;
    return uq[uq_head++ % sizeof(uq)];
}
void flush_uart_buffer(void) {}

/* UART TX capture (GPS config commands + watchdog resets) */
static char     uart_tx_log[2048];
static size_t   uart_tx_len = 0;
void uart_tx_byte(uint8_t b)
{
    if (uart_tx_len + 1 < sizeof(uart_tx_log)) {
        uart_tx_log[uart_tx_len++] = (char)b;
        uart_tx_log[uart_tx_len] = '\0';
    }
}
void uart_tx_string(const char *s)
{
    while (*s) uart_tx_byte((uint8_t)*s++);
}

/* nav layer capture */
static int   nav_updates = 0;
static float nav_last_lat = 0, nav_last_lon = 0, nav_last_alt = 0;
static uint8_t nav_last_sats = 0, nav_last_fq = 0;
extern "C" void nav_update_from_gps(float lat_deg, float lon_deg, float alt_m,
                                    uint8_t sats, uint8_t fix_quality)
{
    nav_updates++;
    nav_last_lat = lat_deg; nav_last_lon = lon_deg; nav_last_alt = alt_m;
    nav_last_sats = sats;   nav_last_fq  = fix_quality;
}

/* Include the module under test (real nmea_fields.c linked separately) */
#include "../firmware/gps.cpp"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void feed(const char *s)
{
    while (*s) uq[uq_tail++ % sizeof(uq)] = (uint8_t)*s++;
}

static void reset_uart_capture(void)
{
    uart_tx_len = 0;
    uart_tx_log[0] = '\0';
}

/** Feed one sentence and poll it through.
 * The second (empty) poll matters: the watchdog latches its "last byte"
 * timestamp by comparing byte counters at the TOP of the next poll, so
 * without it the freshness clocks lag one call behind the data. */
static void feed_and_poll(const char *sentence)
{
    feed(sentence);
    (void)gps_poll_rx();
    (void)gps_poll_rx();
}

#define GGA_FIX \
    "$GPGGA,123519,3953.40284,N,10453.11007,W,1,08,0.9,1655.4,M,46.9,M,,*47\r\n"
#define RMC_FIX \
    "$GNRMC,123519,A,3954.00000,N,10454.00000,W,022.4,084.4,230394,,*6A\r\n"

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_sends_ublox_config)
{
    reset_uart_capture();
    gps_init();
    CHECK(gpsInitialized == 1);
    CHECK(strstr(uart_tx_log, "$PUBX,40,GGA,1,1") != NULL);   /* GGA on  */
    CHECK(strstr(uart_tx_log, "$PUBX,40,GLL,1,0") != NULL);   /* GLL off */
    CHECK(strstr(uart_tx_log, "$PUBX,40,RMC,1,1") != NULL);   /* RMC on  */

    /* gps_tx_string passthrough (and NULL safety) */
    reset_uart_capture();
    gps_tx_string("hello");
    CHECK(strcmp(uart_tx_log, "hello") == 0);
    gps_tx_string(NULL);
}

TEST(test_gga_full_fix_parsed_and_fed_to_nav)
{
    nav_updates = 0;
    feed_and_poll(GGA_FIX);

    const GPSCoordinates_t *c = gps_get_current_coordinates();
    CHECK(c->valid == 1);
    CHECK(strcmp(c->lat, "3953.40284") == 0);
    CHECK(strcmp(c->lon, "10453.11007") == 0);
    CHECK(c->lat_dir == 'N' && c->lon_dir == 'W');
    CHECK(c->fix_quality == 1);
    CHECK(strcmp(c->satellites, "08") == 0);
    CHECK(strcmp(c->altitude, "1655.4") == 0);

    /* The GGA fix was pushed into the EKF with converted coordinates */
    CHECK(nav_updates == 1);
    CHECK(nav_last_lat > 39.88f && nav_last_lat < 39.90f);
    CHECK(nav_last_lon < -104.88f && nav_last_lon > -104.89f);
    CHECK(fabsf(nav_last_alt - 1655.4f) < 0.1f);
    CHECK(nav_last_sats == 8 && nav_last_fq == 1);
}

TEST(test_burst_drains_all_sentences_in_one_poll)
{
    /* A 1 Hz burst arrives as several back-to-back sentences. One poll
     * must parse them ALL: the old one-sentence-per-call flow (plus the
     * hardware flush at the top of the next call) locked onto the burst's
     * first sentence and starved the GGA-only EKF feed - field logs
     * 2026-08-30: FUS age pinned at 255 ds / anchor never confirming
     * while raw RMC positions streamed. */
    nav_updates = 0;
    feed(GGA_FIX);
    feed(RMC_FIX);
    (void)gps_poll_rx();                        /* one call, whole burst */

    const GPSCoordinates_t *c = gps_get_current_coordinates();
    CHECK(nav_updates == 1);                    /* GGA fed the EKF */
    CHECK(strcmp(c->lat, "3954.00000") == 0);   /* RMC parsed after it */
    CHECK(strcmp(c->satellites, "08") == 0);    /* GGA fields landed */
}

TEST(test_rmc_updates_position_only)
{
    nav_updates = 0;
    feed_and_poll(RMC_FIX);

    const GPSCoordinates_t *c = gps_get_current_coordinates();
    CHECK(strcmp(c->lat, "3954.00000") == 0);   /* moved */
    CHECK(strcmp(c->lon, "10454.00000") == 0);
    CHECK(nav_updates == 0);                    /* only GGA feeds the EKF */
}

TEST(test_framing_error_bytes_skipped_pre_and_mid_sentence)
{
    /* 0xFE/0xFF UART framing glitches must be dropped both while seeking
     * '$' and inside a sentence, without stalling or overflowing. */
    feed("\xFE\xFF");                /* glitches before the '$' */
    feed_and_poll(GGA_FIX);
    const GPSCoordinates_t *c = gps_get_current_coordinates();
    CHECK(strcmp(c->lat, "3953.40284") == 0);

    /* Mid-sentence glitches: dropped before buffering, so the parser sees
     * the de-glitched text (no checksum layer on TX to reject it). */
    feed("$GPRMC,1235\xFE\xFF19,A,3954.5,N,10454.5,W,,,230394,,*6A\r\n");
    (void)gps_poll_rx();
    (void)gps_poll_rx();
    CHECK(strcmp(c->lat, "3954.5") == 0);
}

TEST(test_rmc_void_status_skipped)
{
    const GPSCoordinates_t *c = gps_get_current_coordinates();
    char lat_before[12];
    strcpy(lat_before, c->lat);

    feed_and_poll("$GPRMC,123519,V,9999.99999,N,19999.99999,W,,,230394,,*6A\r\n");
    CHECK(strcmp(c->lat, lat_before) == 0);     /* V = void: ignored */
}

TEST(test_partial_gga_preserves_last_coordinates)
{
    /* Empty lat/lon fields (cold GGA) must NOT clobber the last good
     * coordinates - the absolute-position field extractor keeps empties
     * distinct instead of shifting later fields */
    const GPSCoordinates_t *c = gps_get_current_coordinates();
    char lat_before[12], lon_before[13];
    strcpy(lat_before, c->lat);
    strcpy(lon_before, c->lon);

    feed_and_poll("$GPGGA,123520,,,,,0,00,99.9,,M,,M,,*66\r\n");
    CHECK(strcmp(c->lat, lat_before) == 0);
    CHECK(strcmp(c->lon, lon_before) == 0);
    CHECK(c->fix_quality == 0);                 /* quality DID update */
    CHECK(strcmp(c->satellites, "00") == 0);
}

TEST(test_unknown_sentences_ignored)
{
    nav_updates = 0;
    feed_and_poll("$GPGSV,3,1,11,03,03,111,00,04,15,270,00*74\r\n");
    feed_and_poll("garbage without a dollar\r\n");
    CHECK(nav_updates == 0);
}

TEST(test_health_verdicts)
{
    /* Fresh sentence just parsed: ACQUIRING */
    feed_and_poll(GGA_FIX);
    CHECK(HB_GPS_STATE(gps_get_health()) == HB_GPS_ACQUIRING);

    /* Bytes flowing but nothing parses for > 5 s: NO_NMEA.
     * Keep the byte clock fresh with garbage. */
    now_ms += 6000;
    feed_and_poll("random noise bytes no dollar sign\r\n");
    CHECK(HB_GPS_STATE(gps_get_health()) == HB_GPS_NO_NMEA);

    /* Total UART silence > 3 s: NO_DATA */
    now_ms += 4000;
    (void)gps_poll_rx();                        /* empty queue */
    CHECK(HB_GPS_STATE(gps_get_health()) == HB_GPS_NO_DATA);

    /* A good sentence restores ACQUIRING */
    feed_and_poll(GGA_FIX);
    CHECK(HB_GPS_STATE(gps_get_health()) == HB_GPS_ACQUIRING);
}

TEST(test_watchdog_silent_uart_recovery)
{
    /* Re-baseline the watchdog with live data */
    feed_and_poll(GGA_FIX);
    gps_recovery_attempts = 0;
    reset_uart_capture();

    /* 31 s of UART silence: watchdog issues a reset + re-init */
    now_ms += 31000;
    (void)gps_poll_rx();

    CHECK(gps_recovery_attempts == 1);
    CHECK(strstr(uart_tx_log, "$PUBX,00") != NULL);        /* poll request */
    CHECK(strstr(uart_tx_log, "$PUBX,40,GGA,1,1") != NULL);/* gps_init ran */
    CHECK(HB_GPS_RESETS(gps_get_health()) == 1);

    /* Data resumes with a fix: attempt counter clears */
    feed_and_poll(GGA_FIX);
    (void)gps_poll_rx();
    CHECK(gps_recovery_attempts == 0);
}

TEST(test_watchdog_no_fix_cold_restart)
{
    /* Bytes flowing (garbage counts) but coordinates never valid for
     * > 60 s: cold restart. Force the no-valid state directly. */
    feed_and_poll(GGA_FIX);
    gps_recovery_attempts = 0;
    current_coords.valid = 0;

    /* The activity counter increments once per data-bearing poll, so the
     * ">100" gate needs ~100 polls - run the real 1 Hz NMEA cadence until
     * both gates (activity > 100 polls, no fix > 60 s) open. */
    reset_uart_capture();
    int fired_at = -1;
    for (int i = 0; i < 200 && fired_at < 0; i++) {
        now_ms += 1000;
        feed_and_poll("$GPGSV,3,1,11,03,03,111,00*74\r\n"); /* parses as nothing */
        if (gps_recovery_attempts >= 1) fired_at = i;
    }

    CHECK(fired_at >= 60);                    /* not before the 60 s gate */
    CHECK(gps_recovery_attempts == 1);
    CHECK(strstr(uart_tx_log, "$PUBX,04,0,0,2,0,0") != NULL); /* cold start */
}

TEST(test_hotstart_stale_position_unlatches)
{
    /* The 2026-08-30 field failure: hot start emits the CACHED position
     * (RMC status A / GGA with lat+lon) but fix quality 0 and 00 sats.
     * The old code latched valid=1 forever, which both hid the loss and
     * disarmed the no-fix watchdog (its timer refreshed on 'valid'). */
    feed_and_poll("$GNRMC,123519,A,3953.00000,N,10453.00000,W,0.0,0.0,230394,,*6A\r\n");
    CHECK(gps_get_current_coordinates()->valid == 1);   /* position latched */

    feed_and_poll("$GPGGA,123520,3953.00000,N,10453.00000,W,0,00,99.9,,M,,M,,*6A\r\n");
    const GPSCoordinates_t *c = gps_get_current_coordinates();
    CHECK(c->valid == 0);                               /* unlatched: no fix */
    CHECK(c->fix_quality == 0);
    CHECK(c->lat[0] != '\0');                           /* position kept, just not valid */

    /* RMC status V also unlatches */
    feed_and_poll(RMC_FIX);
    CHECK(c->valid == 1);
    feed_and_poll("$GPRMC,123519,V,3953.00000,N,10453.00000,W,,,230394,,*6A\r\n");
    CHECK(c->valid == 0);

    /* Restore a good fix for the next test */
    feed_and_poll(GGA_FIX);
    CHECK(c->valid == 1);
}

TEST(test_watchdog_zero_sats_cold_restart)
{
    /* Field failure, isolated: sentences flowing and the module even
     * CLAIMS fix quality 1, but ZERO satellites tracked - a wedged
     * acquisition engine. The good-fix gate requires sats > 0, so this
     * stream must NOT disarm the watchdog: cold restart after 60 s. */
    feed_and_poll(GGA_FIX);
    gps_recovery_attempts = 0;

    reset_uart_capture();
    int fired_at = -1;
    for (int i = 0; i < 150 && fired_at < 0; i++) {
        now_ms += 1000;
        feed_and_poll("$GPGGA,123519,3953.40284,N,10453.11007,W,1,00,0.9,1655.4,M,46.9,M,,*47\r\n");
        if (gps_recovery_attempts >= 1) fired_at = i;
    }

    CHECK(fired_at >= 60);                     /* no-good-fix gate */
    CHECK(gps_recovery_attempts == 1);         /* ...and it STAYS fired */
    CHECK(strstr(uart_tx_log, "$PUBX,04,0,0,2,0,0") != NULL);  /* cold start */
    CHECK(HB_GPS_RESETS(gps_get_health()) == 1);

    /* Satellites tracked again: watchdog disarms */
    feed_and_poll(GGA_FIX);
    CHECK(gps_recovery_attempts == 0);
}

TEST(test_watchdog_never_factory_resets)
{
    /* The silent-UART recovery used to send $PUBX,04,0,0,0,0,0 (factory
     * defaults), which reverts UART1 to 9600 baud - we talk at 115200, so
     * 'recovery' would permanently deafen the link. It must cold-start. */
    feed_and_poll(GGA_FIX);
    gps_recovery_attempts = 0;
    reset_uart_capture();

    now_ms += 31000;
    (void)gps_poll_rx();
    CHECK(gps_recovery_attempts == 1);
    CHECK(strstr(uart_tx_log, "$PUBX,04,0,0,0,0,0*10") == NULL);
    CHECK(strstr(uart_tx_log, "$PUBX,04,0,0,2,0,0*12") != NULL);

    /* Restore live data for the cooldown test */
    feed_and_poll(GGA_FIX);
    CHECK(gps_recovery_attempts == 0);
}

TEST(test_watchdog_three_strikes_then_cooldown_rearm)
{
    /* Burn through the remaining attempts with more silence */
    while (gps_recovery_attempts < 3) {
        now_ms += 31000;
        (void)gps_poll_rx();
    }
    CHECK(gps_recovery_attempts == 3);

    /* Attempts exhausted: more silence must NOT trigger further resets */
    reset_uart_capture();
    now_ms += 31000;
    (void)gps_poll_rx();
    CHECK(gps_recovery_attempts == 3);
    CHECK(strstr(uart_tx_log, "$PUBX") == NULL);

    /* Health caps the reset count at 15 (4-bit field) */
    gps_recovery_attempts = 20;
    CHECK(HB_GPS_RESETS(gps_get_health()) == 15);
    gps_recovery_attempts = 3;

    /* Resiliency fix under test: after a 10-minute cooldown the watchdog
     * re-arms instead of demanding a power cycle */
    now_ms += 601000;
    (void)gps_poll_rx();                       /* cooldown expires, re-arms */
    CHECK(gps_recovery_attempts == 0);

    /* And it actually recovers again on the next silence window */
    reset_uart_capture();
    now_ms += 31000;
    (void)gps_poll_rx();
    CHECK(gps_recovery_attempts == 1);
    CHECK(strstr(uart_tx_log, "$PUBX,00") != NULL);

    /* Clean up: restore live data (a real fix clears the attempt count) */
    feed_and_poll(GGA_FIX);
    (void)gps_poll_rx();
    CHECK(gps_recovery_attempts == 0);
}

TEST(test_oversize_sentence_does_not_overflow)
{
    /* A run-on "sentence" longer than the 84-byte NMEA buffer must be
     * truncated safely, and a following real sentence must still parse */
    char runon[200];
    runon[0] = '$';
    memset(&runon[1], 'A', 150);
    memcpy(&runon[151], "\r\n", 3);
    feed_and_poll(runon);

    nav_updates = 0;
    feed_and_poll(GGA_FIX);
    CHECK(nav_updates == 1);
}

TEST(test_mid_sentence_silence_abandons_parse)
{
    /* The GPS watchdog's cold-start reboots the module, which goes silent
     * mid-sentence. The sentence reader must abandon the partial sentence
     * (~50 ms) instead of spinning forever - the old loop never terminated,
     * which the WDT would have answered by rebooting the whole beacon. */
    feed("$GPGGA,123519,3953.40284,N");   /* no \r\n: silence mid-sentence */
    (void)gps_poll_rx();                  /* must return, not hang */

    /* The next '$' resyncs and a real sentence parses normally */
    nav_updates = 0;
    feed_and_poll(GGA_FIX);
    CHECK(nav_updates == 1);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_sends_ublox_config();
    run_test_gga_full_fix_parsed_and_fed_to_nav();
    run_test_burst_drains_all_sentences_in_one_poll();
    run_test_rmc_updates_position_only();
    run_test_framing_error_bytes_skipped_pre_and_mid_sentence();
    run_test_rmc_void_status_skipped();
    run_test_partial_gga_preserves_last_coordinates();
    run_test_unknown_sentences_ignored();
    run_test_health_verdicts();
    run_test_watchdog_silent_uart_recovery();
    run_test_watchdog_no_fix_cold_restart();
    run_test_hotstart_stale_position_unlatches();
    run_test_watchdog_zero_sats_cold_restart();
    run_test_watchdog_never_factory_resets();
    run_test_watchdog_three_strikes_then_cooldown_rearm();
    run_test_oversize_sentence_does_not_overflow();
    run_test_mid_sentence_silence_abandons_parse();

    return TEST_SUMMARY();
}
