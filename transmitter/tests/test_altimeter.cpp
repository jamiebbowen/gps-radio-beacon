/**
 * test_altimeter.cpp - commercial altimeter serial decoders (altimeter.cpp)
 * Sample records follow each maker's published telemetry format.
 */
#include <stdio.h>
#include <string.h>
#include "../firmware/altimeter.cpp"
#include "test_harness.h"

static uint32_t feed(alt_state_t *s, const char *txt, uint32_t now)
{
    uint32_t n = 0;
    for (const char *p = txt; *p; p++) n += alt_feed(s, *p, now);
    return n;
}

TEST(test_stratologger_lines)
{
    alt_state_t s; alt_init(&s);
    CHECK(feed(&s, "198\r\n248\r\n300\r\n349\r\n", 1000) == 4);
    CHECK(s.format == ALT_FMT_STRATOLOGGER);
    CHECK(s.alt_ft == 349 && s.apogee_ft == 349 && s.events == 0);
    CHECK(feed(&s, "320\r\n", 2000) == 1);
    CHECK(s.alt_ft == 320 && s.apogee_ft == 349 && s.last_ms == 2000);
}

TEST(test_rrc3_all_items)
{
    alt_state_t s; alt_init(&s);
    CHECK(feed(&s, "0012.5,+01200,+0410,+072,dma\r", 10) == 1);
    CHECK(feed(&s, "0013.0,+01380,+0300,+071,dma\r", 20) == 1);
    CHECK(feed(&s, "0018.0,+02510,+0001,+070,Dma\r", 30) == 1);
    CHECK(s.format == ALT_FMT_RRC3);
    CHECK(s.alt_ft == 2510 && s.events == ALT_EVT_DROGUE);
    CHECK(feed(&s, "0061.0,+00700,-0055,+072,DMa\r", 40) == 1);
    CHECK(s.events == (ALT_EVT_DROGUE | ALT_EVT_MAIN) && s.apogee_ft == 2510);
    CHECK(feed(&s, "0062.0,+00650,-0054,+072,---\r", 50) == 1);   /* latched */
    CHECK(s.events == (ALT_EVT_DROGUE | ALT_EVT_MAIN));
}

TEST(test_eggtimer_elements)
{
    alt_state_t s; alt_init(&s);
    CHECK(feed(&s, "#0012>@3><01234>(0350>~AB----><02300>", 5) == 6);
    CHECK(s.format == ALT_FMT_EGGTIMER && s.alt_ft == 2300 && s.events == 0);
    CHECK(feed(&s, "~1B----><02450>%02510>", 6) == 3);
    CHECK(s.events == ALT_EVT_DROGUE && s.apogee_ft == 2510);
    CHECK(feed(&s, "~12----><00700>", 7) == 2);
    CHECK(s.events == (ALT_EVT_DROGUE | ALT_EVT_MAIN) && s.alt_ft == 700);
}

TEST(test_noise_overflow_and_format_lock)
{
    alt_state_t s; alt_init(&s);
    CHECK(feed(&s, "12x4\r\n", 1) == 0 && s.errors == 1);          /* garbage */
    char longl[80]; memset(longl, '7', 70); longl[70] = 0;
    CHECK(feed(&s, longl, 2) == 0 && feed(&s, "\r\n", 2) == 0 && s.errors == 2);
    CHECK(feed(&s, "100\r\n101\r\n102\r\n", 3) == 3);
    CHECK(s.format == ALT_FMT_STRATOLOGGER);
    CHECK(feed(&s, "<01234>", 4) == 0 && s.alt_ft == 102);        /* locked */
    CHECK(feed(&s, "\x01\xff" "103\r\n", 5) == 1 && s.alt_ft == 103); /* noise bytes dropped */
}

int main(void)
{
    run_test_stratologger_lines();
    run_test_rrc3_all_items();
    run_test_eggtimer_elements();
    run_test_noise_overflow_and_format_lock();
    return TEST_SUMMARY();
}
