/**
 * test_altimeter.cpp - commercial altimeter serial decoders (altimeter.cpp)
 * Sample records follow each maker's published telemetry format.
 */
#include <stdio.h>
#include <string.h>
#include "Arduino.h"
#include "test_harness.h"

FakeSerial Serial;
void pinMode(int pin, int mode) { (void)pin; (void)mode; }

static uint32_t flog_calls = 0;
static uint8_t  flog_input = 0xFF, flog_status = 0;
static int32_t  flog_alt = 0, flog_apo = 0;
void flight_log_altimeter(uint32_t ms, uint8_t input, uint8_t status,
                          int32_t alt_ft, int32_t apogee_ft)
{
    (void)ms;
    flog_calls++;
    flog_input = input;
    flog_status = status;
    flog_alt = alt_ft;
    flog_apo = apogee_ft;
}

#include "../firmware/altimeter.cpp"
#include "../firmware/altimeter_input.cpp"

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


/* ---- software UART ---------------------------------------------------- */

/* Line samples for `n` bytes (8N1) at ALT_RX_OVERSAMPLE samples per nominal
 * bit; the sender's bit time is stretched by `scale` (its clock error) and
 * the first frame starts `offset` samples late. Returns the bytes decoded. */
static int rx_run(const uint8_t *bytes, int n, double scale, double offset,
                  uint8_t *out)
{
    alt_rx_t rx;
    memset(&rx, 0, sizeof(rx));
    const double bit = ALT_RX_OVERSAMPLE * scale;
    const double start = 10.0 + offset;
    const double frames = n * 10 * bit;
    int got = 0;
    for (int smp = 0; smp < (int)(start + frames) + 20; smp++) {
        double t = smp - start;
        uint8_t level = 1;
        if (t >= 0 && t < frames) {
            int idx = (int)(t / bit);
            uint16_t frame = (uint16_t)((bytes[idx / 10] << 1) | 0x200);
            level = (uint8_t)((frame >> (idx % 10)) & 1u);
        }
        int16_t r = alt_rx_sample(&rx, level);
        if (r >= 0) out[got++] = (uint8_t)r;
    }
    return got;
}

TEST(test_rx_every_byte_any_phase)
{
    uint8_t all[256], out[300];
    for (int i = 0; i < 256; i++) all[i] = (uint8_t)i;
    for (int ph = 0; ph < 4; ph++) {
        int got = rx_run(all, 256, 1.0, ph * 0.25, out);
        CHECK(got == 256 && memcmp(out, all, 256) == 0);
    }
}

TEST(test_rx_sender_clock_error)
{
    const char *line = "0018.0,+02510,+0001,+070,Dma\r";
    uint8_t out[64];
    int n = (int)strlen(line);
    const double scales[] = { 0.98, 1.02 };
    for (double sc : scales) {
        for (int ph = 0; ph < 4; ph++) {
            int got = rx_run((const uint8_t *)line, n, sc, ph * 0.25, out);
            CHECK(got == n && memcmp(out, line, n) == 0);
        }
    }
}

TEST(test_rx_unplugged_glitch_and_break)
{
    alt_rx_t rx;
    memset(&rx, 0, sizeof(rx));
    int bytes = 0;
    for (int i = 0; i < 4000; i++) bytes += alt_rx_sample(&rx, 0) >= 0;   /* pulled low */
    CHECK(bytes == 0);

    /* Idle high with a one-sample dip: not a start bit */
    for (int i = 0; i < 20; i++) bytes += alt_rx_sample(&rx, 1) >= 0;
    bytes += alt_rx_sample(&rx, 0) >= 0;
    for (int i = 0; i < 60; i++) bytes += alt_rx_sample(&rx, 1) >= 0;
    CHECK(bytes == 0);

    /* A break (12 bit times low): no byte, then a clean byte decodes */
    for (unsigned i = 0; i < 12 * ALT_RX_OVERSAMPLE; i++) bytes += alt_rx_sample(&rx, 0) >= 0;
    CHECK(bytes == 0);
    for (int i = 0; i < 8; i++) (void)alt_rx_sample(&rx, 1);
    uint16_t frame = (uint16_t)(('7' << 1) | 0x200);
    int16_t last = -1;
    for (int b = 0; b < 10; b++)
        for (unsigned k = 0; k < ALT_RX_OVERSAMPLE; k++) {
            int16_t r = alt_rx_sample(&rx, (uint8_t)((frame >> b) & 1u));
            if (r >= 0) last = r;
        }
    CHECK(last == '7');
}

TEST(test_ring_order_and_overflow)
{
    alt_ring_t r;
    memset(&r, 0, sizeof(r));
    CHECK(alt_ring_get(&r) == -1);
    for (int i = 0; i < (int)ALT_RING_SIZE; i++) alt_ring_put(&r, (uint8_t)i);
    CHECK(r.dropped == 0);
    alt_ring_put(&r, 0xEE);                      /* full: dropped */
    CHECK(r.dropped == 1);
    for (int i = 0; i < (int)ALT_RING_SIZE; i++) CHECK(alt_ring_get(&r) == i);
    CHECK(alt_ring_get(&r) == -1);
    for (int i = 0; i < 300; i++) {             /* indices wrap cleanly */
        alt_ring_put(&r, (uint8_t)i);
        CHECK(alt_ring_get(&r) == (i & 0xFF));
    }
}

TEST(test_status_encoding)
{
    alt_state_t s; alt_init(&s);
    CHECK(alt_status(&s, 0) == 0);                       /* nothing recognised */
    feed(&s, "0012.5,+01200,+0410,+072,dma\r0013.0,+01300,+0300,+071,dma\r", 100);
    CHECK(alt_status(&s, 100) == 0);                     /* not locked yet */
    feed(&s, "0018.0,+02510,+0001,+070,Dma\r", 200);
    CHECK(alt_status(&s, 200) == (ALT_ST_FMT_RRC3 | ALT_ST_DROGUE));
    CHECK(alt_status(&s, 200 + ALT_STALE_MS) == (ALT_ST_FMT_RRC3 | ALT_ST_DROGUE));
    CHECK(alt_status(&s, 201 + ALT_STALE_MS) == (ALT_ST_FMT_RRC3 | ALT_ST_DROGUE | ALT_ST_STALE));
}

/* ---- the three inputs (altimeter_input.cpp) --------------------------- */

static uint32_t s_idle = 0;              /* jacks with an altimeter idling high */

static void inject(uint8_t pa_bit, const char *text)
{
    s_idle |= 1u << pa_bit;
    for (int k = 0; k < 8; k++) altimeter_sample(s_idle);
    for (const char *p = text; *p; p++) {
        uint16_t frame = (uint16_t)(((uint8_t)*p << 1) | 0x200);
        for (int b = 0; b < 10; b++)
            for (unsigned k = 0; k < ALT_RX_OVERSAMPLE; k++)
                altimeter_sample((s_idle & ~(1u << pa_bit)) |
                                 ((uint32_t)((frame >> b) & 1u) << pa_bit));
    }
    for (int k = 0; k < 8; k++) altimeter_sample(s_idle);
}

TEST(test_inputs_detect_relay_and_log)
{
    altimeter_init();
    s_idle = 0;
    flog_calls = 0;
    AltimeterPacket_t p;
    for (int i = 0; i < 2000; i++) altimeter_sample(0);   /* all unplugged */
    altimeter_poll(1000);
    CHECK(altimeter_build_packet(&p, 1000) == 0);
    altimeter_log_service(1000);
    CHECK(flog_calls == 0);

    inject(ALT_J2_PA_BIT, "1180\r\n1190\r\n1200\r\n");                 /* StratoLogger */
    inject(ALT_J5_PA_BIT, "0012.5,+01200,+0410,+072,dma\r");
    inject(ALT_J5_PA_BIT, "0013.0,+01380,+0300,+071,dma\r");
    inject(ALT_J5_PA_BIT, "0013.5,+01400,+0290,+071,dma\r");           /* RRC3 */
    inject(ALT_J6_PA_BIT, "#0012><01300>@3>");                         /* Eggtimer */
    altimeter_poll(2000);
    CHECK(altimeter_new_events() == 0);
    CHECK(altimeter_build_packet(&p, 2000) == ALTIMETER_PACKET_SIZE(3));
    CHECK(p.packet_type == PACKET_TYPE_ALTIMETER && p.inputs == 0x07);
    CHECK(p.entry[0].status == ALT_ST_FMT_STRATOLOGGER && p.entry[0].alt_4ft == 300);
    CHECK(p.entry[1].status == ALT_ST_FMT_RRC3 && p.entry[1].alt_4ft == 350);
    CHECK(p.entry[2].status == ALT_ST_FMT_EGGTIMER && p.entry[2].alt_4ft == 325);

    altimeter_log_service(2000);                                       /* one per input */
    CHECK(flog_calls == 3);
    altimeter_log_service(2100);
    CHECK(flog_calls == 3);                                            /* nothing new */

    /* The RRC3 fires its drogue: flagged for an immediate relay, logged at once */
    inject(ALT_J5_PA_BIT, "0018.0,+02510,+0001,+070,Dma\r");
    altimeter_poll(2200);
    CHECK(altimeter_new_events() == 1);
    altimeter_log_service(2200);
    CHECK(flog_calls == 4 && flog_input == 1);
    CHECK(flog_status == (ALT_ST_FMT_RRC3 | ALT_ST_DROGUE));
    CHECK(flog_alt == 2510 && flog_apo == 2510);
    CHECK(altimeter_build_packet(&p, 2200) == ALTIMETER_PACKET_SIZE(3));
    CHECK(altimeter_new_events() == 0);                                /* relayed */
    CHECK((p.entry[1].status & ALT_ST_DROGUE) && p.entry[1].apogee_4ft == 628);   /* 2510 / 4, rounded */

    /* New data logs again only after ALT_LOG_INTERVAL_MS */
    inject(ALT_J2_PA_BIT, "1210\r\n");
    altimeter_poll(2300);
    altimeter_log_service(2300);
    CHECK(flog_calls == 4);
    altimeter_log_service(2000 + ALT_LOG_INTERVAL_MS);
    CHECK(flog_calls == 5 && flog_input == 0 && flog_alt == 1210);

    /* Silence: entries go stale but stay on the air */
    CHECK(altimeter_build_packet(&p, 2300 + ALT_STALE_MS + 1) == ALTIMETER_PACKET_SIZE(3));
    CHECK(p.entry[0].status & ALT_ST_STALE);
}

TEST(test_inputs_ring_overflow_recovers)
{
    altimeter_init();
    s_idle = 0;
    char burst[260];
    memset(burst, '9', sizeof(burst) - 1);
    burst[sizeof(burst) - 1] = 0;
    inject(ALT_J5_PA_BIT, burst);                 /* 259 bytes, nobody polling */
    altimeter_poll(10);                           /* overlong line: rejected */
    inject(ALT_J5_PA_BIT, "\r\n100\r\n101\r\n102\r\n");
    altimeter_poll(20);
    AltimeterPacket_t p;
    CHECK(altimeter_build_packet(&p, 20) == ALTIMETER_PACKET_SIZE(1));
    CHECK(p.inputs == 0x02 && p.entry[0].alt_4ft == 26);   /* 102 ft / 4, rounded */
}

int main(void)
{
    run_test_stratologger_lines();
    run_test_rrc3_all_items();
    run_test_eggtimer_elements();
    run_test_noise_overflow_and_format_lock();
    run_test_rx_every_byte_any_phase();
    run_test_rx_sender_clock_error();
    run_test_rx_unplugged_glitch_and_break();
    run_test_ring_order_and_overflow();
    run_test_status_encoding();
    run_test_inputs_detect_relay_and_log();
    run_test_inputs_ring_overflow_recovers();
    return TEST_SUMMARY();
}
