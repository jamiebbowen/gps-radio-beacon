/**
 * @file altimeter_input.cpp
 * @brief Commercial altimeter inputs (see include/altimeter_input.h)
 */
#include <Arduino.h>
#include "include/altimeter_input.h"
#include "include/altimeter.h"
#include "include/config.h"
#include "include/mpu_config.h"
#include "include/fixed_point.h"
#include "include/flight_log.h"
#include <string.h>

static const uint8_t k_pa_bit[ALTIMETER_INPUTS] = { ALT_J2_PA_BIT, ALT_J5_PA_BIT, ALT_J6_PA_BIT };
static const char   *k_jack[ALTIMETER_INPUTS]   = { "J2", "J5", "J6" };

static alt_rx_t    s_rx[ALTIMETER_INPUTS];
static alt_ring_t  s_ring[ALTIMETER_INPUTS];
static alt_state_t s_dec[ALTIMETER_INPUTS];

static uint8_t  s_events_unsent = 0;
static uint8_t  s_logged_events[ALTIMETER_INPUTS];
static uint32_t s_logged_records[ALTIMETER_INPUTS];
static uint32_t s_last_log_ms[ALTIMETER_INPUTS];

#if defined(__SAMD51__)
/* TC3 off the core's 48 MHz generator (GCLK1), match-frequency mode at
 * ALT_RX_OVERSAMPLE x ALT_BAUD = 38.4 kHz. TC3_Handler is a weak alias in
 * the core; TC2 belongs to tone(), which this firmware never uses. */
static void altimeter_timer_start(void)
{
    GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
    while (!(GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN)) {}
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.SYNCBUSY.bit.SWRST) {}
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1;
    TC3->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    TC3->COUNT16.CC[0].reg = (uint16_t)(48000000UL / (ALT_BAUD * ALT_RX_OVERSAMPLE) - 1);
    while (TC3->COUNT16.SYNCBUSY.bit.CC0) {}
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 1);
    NVIC_EnableIRQ(TC3_IRQn);
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.SYNCBUSY.bit.ENABLE) {}
}

extern "C" void TC3_Handler(void)
{
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    altimeter_sample(PORT->Group[0].IN.reg);
}
#endif

void altimeter_init(void)
{
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_ring, 0, sizeof(s_ring));
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) alt_init(&s_dec[i]);
    memset(s_logged_events, 0, sizeof(s_logged_events));
    memset(s_logged_records, 0, sizeof(s_logged_records));
    memset(s_last_log_ms, 0, sizeof(s_last_log_ms));
    s_events_unsent = 0;
    /* Plain inputs: the carrier's 20k pull-downs hold an unplugged jack
     * low (no edges, no bytes); a connected altimeter idles high. */
    pinMode(ALT_J2_PIN, INPUT);
    pinMode(ALT_J5_PIN, INPUT);
    pinMode(ALT_J6_PIN, INPUT);
#if defined(__SAMD51__)
    altimeter_timer_start();
#endif
}

void altimeter_sample(uint32_t port_in)
{
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        int16_t b = alt_rx_sample(&s_rx[i], (uint8_t)((port_in >> k_pa_bit[i]) & 1u));
        if (b >= 0) alt_ring_put(&s_ring[i], (uint8_t)b);
    }
}

static const char *fmt_name(alt_format_t f)
{
    return f == ALT_FMT_STRATOLOGGER ? "StratoLogger CF" :
           f == ALT_FMT_RRC3         ? "RRC3" :
           f == ALT_FMT_EGGTIMER     ? "Eggtimer" : "?";
}

void altimeter_poll(uint32_t now_ms)
{
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        alt_state_t *d = &s_dec[i];
        int16_t c;
        while ((c = alt_ring_get(&s_ring[i])) >= 0) {
            alt_format_t fmt0 = d->format;
            uint8_t ev0 = d->events;
            if (!alt_feed(d, (char)c, now_ms)) continue;
            if (d->format != fmt0) {
                Serial.print(F("[Alt] "));
                Serial.print(k_jack[i]);
                Serial.print(F(": "));
                Serial.println(fmt_name(d->format));
            }
            if (d->events & ~ev0) {
                /* relayed once the format is recognised (only those ride) */
                if (d->format != ALT_FMT_UNKNOWN) s_events_unsent = 1;
                Serial.print(F("[Alt] "));
                Serial.print(k_jack[i]);
                Serial.print(F(" events"));
                if (d->events & ALT_EVT_DROGUE) Serial.print(F(" drogue"));
                if (d->events & ALT_EVT_MAIN)   Serial.print(F(" main"));
                if (d->events & ALT_EVT_AUX)    Serial.print(F(" aux"));
                Serial.println();
            }
        }
    }
}

void altimeter_log_service(uint32_t now_ms)
{
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        const alt_state_t *d = &s_dec[i];
        if (d->format == ALT_FMT_UNKNOWN) continue;
        uint8_t new_event = (d->events != s_logged_events[i]);
        uint8_t new_data  = (d->records != s_logged_records[i]) &&
                            (now_ms - s_last_log_ms[i] >= ALT_LOG_INTERVAL_MS);
        if (!new_event && !new_data) continue;
        flight_log_altimeter(now_ms, i, alt_status(d, now_ms), d->alt_ft, d->apogee_ft);
        s_logged_events[i] = d->events;
        s_logged_records[i] = d->records;
        s_last_log_ms[i] = now_ms;
    }
}

void altimeter_flight_start(uint32_t now_ms)
{
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        alt_flight_start(&s_dec[i]);
        s_logged_events[i] = 0;
        s_logged_records[i] = s_dec[i].records;
        s_last_log_ms[i] = now_ms - ALT_LOG_INTERVAL_MS;   /* next one logs */
    }
    s_events_unsent = 0;
}

uint8_t altimeter_new_events(void)
{
    return s_events_unsent;
}

uint8_t altimeter_fill_maxima(MaximaAltPacket_t *p, uint32_t now_ms)
{
    p->alt_inputs = 0;
    memset(p->alt, 0, sizeof(p->alt));
    uint8_t n = 0;
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        const alt_state_t *d = &s_dec[i];
        if (d->format == ALT_FMT_UNKNOWN) continue;
        p->alt_inputs |= (uint8_t)(1u << i);
        p->alt[n].status = alt_status(d, now_ms);
        p->alt[n].apogee_4ft = sat_i16((float)d->apogee_ft / ALT_FT_PER_UNIT);
        n++;
    }
    if (n == 0) return 0;
    s_events_unsent = 0;
    return (uint8_t)(MAXIMA_ALT_PACKET_SIZE(n) - MAXIMA_PACKET_SIZE);
}
