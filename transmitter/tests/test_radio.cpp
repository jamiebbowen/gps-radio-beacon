/**
 * @file test_radio.cpp
 * @brief Host-side unit tests for the transmitter LoRa radio glue
 *        (radio.cpp) with a stubbed RadioLib.
 *
 * Verifies jumper-based channel resolution (primary vs backup),
 * init success/failure bookkeeping, the radio_enable() re-init retry
 * ladder (3 attempts, then gives up), transmit plumbing + payload
 * capture, TXEN driving, and the uninitialized guards.
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <Arduino.h>
#include <RadioLib.h>
#include "include/radio.h"
#include "include/mpu_config.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / RadioLib fakes                                            */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
SPIClass SPI;

static unsigned long now_ms = 0;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }

/* GPIO: jumper pin + BUSY pin are scriptable */
static int jumper_level  = HIGH;   /* open = primary */
static int busy_level    = LOW;
static int txen_level    = -1;

void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin)
{
    if (pin == CHANNEL_JUMPER_PIN) return jumper_level;
    if (pin == LORA_BUSY) return busy_level;
    return LOW;
}
void digitalWrite(int pin, int value)
{
    if (pin == LORA_TXEN) txen_level = value;
}

/* RadioLib knobs */
int     radiolib_begin_result = RADIOLIB_ERR_NONE;
float   radiolib_begin_freq = 0;
int     radiolib_begin_calls = 0;
int     radiolib_standby_calls = 0;
int     radiolib_transmit_result = RADIOLIB_ERR_NONE;
int     radiolib_transmit_calls = 0;
uint8_t radiolib_last_tx[256];
size_t  radiolib_last_tx_len = 0;

/* Include the module under test */
#include "../firmware/radio.cpp"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void reset_knobs(void)
{
    radiolib_begin_result = RADIOLIB_ERR_NONE;
    radiolib_begin_calls = 0;
    radiolib_standby_calls = 0;
    radiolib_transmit_result = RADIOLIB_ERR_NONE;
    radiolib_transmit_calls = 0;
    radiolib_last_tx_len = 0;
    jumper_level = HIGH;
    busy_level = LOW;
    txen_level = -1;
    Serial.clear();
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_primary_channel)
{
    reset_knobs();
    radio_init();

    CHECK(radio_initialized == true);
    CHECK(radio_get_channel() == LORA_CHANNEL);
    CHECK(radiolib_begin_calls == 1);
    CHECK(radiolib_begin_freq == LORA_CHANNEL_FREQ(LORA_CHANNEL));
    CHECK(radiolib_standby_calls == 1);        /* parked in standby */
    CHECK(radio_enabled == false);
    CHECK(strstr(Serial.log, "Initialized successfully") != NULL);
}

TEST(test_init_backup_jumper)
{
    reset_knobs();
    jumper_level = LOW;                        /* shorted to GND */
    radio_init();

    CHECK(radio_get_channel() == LORA_CHANNEL_BACKUP);
    CHECK(radiolib_begin_freq == LORA_CHANNEL_FREQ(LORA_CHANNEL_BACKUP));
    CHECK(strstr(Serial.log, "backup jumper") != NULL);
}

TEST(test_init_failure_and_transmit_guard)
{
    reset_knobs();
    radiolib_begin_result = RADIOLIB_ERR_CHIP_NOT_FOUND;
    radio_init();

    CHECK(radio_initialized == false);
    CHECK(strstr(Serial.log, "CHIP_NOT_FOUND") != NULL);

    /* Transmit with a dead radio is refused */
    const uint8_t pkt[3] = {1, 2, 3};
    CHECK(transmit_packet(pkt, 3) == RADIOLIB_ERR_CHIP_NOT_FOUND);
    CHECK(radiolib_transmit_calls == 0);
    CHECK(radio_is_transmitting() == false);

    /* Disable on a dead radio is a no-op (no standby call) */
    int before = radiolib_standby_calls;
    radio_disable();
    CHECK(radiolib_standby_calls == before);
}

TEST(test_enable_retries_then_gives_up)
{
    /* init already failed (previous test): radio_initialized == false */
    reset_knobs();
    radiolib_begin_result = RADIOLIB_ERR_CHIP_NOT_FOUND;
    radio_initialized = false;
    init_retry_count = 0;

    radio_enable();                            /* retry 1: fails */
    CHECK(init_retry_count == 1);
    radio_enable();                            /* retry 2 */
    radio_enable();                            /* retry 3 */
    CHECK(init_retry_count == 3);
    int calls = radiolib_begin_calls;
    radio_enable();                            /* ladder exhausted */
    CHECK(radiolib_begin_calls == calls);      /* no 4th attempt */
    CHECK(strstr(Serial.log, "Max retry attempts") != NULL);
    CHECK(radio_enabled == false);
}

TEST(test_enable_recovers_on_retry)
{
    reset_knobs();
    radio_initialized = false;
    init_retry_count = 2;                      /* last chance */
    radiolib_begin_result = RADIOLIB_ERR_NONE; /* radio comes back */

    radio_enable();
    CHECK(radio_initialized == true);
    CHECK(radio_enabled == true);
    CHECK(strstr(Serial.log, "Reinitialize succeeded") != NULL);
    CHECK(txen_level == HIGH);                 /* TXEN driven */
}

TEST(test_transmit_paths)
{
    /* Radio healthy + enabled from the previous test */
    const uint8_t pkt[5] = {0xAA, 1, 2, 3, 4};
    CHECK(transmit_packet(pkt, 5) == RADIOLIB_ERR_NONE);
    CHECK(radiolib_transmit_calls == 1);
    CHECK(radiolib_last_tx_len == 5);
    CHECK(memcmp(radiolib_last_tx, pkt, 5) == 0);

    CHECK(transmit_string("beacon") == RADIOLIB_ERR_NONE);
    CHECK(radiolib_last_tx_len == 6);
    CHECK(memcmp(radiolib_last_tx, "beacon", 6) == 0);
    CHECK(transmit_string(NULL) == RADIOLIB_ERR_INVALID_ENCODING);

    /* RadioLib-level TX failure surfaces */
    radiolib_transmit_result = -4;             /* TX timeout */
    CHECK(transmit_packet(pkt, 5) == -4);
    CHECK(strstr(Serial.log, "Transmission failed") != NULL);
    radiolib_transmit_result = RADIOLIB_ERR_NONE;

    /* Busy pin mirrors TX activity */
    busy_level = HIGH;
    CHECK(radio_is_transmitting() == true);
    busy_level = LOW;
    CHECK(radio_is_transmitting() == false);
}

TEST(test_disable)
{
    int before = radiolib_standby_calls;
    radio_disable();
    CHECK(radio_enabled == false);
    CHECK(radiolib_standby_calls == before + 1);
    CHECK(txen_level == LOW);                  /* TXEN released */

    /* Transmitting while disabled still works but warns */
    Serial.clear();
    const uint8_t pkt[1] = {7};
    CHECK(transmit_packet(pkt, 1) == RADIOLIB_ERR_NONE);
    CHECK(strstr(Serial.log, "while disabled") != NULL);
}

/* ------------------------------------------------------------------ */

TEST(test_tx_failure_streak_forces_reinit)
{
    reset_knobs();
    radio_init();                            /* healthy */
    radio_enabled = true;
    const uint8_t pkt[1] = {0x42};

    /* 7 consecutive TX failures: tolerated (one bad packet is noise) */
    radiolib_transmit_result = -4;
    for (int i = 0; i < 7; i++) transmit_packet(pkt, 1);
    CHECK(radiolib_begin_calls == 1);

    /* 8th consecutive: wedged chip gets a full NRST + begin() */
    transmit_packet(pkt, 1);
    CHECK(radiolib_begin_calls == 2);
    CHECK(radio_initialized == true);
    CHECK(tx_fail_streak == 0);

    /* A success mid-streak resets the counter */
    radiolib_transmit_result = -4;
    for (int i = 0; i < 7; i++) transmit_packet(pkt, 1);
    radiolib_transmit_result = RADIOLIB_ERR_NONE;
    transmit_packet(pkt, 1);
    radiolib_transmit_result = -4;
    for (int i = 0; i < 7; i++) transmit_packet(pkt, 1);
    CHECK(radiolib_begin_calls == 2);        /* still no re-init */
    transmit_packet(pkt, 1);                 /* 8th consecutive */
    CHECK(radiolib_begin_calls == 3);
}

TEST(test_dead_radio_keeps_retrying_reinit)
{
    reset_knobs();
    radiolib_begin_result = RADIOLIB_ERR_CHIP_NOT_FOUND;
    radio_init();                            /* chip absent */
    CHECK(radio_initialized == false);
    const uint8_t pkt[1] = {1};

    /* Every 8 transmit attempts buys one full re-init: a transient
     * radio brownout can self-heal without a beacon reboot. */
    for (int round = 0; round < 3; round++) {
        int calls = radiolib_begin_calls;
        for (int i = 0; i < 8; i++) {
            CHECK(transmit_packet(pkt, 1) == RADIOLIB_ERR_CHIP_NOT_FOUND);
        }
        CHECK(radiolib_begin_calls == calls + 1);
    }

    /* Chip comes back: picked up automatically, ladder re-armed */
    radiolib_begin_result = RADIOLIB_ERR_NONE;
    for (int i = 0; i < 8; i++) transmit_packet(pkt, 1);
    CHECK(radio_initialized == true);
    CHECK(init_retry_count == 0);
    radio_enabled = true;                    /* leave a healthy radio */
    radiolib_transmit_result = RADIOLIB_ERR_NONE;
}

int main(void)
{
    run_test_init_primary_channel();
    run_test_init_backup_jumper();
    run_test_init_failure_and_transmit_guard();
    run_test_enable_retries_then_gives_up();
    run_test_enable_recovers_on_retry();
    run_test_transmit_paths();
    run_test_disable();
    run_test_tx_failure_streak_forces_reinit();
    run_test_dead_radio_keeps_retrying_reinit();

    return TEST_SUMMARY();
}
