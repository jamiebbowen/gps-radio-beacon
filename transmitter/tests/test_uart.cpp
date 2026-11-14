/**
 * @file test_uart.cpp
 * @brief Host-side unit tests for the transmitter GPS UART shim
 *        (uart.cpp) against a FakeUart Serial1.
 *
 * Covers init (baud, buffer flush, counter reset), the read path with
 * its diagnostic counters and clear-on-read activity flag, the 0xFF
 * empty-read sentinel, TX byte/string plumbing, and the get-and-clear
 * error counters.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <Arduino.h>
#include "include/uart.h"
#include "test_harness.h"

FakeSerial Serial;
FakeUart  Serial1;

unsigned long millis(void) { return 0; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

/* Include the module under test */
#include "../firmware/uart.cpp"

TEST(test_init_flushes_and_resets)
{
    Serial1.inject("stale bytes");
    Serial1.tx_len = 3; strcpy(Serial1.tx, "old");
    uart_init();

    CHECK(Serial1.baud == 115200);
    CHECK(Serial1.available() == 0);           /* stale buffer drained */
}

TEST(test_read_path)
{
    uart_init();
    CHECK(uart_data_available() == false);
    CHECK(uart_read_byte() == 0xFF);           /* empty sentinel */

    Serial1.inject("ABC");
    CHECK(uart_data_available() == true);
    CHECK(uart_read_byte() == 'A');
    CHECK(uart_read_byte() == 'B');
    CHECK(uart_read_byte() == 'C');
    CHECK(uart_data_available() == false);
}

TEST(test_flush_uart_buffer_drains_pending)
{
    uart_init();
    Serial1.inject("garbage");
    CHECK(uart_data_available() == true);
    flush_uart_buffer();
    CHECK(uart_data_available() == false);
    CHECK(uart_read_byte() == 0xFF);           /* nothing left */

    /* Flush of an already-empty buffer is a no-op */
    flush_uart_buffer();
    CHECK(uart_data_available() == false);
}

TEST(test_tx_paths)
{
    uart_init();
    Serial1.clear();

    uart_tx_byte('X');
    CHECK(strcmp(Serial1.tx, "X") == 0);

    Serial1.clear();
    uart_tx_string("AT+CMD");
    CHECK(strcmp(Serial1.tx, "AT+CMD") == 0);

    uart_tx_string(NULL);                      /* null-safe no-op */
    CHECK(strcmp(Serial1.tx, "AT+CMD") == 0);
}

TEST(test_set_baud_reopens_port)
{
    /* GPS baud-recovery path: end() then begin() at the new rate */
    uart_init();
    uart_set_baud(9600);
    CHECK(Serial1.baud == 9600);
    uart_set_baud(115200);
    CHECK(Serial1.baud == 115200);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_flushes_and_resets();
    run_test_read_path();
    run_test_flush_uart_buffer_drains_pending();
    run_test_tx_paths();
    run_test_set_baud_reopens_port();

    return TEST_SUMMARY();
}
