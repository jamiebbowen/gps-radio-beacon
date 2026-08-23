/**
 * @file Arduino.h (HOST TEST STUB)
 * @brief Minimal Arduino core stand-in so transmitter modules compile on a
 *        host PC. Serial output is captured into a buffer that tests can
 *        inspect; millis() is test-settable.
 *
 * Only the surface actually used by the modules under test is provided.
 * Do NOT add hardware behavior here - fakes live in the test executables.
 */

#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* F() PROGMEM helper: plain strings on the host */
#define F(s) (s)

#define DEC 10
#define HEX 16

/* Arduino cores define min/max as macros */
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

/* Timing: fakes defined in the test executable */
unsigned long millis(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

/* GPIO + SPI: fakes defined in the test executable */
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
void pinMode(int pin, int mode);
int  digitalRead(int pin);
void digitalWrite(int pin, int value);

class SPIClass {
public:
    void begin() {}
};
extern SPIClass SPI;

/* ------------------------------------------------------------------ */
/* Capturing Serial fake                                               */
/* ------------------------------------------------------------------ */

class FakeSerial {
public:
    /* Capture buffer, inspectable from tests (serial_log/serial_clear) */
    char   log[16384];
    size_t log_len = 0;

    void clear() { log_len = 0; log[0] = '\0'; }

    void begin(unsigned long baud) { (void)baud; }

    void append(const char *s) {
        size_t n = strlen(s);
        if (log_len + n >= sizeof(log)) { log_len = 0; }  /* wrap: tests clear often */
        memcpy(&log[log_len], s, n);
        log_len += n;
        log[log_len] = '\0';
    }

    void print(const char *s)       { append(s); }
    void print(char c)              { char b[2] = {c, 0}; append(b); }
    void print(int v)               { char b[16]; snprintf(b, sizeof(b), "%d", v); append(b); }
    void print(unsigned int v)      { char b[16]; snprintf(b, sizeof(b), "%u", v); append(b); }
    void print(long v)              { char b[24]; snprintf(b, sizeof(b), "%ld", v); append(b); }
    void print(unsigned long v)     { char b[24]; snprintf(b, sizeof(b), "%lu", v); append(b); }
    void print(double v)            { char b[32]; snprintf(b, sizeof(b), "%.2f", v); append(b); }
    void print(double v, int digits){ char b[48]; snprintf(b, sizeof(b), "%.*f", digits, v); append(b); }
    void print(int v, int base) {
        char b[24];
        snprintf(b, sizeof(b), (base == HEX) ? "%X" : "%d", v);
        append(b);
    }
    void print(unsigned int v, int base) {
        char b[24];
        snprintf(b, sizeof(b), (base == HEX) ? "%X" : "%u", v);
        append(b);
    }

    void write(const uint8_t *buf, size_t n) {
        char tmp[128];
        if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
        memcpy(tmp, buf, n);
        tmp[n] = '\0';
        append(tmp);
    }

    void println()                    { append("\n"); }
    template <typename T> void println(T v)        { print(v); append("\n"); }
    template <typename T> void println(T v, int m) { print(v, m); append("\n"); }
};

extern FakeSerial Serial;

/* ------------------------------------------------------------------ */
/* Hardware-UART fake (Serial1, GPS) with an RX queue and TX capture   */
/* ------------------------------------------------------------------ */

class FakeUart {
public:
    uint8_t  rx[512];
    size_t   rx_head = 0, rx_tail = 0;
    char     tx[1024];
    size_t   tx_len = 0;
    unsigned long baud = 0;

    void begin(unsigned long b) { baud = b; }
    int  available() { return (int)(rx_tail - rx_head); }
    int  read() {
        if (rx_head == rx_tail) return -1;
        return rx[rx_head++ % sizeof(rx)];
    }
    size_t write(uint8_t b) {
        if (tx_len < sizeof(tx) - 1) { tx[tx_len++] = (char)b; tx[tx_len] = '\0'; }
        return 1;
    }
    void print(const char *s) { while (*s) write((uint8_t)*s++); }

    /* Test helpers */
    void inject(const char *s) { while (*s) rx[rx_tail++ % sizeof(rx)] = (uint8_t)*s++; }
    void clear() { rx_head = rx_tail = 0; tx_len = 0; tx[0] = '\0'; }
};

extern FakeUart Serial1;

#endif /* ARDUINO_STUB_H */
