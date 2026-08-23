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

/* ------------------------------------------------------------------ */
/* Capturing Serial fake                                               */
/* ------------------------------------------------------------------ */

class FakeSerial {
public:
    /* Capture buffer, inspectable from tests (serial_log/serial_clear) */
    char   log[16384];
    size_t log_len = 0;

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

#endif /* ARDUINO_STUB_H */
