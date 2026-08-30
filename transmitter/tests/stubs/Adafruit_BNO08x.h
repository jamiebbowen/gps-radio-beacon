/*
 * Minimal Adafruit_BNO08x stub for host-side testing of
 * transmitter/launch_detect.cpp. The test executable scripts sensor
 * events via a queue and controls begin/enable/wasReset outcomes.
 */
#ifndef ADAFRUIT_BNO08X_STUB_H
#define ADAFRUIT_BNO08X_STUB_H

#include <stdint.h>
#include <stdbool.h>

#define SH2_LINEAR_ACCELERATION   0x04
#define SH2_ROTATION_VECTOR       0x05
#define SH2_GAME_ROTATION_VECTOR  0x08

typedef struct {
    uint8_t sensorId;
    union {
        struct { float real, i, j, k; } rotationVector;
        struct { float x, y, z; } linearAcceleration;
    } un;
} sh2_SensorValue_t;

/* Test knobs (defined by the test executable) */
extern bool     bno_begin_result;
extern bool     bno_enable_result;
extern int      bno_enable_fail_sensor; /* sensorId to fail (0 = none) */
extern bool     bno_was_reset;
extern int      bno_enable_calls;
extern int      bno_begin_calls;
extern int      bno_last_report;        /* last enableReport() sensor id */

/* Scripted event queue: getSensorEvent() pops until empty */
extern sh2_SensorValue_t bno_events[32];
extern int               bno_event_head;
extern int               bno_event_tail;

class Adafruit_BNO08x {
public:
    Adafruit_BNO08x(int reset_pin) : reset_pin_(reset_pin) {}

    bool begin_I2C(uint8_t addr)
    {
        (void)addr;
        bno_begin_calls++;
        return bno_begin_result;
    }

    bool enableReport(int sensorId)
    {
        bno_enable_calls++;
        bno_last_report = sensorId;
        if (bno_enable_fail_sensor && sensorId == bno_enable_fail_sensor) return false;
        return bno_enable_result;
    }

    bool wasReset() { return bno_was_reset; }

    bool getSensorEvent(sh2_SensorValue_t *value)
    {
        if (bno_event_head == bno_event_tail) return false;
        *value = bno_events[bno_event_head++ % 32];
        return true;
    }

private:
    int reset_pin_;
};

#endif /* ADAFRUIT_BNO08X_STUB_H */
