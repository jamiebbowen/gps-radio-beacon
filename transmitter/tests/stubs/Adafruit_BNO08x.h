/*
 * Minimal Adafruit_BNO08x stub for host-side testing of
 * transmitter/launch_detect.cpp, modelled on the real library:
 *  - the hub hands over SHTP transfers, and one transfer can carry several
 *    batched input reports (bno_batch per transfer, default 1);
 *  - sh2_service() processes ONE transfer and passes every report in it to
 *    the installed sensor callback;
 *  - begin_I2C() installs the library's own callback, which decodes each
 *    report into a single slot - so getSensorEvent() returns only the LAST
 *    report of a transfer (Adafruit_BNO08x.cpp sensorHandler).
 * The test executable scripts reports via a queue and controls
 * begin/enable/wasReset outcomes.
 */
#ifndef ADAFRUIT_BNO08X_STUB_H
#define ADAFRUIT_BNO08X_STUB_H

#include <stdint.h>
#include <stdbool.h>

#define SH2_LINEAR_ACCELERATION   0x04
#define SH2_ROTATION_VECTOR       0x05
#define SH2_GAME_ROTATION_VECTOR  0x08
#define SH2_GYROSCOPE_CALIBRATED  0x02

typedef struct {
    uint8_t sensorId;
    union {
        struct { float real, i, j, k; } rotationVector;
        struct { float x, y, z; } linearAcceleration;
        struct { float x, y, z; } gyroscope;   /* rad/s, calibrated */
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

/* Scripted report queue, consumed one SHTP transfer at a time */
extern sh2_SensorValue_t bno_events[32];
extern int               bno_event_head;
extern int               bno_event_tail;
extern int               bno_batch;      /* reports per transfer */

/* sh2 API surface the firmware uses (sh2.h / sh2_SensorValue.h). The stub's
 * event carries the decoded value; sh2_decodeSensorEvent copies it out. */
typedef struct { sh2_SensorValue_t v; } sh2_SensorEvent_t;
typedef void (sh2_SensorCallback_t)(void *cookie, sh2_SensorEvent_t *pEvent);
#define SH2_OK 0

static sh2_SensorCallback_t *bno_sensor_cb = 0;   /* 0 = the library's own */
static void *bno_sensor_cookie = 0;
static sh2_SensorValue_t bno_slot;                /* Adafruit's _sensor_value */
static bool bno_slot_fresh = false;

static inline int sh2_setSensorCallback(sh2_SensorCallback_t *cb, void *cookie)
{
    bno_sensor_cb = cb;
    bno_sensor_cookie = cookie;
    return SH2_OK;
}

static inline int sh2_decodeSensorEvent(sh2_SensorValue_t *value,
                                        const sh2_SensorEvent_t *event)
{
    *value = event->v;
    return SH2_OK;
}

/* One transfer: every report in it goes to the callback in order */
static inline void sh2_service(void)
{
    int per = bno_batch > 0 ? bno_batch : 1;
    for (int n = 0; n < per && bno_event_head != bno_event_tail; n++) {
        sh2_SensorEvent_t e;
        e.v = bno_events[bno_event_head++ % 32];
        if (bno_sensor_cb) {
            bno_sensor_cb(bno_sensor_cookie, &e);
        } else {
            bno_slot = e.v;              /* the library's handler: one slot */
            bno_slot_fresh = true;
        }
    }
}

class Adafruit_BNO08x {
public:
    Adafruit_BNO08x(int reset_pin) : reset_pin_(reset_pin) {}

    bool begin_I2C(uint8_t addr)
    {
        (void)addr;
        bno_begin_calls++;
        sh2_setSensorCallback(0, 0);     /* begin installs its own handler */
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

    /* Services one transfer and returns its LAST report (or false). With a
     * caller-installed sensor callback the reports go there instead. */
    bool getSensorEvent(sh2_SensorValue_t *value)
    {
        bno_slot_fresh = false;
        sh2_service();
        if (!bno_slot_fresh) return false;
        *value = bno_slot;
        return true;
    }

private:
    int reset_pin_;
};

#endif /* ADAFRUIT_BNO08X_STUB_H */
