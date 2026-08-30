/**
 * @file test_launch_detect.cpp
 * @brief Host-side unit tests for the BNO085 launch detector
 *        (launch_detect.cpp) with a stubbed Adafruit_BNO08x.
 *
 * Scripts sensor events (linear accel + rotation vector) through the
 * stub's queue to exercise the IDLE -> DETECTING -> CONFIRMED state
 * machine: threshold crossing, the 100 ms sustain requirement, the
 * false-alarm drop-out, the is_launched() one-shot flag, sensor-reset
 * re-arm, event draining, and the nav-layer accessors.
 *
 * Build & run:  make -C transmitter/tests
 * Coverage:     make -C transmitter/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "include/launch_detect.h"
#include "include/mpu_config.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / BNO08x fakes                                              */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
WireClass Wire;

static unsigned long now_ms = 100000;
unsigned long millis(void) { return now_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

/* BNO08x knobs + event queue */
bool bno_begin_result  = true;
bool bno_enable_result = true;
int  bno_enable_fail_sensor = 0;
bool bno_was_reset     = false;
int  bno_enable_calls  = 0;
int  bno_begin_calls   = 0;
int  bno_last_report   = -1;
sh2_SensorValue_t bno_events[32];
int bno_event_head = 0;
int bno_event_tail = 0;

/* Include the module under test */
#include "../firmware/launch_detect.cpp"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void reset_all(void)
{
    bno_begin_result = true;
    bno_enable_result = true;
    bno_enable_fail_sensor = 0;
    bno_was_reset = false;
    bno_enable_calls = 0;
    bno_begin_calls = 0;
    bno_last_report = -1;
    bno_event_head = bno_event_tail = 0;
    Serial.clear();
}

static void push_accel(float x, float y, float z)
{
    sh2_SensorValue_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.sensorId = SH2_LINEAR_ACCELERATION;
    ev.un.linearAcceleration.x = x;
    ev.un.linearAcceleration.y = y;
    ev.un.linearAcceleration.z = z;
    bno_events[bno_event_tail++ % 32] = ev;
}

static void push_rotvec(float w, float i, float j, float k)
{
    sh2_SensorValue_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.sensorId = SH2_ROTATION_VECTOR;
    ev.un.rotationVector.real = w;
    ev.un.rotationVector.i = i;
    ev.un.rotationVector.j = j;
    ev.un.rotationVector.k = k;
    bno_events[bno_event_tail++ % 32] = ev;
}

/** One update tick at the current clock, after advancing it */
static void tick(uint32_t ms, uint32_t sys_s)
{
    now_ms += ms;
    launch_detect_update(sys_s, now_ms);
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_success)
{
    reset_all();
    launch_detect_init();

    CHECK(bno_begin_calls == 1);
    CHECK(launch_detect_get_imu_status() == true);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_IDLE);
    /* Linear accel + full rotation vector (IMU_FUSION_USE_GAME_ROTVEC=0) */
    CHECK(bno_enable_calls == 2);
    CHECK(bno_last_report == SH2_ROTATION_VECTOR);
    CHECK(strstr(Serial.log, "Launch detection ready") != NULL);
}

TEST(test_rotvec_enable_failure_warns_but_init_succeeds)
{
    /* Linear accel is mandatory, but the rotation vector is advisory: its
     * enable failure must warn yet still leave the detector initialized. */
    reset_all();
    bno_enable_fail_sensor = SH2_ROTATION_VECTOR;
    launch_detect_init();
    CHECK(launch_detect_get_imu_status() == true);
    CHECK(strstr(Serial.log, "Could not enable rotation vector") != NULL);

    /* No launch yet: time-since-launch is 0 (not garbage) */
    CHECK(launch_detect_get_time_since_launch(999) == 0);
}

TEST(test_init_failures)
{
    /* Chip not found */
    reset_all();
    bno_begin_result = false;
    launch_detect_init();
    CHECK(launch_detect_get_imu_status() == false);
    CHECK(strstr(Serial.log, "Failed to find") != NULL);

    /* Reports won't enable */
    reset_all();
    bno_enable_result = false;
    launch_detect_init();
    CHECK(launch_detect_get_imu_status() == false);
    CHECK(strstr(Serial.log, "Could not enable linear acceleration") != NULL);

    /* Uninitialized: update is a no-op */
    push_accel(50, 0, 0);
    tick(10, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_IDLE);
    CHECK(launch_detect_is_launched() == false);
}

TEST(test_launch_detection_happy_path)
{
    reset_all();
    launch_detect_init();

    /* Below threshold: nothing happens */
    push_accel(5, 0, 0);
    tick(10, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_IDLE);

    /* Above threshold: enters DETECTING but not yet confirmed */
    push_accel(25, 0, 0);
    tick(10, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_DETECTING);
    CHECK(launch_detect_is_launched() == false);

    /* Sustained past LAUNCH_ACCEL_DURATION (100 ms): CONFIRMED */
    push_accel(25, 0, 0);
    tick(50, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_DETECTING);
    push_accel(25, 0, 0);
    tick(50, 42);                                /* t=110 ms into detection */
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(strstr(Serial.log, "LAUNCH CONFIRMED") != NULL);

    /* is_launched is a one-shot flag */
    CHECK(launch_detect_is_launched() == true);
    CHECK(launch_detect_is_launched() == false);

    /* Launch time latched from system_time_seconds */
    CHECK(launch_detect_get_time_since_launch(42) == 0);
    CHECK(launch_detect_get_time_since_launch(52) == 10);

    /* Confirmed is terminal */
    push_accel(0, 0, 0);
    tick(100, 60);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
}

TEST(test_false_alarm_drops_to_idle)
{
    reset_all();
    launch_detect_init();

    push_accel(30, 0, 0);
    tick(10, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_DETECTING);

    /* Accel drops before the sustain window closes */
    push_accel(5, 0, 0);
    tick(50, 0);
    CHECK(launch_detect_get_state() == LAUNCH_STATE_IDLE);
    CHECK(launch_detect_is_launched() == false);
    CHECK(strstr(Serial.log, "False alarm") != NULL);
}

TEST(test_sensor_event_draining_and_accessors)
{
    reset_all();
    launch_detect_init();

    /* Multiple queued events in one tick: all must be consumed */
    push_accel(1, 2, 3);
    push_rotvec(0.5f, 0.5f, 0.5f, 0.5f);
    push_accel(4, 5, 6);
    tick(10, 0);

    CHECK(bno_event_head == bno_event_tail);   /* fully drained */
    CHECK(launch_detect_get_current_accel() > 8.7f && /* sqrt(16+25+36) */
          launch_detect_get_current_accel() < 8.8f);

    float x = 0, y = 0, z = 0;
    launch_detect_get_accel_xyz(&x, &y, &z);
    CHECK(x == 4 && y == 5 && z == 6);
    launch_detect_get_accel_xyz(NULL, NULL, NULL);   /* null-safe */

    float ax, ay, az;
    imu_get_linear_accel_body(&ax, &ay, &az);
    CHECK(ax == 4 && ay == 5 && az == 6);
    imu_get_linear_accel_body(NULL, NULL, NULL);

    /* Rotation vector captured + timestamped */
    CHECK(imu_has_quaternion() == true);
    float w, qi, qj, qk;
    imu_get_quaternion(&w, &qi, &qj, &qk);
    CHECK(fabsf(w - 0.5f) < 1e-6f && fabsf(qi - 0.5f) < 1e-6f);
    imu_get_quaternion(NULL, NULL, NULL, NULL);
    CHECK(imu_last_accel_ms() == now_ms);
    CHECK(imu_last_rotvec_ms() == now_ms);
}

TEST(test_sensor_reset_reenables_reports)
{
    reset_all();
    launch_detect_init();
    /* A rotation vector from the previous test left rot_valid set */
    CHECK(imu_has_quaternion() == true);

    bno_was_reset = true;
    int calls_before = bno_enable_calls;
    tick(10, 0);
    bno_was_reset = false;

    CHECK(bno_enable_calls == calls_before + 2);   /* accel + rotvec */
    CHECK(imu_has_quaternion() == false);          /* rot_valid cleared */
    CHECK(strstr(Serial.log, "Sensor was reset") != NULL);
}

TEST(test_gps_fallback_confirms_on_sustained_climb)
{
    reset_all();
    launch_detect_init();

    CHECK(launch_detect_gps_fallback_update(1667.0f, 10) == false);  /* baseline */
    CHECK(launch_detect_gps_fallback_update(1668.0f, 11) == false);  /* noise */
    CHECK(launch_detect_gps_fallback_update(1717.0f, 12) == false);  /* climb 1 */
    CHECK(launch_detect_gps_fallback_update(1718.0f, 13) == false);  /* climb 2 */
    CHECK(launch_detect_gps_fallback_update(1600.0f, 14) == false);  /* dip resets */
    CHECK(launch_detect_gps_fallback_update(1717.0f, 15) == false);  /* climb 1 */
    CHECK(launch_detect_gps_fallback_update(1718.0f, 16) == false);  /* climb 2 */
    CHECK(launch_detect_gps_fallback_update(1719.0f, 17) == true);   /* climb 3 */

    /* Identical outcome to the IMU path: state + one-shot edge + timestamp */
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(launch_detect_get_time_since_launch(20) == 3);
    CHECK(launch_detect_is_launched() == true);
    CHECK(launch_detect_is_launched() == false);
    CHECK(launch_detect_gps_fallback_update(2000.0f, 18) == false);  /* no refire */
}

TEST(test_gps_fallback_ignores_pad_drift)
{
    reset_all();
    launch_detect_init();

    launch_detect_gps_fallback_update(1667.0f, 10);  /* baseline */
    /* Stationary GPS alt wanders tens of metres but must never sustain +50 */
    for (int i = 1; i <= 20; i++) {
        float wander = 1667.0f + ((i % 2) ? 25.0f : -25.0f);
        CHECK(launch_detect_gps_fallback_update(wander, 10 + i) == false);
    }
    CHECK(launch_detect_get_state() == LAUNCH_STATE_IDLE);
}

TEST(test_landing_latches_on_quiet_and_stable)
{
    reset_all();
    launch_detect_init();

    /* Get to CONFIRMED via the fallback path */
    launch_detect_gps_fallback_update(1000.0f, 1);
    launch_detect_gps_fallback_update(1100.0f, 2);
    launch_detect_gps_fallback_update(1100.0f, 3);
    CHECK(launch_detect_gps_fallback_update(1100.0f, 4) == true);

    /* Quiet accel, altitude stable within +/-3 m for 60 s */
    total_accel = 0.5f;
    for (uint32_t t = 100; t < 160; t++) {
        float alt = 500.0f + (float)((t % 2) ? 3 : -3);
        CHECK(landing_detect_update(alt, true, t) == false);
    }
    CHECK(landing_detect_update(500.0f, true, 160) == true);
    CHECK(launch_detect_has_landed() == true);
    CHECK(landing_detect_update(500.0f, true, 161) == true);  /* latched */
}

TEST(test_landing_never_during_descent)
{
    reset_all();
    launch_detect_init();
    launch_detect_gps_fallback_update(1000.0f, 1);
    launch_detect_gps_fallback_update(1100.0f, 2);
    launch_detect_gps_fallback_update(1100.0f, 3);
    launch_detect_gps_fallback_update(1100.0f, 4);  /* confirmed */

    /* Parachute descent: quiet accel (gravity removed) but alt falling 6 m/s
     * - the altitude-stability half of the test must keep it from firing. */
    total_accel = 0.5f;
    float alt = 900.0f;
    for (uint32_t t = 100; t < 220; t++) {
        CHECK(landing_detect_update(alt, true, t) == false);
        alt -= 6.0f;
    }
    CHECK(launch_detect_has_landed() == false);
}

TEST(test_landing_requires_launch_and_valid_gps)
{
    reset_all();
    launch_detect_init();
    total_accel = 0.5f;

    /* Not launched yet: quiet + stable means nothing */
    for (uint32_t t = 1; t < 70; t++) {
        CHECK(landing_detect_update(100.0f, true, t) == false);
    }

    /* Launched, but GPS invalid: window must not accumulate */
    launch_detect_gps_fallback_update(1000.0f, 100);
    launch_detect_gps_fallback_update(1100.0f, 101);
    launch_detect_gps_fallback_update(1100.0f, 102);
    launch_detect_gps_fallback_update(1100.0f, 103);  /* confirmed */
    for (uint32_t t = 200; t < 270; t++) {
        CHECK(landing_detect_update(0.0f, false, t) == false);
    }
    CHECK(launch_detect_has_landed() == false);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_success();
    run_test_rotvec_enable_failure_warns_but_init_succeeds();
    run_test_init_failures();
    run_test_launch_detection_happy_path();
    run_test_false_alarm_drops_to_idle();
    run_test_sensor_event_draining_and_accessors();
    run_test_sensor_reset_reenables_reports();
    run_test_gps_fallback_confirms_on_sustained_climb();
    run_test_gps_fallback_ignores_pad_drift();
    run_test_landing_latches_on_quiet_and_stable();
    run_test_landing_never_during_descent();
    run_test_landing_requires_launch_and_valid_gps();

    return TEST_SUMMARY();
}
