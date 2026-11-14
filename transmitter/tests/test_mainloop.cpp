/**
 * @file test_mainloop.cpp
 * @brief Host simulation of the REAL firmware.ino loop(): boots the sketch,
 *        then scripts GPS fixes, an IMU launch burst, descent, and landing,
 *        over minutes of simulated time.
 *
 * This is the last untested structural unit on the TX side: the module
 * tests cover the pieces, flight_cadence covers the pacing table, but the
 * composition glue (flag/heartbeat interplay at fix loss, state entries
 * forcing a TX, radio enable/disable choreography, callsign gating) only
 * lives in loop().
 *
 * Wire format checks ride along: every packet type on the air is counted
 * by type byte, so the sim asserts exact rates per phase.
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "include/gps.h"
#include "include/radio.h"
#include "include/nav.h"
#include "include/launch_detect.h"
#include "include/packet_format.h"
#include "include/flight_cadence.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Arduino / peripheral fakes                                          */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
WireClass Wire;

static unsigned long sim_ms = 0;
unsigned long millis(void) { return sim_ms; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

/* SAMD51 WDT + RSTC register fakes (firmware.ino pokes them directly).
 * SYNCBUSY reads 0 forever: writes never "synchronize" on the host. */
static struct {
    struct { struct { uint8_t ENABLE; } bit; } CTRLA;
    struct { struct { uint32_t ENABLE; uint32_t CLEAR; } bit; } SYNCBUSY;
    struct { struct { uint8_t PER; } bit; } CONFIG;
    struct { uint8_t reg; } CLEAR;
} fake_wdt;
#define WDT (&fake_wdt)
#define WDT_CONFIG_PER_CYC16384_Val  0x0B
#define WDT_CLEAR_CLEAR_KEY_Val      0xA5
static struct { struct { uint8_t reg; } RCAUSE; } fake_rstc;
#define RSTC (&fake_rstc)
#define RSTC_RCAUSE_WDT     0x20
#define RSTC_RCAUSE_BODCORE 0x02
#define RSTC_RCAUSE_BODVDD  0x04
#define RSTC_RCAUSE_POR     0x01

/* ------------------------------------------------------------------ */
/* Radio fakes: count every packet by type                             */
/* ------------------------------------------------------------------ */

static uint32_t tx_by_type[16];   /* indexed by packet_type byte */
static uint32_t fev_code_count[256]; /* FLIGHT_EVENT_* code histogram   */
static uint32_t tx_callsigns = 0;
static uint32_t tx_other = 0;
static uint32_t radio_enables = 0, radio_disables = 0;
static uint8_t  radio_enabled_flag = 0;
static int      tx_result = 0;
static uint8_t  last_imu_pkt[IMU_TRACE_PACKET_SIZE];  /* newest IMU on air */
static uint8_t  have_last_imu = 0;

void radio_enable(void)  { radio_enables++; radio_enabled_flag = 1; }
void radio_disable(void) { radio_disables++; radio_enabled_flag = 0; }
void radio_init(void)    {}
uint8_t radio_get_channel(void) { return 0; }

int transmit_packet(const uint8_t *data, size_t length)
{
    uint8_t t = data[0];
    if (t >= PACKET_TYPE_GPS && t <= PACKET_TYPE_HELLO) {
        tx_by_type[t]++;
        if (t == PACKET_TYPE_FLIGHT_EVENT && length >= 3)
            fev_code_count[data[2]]++;      /* code field, offset 2 */
        if (t == PACKET_TYPE_IMU && length == sizeof(last_imu_pkt)) {
            memcpy(last_imu_pkt, data, sizeof(last_imu_pkt));
            have_last_imu = 1;
        }
    } else if (t >= 0x20 && t <= 0x7E) {
        tx_callsigns++;              /* printable = callsign text */
    } else {
        tx_other++;
    }
    return tx_result;
}
int transmit_string(const char *str)
{
    return transmit_packet((const uint8_t *)str, strlen(str) + 1);
}

/* ------------------------------------------------------------------ */
/* IMU event injection (same pattern as test_flight_profile)           */
/* ------------------------------------------------------------------ */

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
    ev.un.rotationVector.i    = i;
    ev.un.rotationVector.j    = j;
    ev.un.rotationVector.k    = k;
    bno_events[bno_event_tail++ % 32] = ev;
}

/* ------------------------------------------------------------------ */
/* GPS fakes with a scripted fix stream                                */
/* ------------------------------------------------------------------ */

static GPSCoordinates_t fake_coords;
static uint8_t fake_fix_ok = 1;

void    gps_init(void) {}
uint16_t gps_poll_rx(void) { return 0; }
const GPSCoordinates_t* gps_get_current_coordinates(void) { return &fake_coords; }
uint8_t gps_get_health(void) { return HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0); }
static uint8_t gps_in_flight_flag = 0;   /* last gps_set_in_flight() value */
void gps_set_in_flight(uint8_t f) { gps_in_flight_flag = f; }
static uint32_t fake_fix_age_ms = 250;  /* GPS fix age seen by the anomaly logic */
uint32_t gps_get_fix_age_ms(void) { return fake_fix_age_ms; }

/* flight_log.cpp is target-only (raw NVMCTRL); stub its API here. The
 * host test's concern is WHEN the recorder is armed/written, not the
 * register pokes themselves (those get verified by bench smoke). */
static uint8_t flog_armed_count = 0;
static uint32_t flog_imu_calls = 0;
static uint32_t flog_gps_calls = 0;
static uint32_t flog_event_calls = 0;
void     flight_log_init(void) {}
void     flight_log_arm(void) { flog_armed_count++; }
static int16_t  flog_imu_last[7];   /* ax, ay, az, gx, gy, gz, peak */
void     flight_log_imu(uint32_t ms, int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz, int16_t pk)
{
    (void)ms; flog_imu_calls++;
    int16_t v[7] = { ax, ay, az, gx, gy, gz, pk };
    memcpy(flog_imu_last, v, sizeof(v));
}
void     flight_log_gps(uint32_t ms, int32_t, int32_t, int16_t, uint8_t, uint8_t)
         { (void)ms; flog_gps_calls++; }
static uint32_t flog_kind_count[16];   /* flash-log event records by kind */
void     flight_log_event(uint32_t ms, uint8_t kind)
         { (void)ms; flog_event_calls++; flog_kind_count[kind & 15]++; }
static uint8_t flog_last_event_code = 0;
static int16_t flog_last_event_value = 0;
void     flight_log_event_ex(uint32_t ms, uint8_t kind, uint8_t code, int16_t value)
         { (void)ms; flog_event_calls++; flog_kind_count[kind & 15]++;
           flog_last_event_code = code; flog_last_event_value = value; }
static uint32_t flog_fused_calls = 0;
void     flight_log_fused(uint32_t ms, int32_t, int32_t, int16_t, int16_t, int16_t, int16_t, uint8_t)
         { (void)ms; flog_fused_calls++; }
static uint32_t flog_dump_calls = 0;
void     flight_log_dump_serial(void) { flog_dump_calls++; }
/* Reboot-resume seam: the host can't fake NVM rows, so the test scripts
 * the persisted state directly and spies on which path setup() took. */
static uint8_t  flog_prev_flying_flag = 0;
static uint32_t flog_resume_calls = 0;
static uint32_t flog_landed_marks = 0;
uint8_t  flight_log_prev_flying(void) { return flog_prev_flying_flag; }
static uint8_t flog_resume_result = 1;  /* 0 = append point not found */
uint8_t  flight_log_resume(void)      { flog_resume_calls++; return flog_resume_result; }
void     flight_log_mark_landed(void) { flog_landed_marks++; }
uint8_t  flog_armed_count_value(void) { return flog_armed_count; }
uint32_t flog_gps_calls_value(void)   { return flog_gps_calls; }

/* Two-way scaffolding stubs (beacon_poll_commands calls these) */
bool radio_is_transmitting(void) { return false; }
int  radio_poll_rx(uint8_t *out, size_t max_len, uint16_t timeout_ms)
{ (void)out; (void)max_len; (void)timeout_ms; return 0; }
int  radio_transmit_ack(uint8_t rocket_id, uint8_t cmd_code,
                        uint8_t seq_hi, uint8_t seq_lo, uint8_t echo)
{ (void)rocket_id; (void)cmd_code; (void)seq_hi; (void)seq_lo; (void)echo; return 0; }
float   gps_nmea_to_decimal(const char *n, char d)
{
    double v = atof(n);
    /* (int) of a non-finite value is UB: report it as NaN, which every
     * caller rejects - the same contract as the firmware converter. */
    if (!isfinite(v)) return NAN;
    int deg = (int)(v / 100.0);
    double out = deg + (v - deg * 100.0) / 60.0;
    if (d == 'S' || d == 'W') out = -out;
    return (float)out;
}

static void script_fix(double lat, double lon, double alt, int sats,
                       uint8_t fix_quality)
{
    /* NMEA strings the way the u-blox prints them */
    int latd = (int)fabs(lat);
    double latm = (fabs(lat) - latd) * 60.0;
    snprintf(fake_coords.lat, sizeof(fake_coords.lat), "%02d%08.5f", latd, latm);
    fake_coords.lat_dir = lat < 0 ? 'S' : 'N';
    int lond = (int)fabs(lon);
    double lonm = (fabs(lon) - lond) * 60.0;
    snprintf(fake_coords.lon, sizeof(fake_coords.lon), "%03d%08.5f", lond, lonm);
    fake_coords.lon_dir = lon < 0 ? 'W' : 'E';
    snprintf(fake_coords.altitude, sizeof(fake_coords.altitude), "%.1f", alt);
    snprintf(fake_coords.satellites, sizeof(fake_coords.satellites), "%d", sats);
    fake_coords.fix_quality = fix_quality;
    fake_coords.valid = fake_fix_ok && fix_quality >= 1;
}

void uart_init(void) {}

/* ------------------------------------------------------------------ */
/* Real modules + the sketch itself                                    */
/* ------------------------------------------------------------------ */

#include "../firmware/launch_detect.cpp"
#include "../firmware/nav.cpp"
#include "../firmware/ekf.cpp"
#include "../firmware/beacon.cpp"
#include "../firmware/firmware.ino"   /* the sketch under test - AFTER fakes */

/* ------------------------------------------------------------------ */
/* Sim driver                                                          */
/* ------------------------------------------------------------------ */

#define SIM_LAT 39.8900004
#define SIM_LON (-104.8850002)
#define SIM_ALT 1650.0

static void sim_run_burst(uint32_t duration_ms, float burst_z);

/* Advance the sim clock, calling loop() every 10 ms like a fast MCU.
 * Feeds the nav layer 1 Hz from the scripted fix exactly as gps.cpp
 * would, and keeps the IMU streaming so the health flags stay up.
 * burst_z != 0 scripts a motor burn: ONLY the burst accel is queued
 * (interleaving idle accel would break sustained-threshold detection). */
static void sim_run(uint32_t duration_ms)
{
    sim_run_burst(duration_ms, 0.0f);
}

static void sim_run_burst(uint32_t duration_ms, float burst_z)
{
    uint32_t end = sim_ms + duration_ms;
    uint32_t last_gps_feed = 0;
    uint32_t last_burst_ms = 0;
    while (sim_ms < end) {
        sim_ms += 10;
        if (burst_z != 0.0f) {
            if (sim_ms - last_burst_ms >= 50) {
                last_burst_ms = sim_ms;
                push_accel(0.0f, 0.0f, burst_z);
            }
        } else {
            push_accel(0.0f, 0.0f, 0.0f);
        }
        push_rotvec(1.0f, 0.0f, 0.0f, 0.0f);
        if (fake_fix_ok && sim_ms / 1000 != last_gps_feed) {
            last_gps_feed = (uint32_t)(sim_ms / 1000);
            nav_update_from_gps(SIM_LAT, SIM_LON, (float)SIM_ALT, 8, 1);
        }
        loop();
    }
}

static uint32_t type_count(uint8_t type) { return tx_by_type[type]; }

/* ------------------------------------------------------------------ */

int main(void)
{
    Test_SetTickAutoAdvance(0);

    /* ---------- Boot: turn-on grace, fix healthy ---------- */
    fake_fix_ok = 1;
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 9, 1);
    setup();
    CHECK(tx_callsigns == 1);                     /* boot ID (FCC) */
    /* Boot identity packet right behind it (fw hash on the air) */
    CHECK(type_count(PACKET_TYPE_HELLO) == 1);
    CHECK(beacon_state == BEACON_STATE_TURN_ON);

    sim_run(40000);                               /* 40 s of the grace minute */
    /* Turn-on heartbeat every 5 s: boot TX + t=5..40 -> 8 or 9 */
    CHECK(type_count(PACKET_TYPE_HEARTBEAT) >= 8 &&
          type_count(PACKET_TYPE_HEARTBEAT) <= 10);
    /* Exactly one full position: the first-good-fix pad audit copy */
    CHECK(type_count(PACKET_TYPE_GPS) == 1);
    /* No fused stream on the pad at all (PA stays out of the GPS front
     * end's face; a stationary beacon has nothing to fuse) */
    CHECK(type_count(PACKET_TYPE_FUSED) == 0);
    /* Pad TX uses radio enable/disable around each non-fast packet */
    CHECK(radio_enables > 0 && radio_disables == radio_enables);
    CHECK(radio_enabled_flag == 0);

    /* ---------- USB 'DUMP' command: the serial command the boot banner
     * advertises to recovery crews (was advertised but never wired) --- */
    Serial.inject("DUMP\n");
    loop();
    CHECK(flog_dump_calls == 1);              /* on the pad: allowed */
    Serial.inject("JUNK\nDUMP\r\n");          /* garbage line, then real */
    loop();
    CHECK(flog_dump_calls == 2);              /* only the real one fires */
    Serial.inject("DU");                      /* a command split across */
    loop();                                   /* loop passes reassembles */
    Serial.inject("MP\n");
    loop();
    CHECK(flog_dump_calls == 3);

    /* ---------- GPS dies at the pad (cable knocked loose) ---------- */
    fake_fix_ok = 0;
    fake_coords.valid = 0;
    uint32_t hb0 = type_count(PACKET_TYPE_HEARTBEAT);
    sim_run(40000);                               /* spans the turn-on exit
                                                     * at t=60 -> PRE_LAUNCH  */
    /* Heartbeats continue (they report the broken GPS health, that is the
     * point): turn-on-tempo ticks through t=55 plus the PRE_LAUNCH entry */
    uint32_t hbs = type_count(PACKET_TYPE_HEARTBEAT) - hb0;
    CHECK(hbs >= 2 && hbs <= 6);
    CHECK(type_count(PACKET_TYPE_GPS) == 1);      /* no new position TX */

    /* ---------- Fix returns, then LAUNCH (sustained accel burst) ----- */
    fake_fix_ok = 1;
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 9, 1);
    sim_run(8000);                                /* re-anchor cleanly */
    CHECK(nav_is_valid() == true);

    uint32_t enables_before = radio_enables;
    sim_run_burst(500, 25.0f);                    /* 500 ms of boost */
    /* The loop must have taken the launch transition to LAUNCH + fast TX */
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(beacon_state == BEACON_STATE_LAUNCH);
    sim_run(50);                                  /* one loop pass */
    CHECK(gps_in_flight_flag == 1);               /* GPS watchdog: no resets */
    CHECK(transmit_fast_flag == 1);
    CHECK(radio_enables == enables_before + 1);   /* radio keyed once */
    CHECK(radio_enabled_flag == 1);
    CHECK(flog_armed_count_value() >= 1);         /* on-chip recorder armed */

    /* DUMP refused while airborne - a 40 s printout is not worth lost
     * fixes; the crew gets it back after touchdown */
    Serial.clear();                            /* drop residual prints */
    Serial.inject("DUMP\n");
    loop();
    CHECK(flog_dump_calls == 3);
    CHECK(strstr(Serial.log, "airborne") != NULL);

    /* LAUNCH exits to POST_LAUNCH after POST_LAUNCH_DURATION_SEC */
    sim_run(3000);
    CHECK(beacon_state == BEACON_STATE_POST_LAUNCH);
    CHECK(transmit_fast_flag == 0);
    CHECK(radio_enabled_flag == 0);               /* radio released */

    /* ---------- Descent under chute: fused + paced inertial stream ----- */
    uint32_t fus0 = type_count(PACKET_TYPE_FUSED);
    uint32_t imu0 = type_count(PACKET_TYPE_IMU);
    uint32_t mx0  = type_count(PACKET_TYPE_MAXIMA);
    sim_run(30000);                               /* 30 s of recovery window */
    uint32_t fus = type_count(PACKET_TYPE_FUSED) - fus0;
    uint32_t imu = type_count(PACKET_TYPE_IMU) - imu0;
    uint32_t mxs = type_count(PACKET_TYPE_MAXIMA) - mx0;
    CHECK(fus == 20);                             /* 1500 ms cadence exactly */
    CHECK((imu == 15 || imu == 16));              /* 2 s paced inertial stream */
    /* The flash copy is the packet that went on the air, field for field
     * (it used to be re-derived with different, wrapping conversions). */
    CHECK(have_last_imu == 1);
    {
        ImuTracePacket_t onair;
        memcpy(&onair, last_imu_pkt, sizeof(onair));
        CHECK((onair.ts_ms & IMU_TS_V3_FLAG) != 0);
        CHECK(flog_imu_last[0] == onair.accel_x_cg &&
              flog_imu_last[1] == onair.accel_y_cg &&
              flog_imu_last[2] == onair.accel_z_cg &&
              flog_imu_last[3] == onair.gyro_x_dds &&
              flog_imu_last[4] == onair.gyro_y_dds &&
              flog_imu_last[5] == onair.gyro_z_dds &&
              flog_imu_last[6] == onair.peak_accel_mg);
    }
    CHECK(mxs >= 5 && mxs <= 7);                  /* 5 s maxima recap cadence */
    CHECK(flog_gps_calls_value() >= 14);          /* GPS rows landing in the flash log */
    /* Raw GPS position packets stop on air - fused+IMU carry the invade */

    /* Corrupt altitude field ("nan" parses literally via atof): the
     * flight-log record must be SKIPPED rather than pass NaN through
     * lroundf (UB) into the flash log. Landing detect treats the same
     * field as no-fix, so the window keeps breaking instead of latching
     * on a poisoned "stable" reading. */
    snprintf(fake_coords.altitude, sizeof(fake_coords.altitude), "nan");
    uint32_t flog0 = flog_gps_calls_value();
    sim_run(4500);                                /* ~2 log ticks poisoned */
    CHECK(flog_gps_calls_value() == flog0);
    CHECK(launch_detect_has_landed() == false);
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 8, 1);  /* honest field returns */
    sim_run(5000);
    CHECK(flog_gps_calls_value() > flog0);        /* logging resumes */

    /* ---------- Touchdown: quiet IMU + stable alt -> battery save ---- */
    sim_run((LAND_QUIET_S + 20) * 1000UL);
    CHECK(launch_detect_has_landed() == true);
    CHECK(beacon_state == BEACON_STATE_BATTERY_SAVE);
    CHECK(gps_in_flight_flag == 0);               /* landed: resets allowed */
    CHECK(fev_code_count[FLIGHT_EVENT_NONE] == 0);  /* no code-0x00 events aired */
    /* Certified LANDED event: FLIGHT_EVENT_REPEATS copies went on the air
     * (spacing ~1.2 s << the quiet window above) and nothing else fired.
     * The landing transition also fires one final maxima copy. */
    CHECK(type_count(PACKET_TYPE_FLIGHT_EVENT) == FLIGHT_EVENT_REPEATS);
    CHECK(type_count(PACKET_TYPE_MAXIMA) >= mxs + 1);

    uint32_t fus1 = type_count(PACKET_TYPE_FUSED);
    uint32_t gps1 = type_count(PACKET_TYPE_GPS);
    sim_run(540000);                            /* 9 min on the ground */
    uint32_t fus_save = type_count(PACKET_TYPE_FUSED) - fus1;
    CHECK(fus_save >= 106 && fus_save <= 110);             /* 5 s idle cadence */
    uint32_t gps_save = type_count(PACKET_TYPE_GPS) - gps1;
    CHECK(gps_save >= 8 && gps_save <= 11);                /* ~60 s spacing */

    /* ---------- FCC ID cadence across the whole sim ---------------- */
    CHECK(tx_callsigns == 3);                     /* boot + 300 s + 600 s */
    CHECK(tx_other == 0);                         /* nothing unaccounted */
    CHECK(flog_landed_marks == 1);                /* state row -> LANDED */

    /* ---------- Mid-flight reboot-resume -----------------------------
     * A WDT bite / brown-out / battery bounce while airborne used to
     * strand the beacon at pad cadence for the rest of the flight: the
     * launch FSM could never re-confirm without another boost. The state
     * row survives the reset; setup() must RESUME, not re-arm. */
    uint8_t  arms_before  = flog_armed_count_value();
    uint32_t call_before  = tx_callsigns;
    uint32_t hello_before = type_count(PACKET_TYPE_HELLO);
    fake_rstc.RCAUSE.reg  = RSTC_RCAUSE_WDT;     /* the reset that killed it */
    flog_prev_flying_flag = 1;                   /* marker row still FLYING */
    /* A real reset clears RAM: the scheduler clock restarts at 0. Running
     * setup() with the old ~700 s uptime hid a resume that could never
     * leave LAUNCH (launch time 0 doubled as "never launched"). */
    system_time_seconds = 0;
    ms_counter = 0;
    last_transmission_time = 0;
    last_callsign_tx_s = 0;
    post_launch_start_time = 0;
    setup();                                     /* <- the reboot itself   */
    CHECK(beacon_state == BEACON_STATE_LAUNCH);  /* airborne cadence now */
    CHECK(launch_detect_get_state() == LAUNCH_STATE_CONFIRMED);
    CHECK(transmit_fast_flag == 1);
    CHECK(radio_enabled_flag == 1);
    CHECK(flog_resume_calls == 1);               /* append point restored */
    CHECK(flog_armed_count_value() == arms_before); /* NO erase of survivors */
    CHECK(tx_callsigns == call_before + 1);      /* boot ID still airs */
    CHECK(type_count(PACKET_TYPE_HELLO) == hello_before + 1);

    /* The reboot itself is certified on the air: ANOM_REBOOT copies ride
     * the certified event path alongside normal telemetry. */
    sim_run(5000);
    CHECK(fev_code_count[FLIGHT_EVENT_ANOM_REBOOT] == FLIGHT_EVENT_REPEATS);

    /* The resumed flight is on the normal post-boost schedule: time since
     * launch counts from the resume, LAUNCH hands over to POST_LAUNCH,
     * and the continuous-TX phase ends so the FCC callsign airs again. */
    CHECK(launch_detect_get_time_since_launch(system_time_seconds) >= 4);
    CHECK(beacon_state == BEACON_STATE_POST_LAUNCH);
    CHECK(transmit_fast_flag == 0);
    uint32_t calls_after_resume = tx_callsigns;
    sim_run((flight_cadence_callsign_interval_s() + 5) * 1000UL);
    CHECK(tx_callsigns > calls_after_resume);    /* ID resumes in flight */

    /* The resumed flight still completes its life-cycle: quiet + stable
     * re-latches landing, which flips the marker to LANDED again. */
    sim_run((LAND_QUIET_S + 20) * 1000UL);
    CHECK(launch_detect_has_landed() == true);
    CHECK(beacon_state == BEACON_STATE_BATTERY_SAVE);
    CHECK(flog_landed_marks == 2);               /* touchdown #1 + re-latch */

    /* ---------- Watchdog never saw >16 s of silence ----------------- */
    /* (loop() feeds it every pass; the sim asserting loop-liveness is
     *  the point of the whole file) */
    CHECK(fake_wdt.CLEAR.reg == WDT_CLEAR_CLEAR_KEY_Val);

    /* ---------- Fresh boot after a brown-out, then a flight whose landing
     * never latches: in-flight anomaly events, the recovery-duration exit
     * to BATTERY_SAVE, and the no-fix heartbeat fallback on the ground --- */
    fake_rstc.RCAUSE.reg = RSTC_RCAUSE_BODVDD;
    flog_prev_flying_flag = 0;                   /* marker row LANDED */
    beacon_state = BEACON_STATE_TURN_ON;         /* what a RAM reset leaves */
    transmit_fast_flag = 0;
    transmit_beacon_flag = 1;
    system_time_seconds = 0;
    ms_counter = 0;
    last_transmission_time = 0;
    last_callsign_tx_s = 0;
    post_launch_start_time = 0;
    Serial.clear();
    setup();
    CHECK(strstr(Serial.log, "BROWN-OUT") != NULL);   /* boot banner says so */
    CHECK(beacon_state == BEACON_STATE_TURN_ON);
    sim_run(70000);                              /* turn-on grace -> pad */
    CHECK(beacon_state == BEACON_STATE_PRE_LAUNCH);
    uint32_t rcause_recs = flog_kind_count[8];
    sim_run_burst(500, 25.0f);                   /* boost */
    CHECK(beacon_state == BEACON_STATE_LAUNCH);
    CHECK(flog_kind_count[8] == rcause_recs + 1);   /* reset cause opens the record */
    sim_run(3000);
    CHECK(beacon_state == BEACON_STATE_POST_LAUNCH);

    /* GPS outage aloft: certified entry copies, a tightened inertial
     * cadence (ANOM_BEACON_INTERVAL_S), and periodic re-announces */
    uint32_t outage0 = fev_code_count[FLIGHT_EVENT_ANOM_GPS_OUTAGE];
    uint32_t imu_o0 = type_count(PACKET_TYPE_IMU);
    fake_fix_age_ms = ANOM_GPS_OUTAGE_MS + 1000;
    sim_run(ANOM_EVENT_REPEAT_MS + 3000);
    CHECK(flight_events_anomaly_active());
    CHECK(fev_code_count[FLIGHT_EVENT_ANOM_GPS_OUTAGE] >= outage0 + FLIGHT_EVENT_REPEATS + 1);
    uint32_t imu_outage = type_count(PACKET_TYPE_IMU) - imu_o0;
    fake_fix_age_ms = 250;
    sim_run(ANOM_CLEAR_MS + 2000);
    CHECK(!flight_events_anomaly_active());
    CHECK(fev_code_count[FLIGHT_EVENT_NONE] == 0);
    uint32_t imu_c0 = type_count(PACKET_TYPE_IMU);
    sim_run(ANOM_EVENT_REPEAT_MS + 3000);       /* same span, no anomaly */
    CHECK(imu_outage > type_count(PACKET_TYPE_IMU) - imu_c0);

    /* DUMP is refused for the whole airborne span, POST_LAUNCH too */
    Serial.clear();
    uint32_t dumps_pl = flog_dump_calls;
    Serial.inject("DUMP\n");
    loop();
    CHECK(flog_dump_calls == dumps_pl);
    CHECK(strstr(Serial.log, "airborne") != NULL);

    /* The airframe never goes quiet (stuck in a tree in the wind):
     * landing never latches and POST_LAUNCH ends on its timeout */
    sim_run_burst((POST_LAUNCH_RECOVERY_DURATION_SEC + 5) * 1000UL, 5.0f);
    CHECK(launch_detect_has_landed() == false);
    CHECK(beacon_state == BEACON_STATE_BATTERY_SAVE);
    CHECK(flog_kind_count[7] >= 1);             /* battery-save record */

    /* Fix lost on the ground: the rejected position TX falls back to a
     * heartbeat so the receiver still hears the beacon */
    fake_coords.valid = 0;
    uint32_t hb_ground = type_count(PACKET_TYPE_HEARTBEAT);
    sim_run(130000);
    CHECK(type_count(PACKET_TYPE_HEARTBEAT) > hb_ground);
    script_fix(SIM_LAT, SIM_LON, SIM_ALT, 9, 1);

    /* Overlong serial line: dropped, the parser resyncs at the newline */
    uint32_t dumps = flog_dump_calls;
    Serial.inject("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\nDUMP\n");
    loop();
    CHECK(flog_dump_calls == dumps + 1);

    /* Mid-flight reboot whose flash resume fails (append point not
     * found): the flight state still resumes, but the surviving record
     * is never re-armed (erased) and no reset-cause row is appended */
    fake_rstc.RCAUSE.reg = RSTC_RCAUSE_WDT;
    flog_prev_flying_flag = 1;
    flog_resume_result = 0;
    uint8_t  arms_fail = flog_armed_count_value();
    uint32_t rc_fail   = flog_kind_count[8];
    system_time_seconds = 0;
    ms_counter = 0;
    last_transmission_time = 0;
    last_callsign_tx_s = 0;
    post_launch_start_time = 0;
    setup();
    CHECK(beacon_state == BEACON_STATE_LAUNCH);
    CHECK(flog_armed_count_value() == arms_fail);
    CHECK(flog_kind_count[8] == rc_fail);
    flog_resume_result = 1;

    printf("mainloop sim: GPS=%lu FUS=%lu HB=%lu CALL=%lu EVT=%lu\n",
           (unsigned long)type_count(PACKET_TYPE_GPS),
           (unsigned long)type_count(PACKET_TYPE_FUSED),
           (unsigned long)type_count(PACKET_TYPE_HEARTBEAT),
           (unsigned long)tx_callsigns,
           (unsigned long)type_count(PACKET_TYPE_FLIGHT_EVENT));

    return TEST_SUMMARY();
}
