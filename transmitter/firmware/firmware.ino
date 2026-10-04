#include "include/mpu_config.h"
#include "include/packet_format.h"
#include <Arduino.h>
#include "include/config.h"
#include "include/radio.h"
#include "include/gps.h"
#include "include/uart.h"
#include "include/launch_detect.h"
#include "include/beacon.h"
#include "include/nav.h"
#include "include/flight_cadence.h"
#include "include/flight_events.h"
#include "include/flight_log.h"

/**
 * Beacon State Machine Documentation
 * =================================
 *
 * The beacon operates in five states. All durations/intervals below are set
 * in include/config.h (values differ between TESTING_MODE and production
 * builds for the legacy constants; the turn-on/pad policy is mode-neutral).
 *
 * 1. TURN_ON (Boot grace, TURN_ON_DURATION_SEC):
 *    - Purpose: immediate operator feedback at power-up ("did it come up?
 *      is the GPS alive? which channel?")
 *    - Behavior: heartbeat every TURN_ON_HEARTBEAT_INTERVAL_SEC; full GPS
 *      packet when the first good fix lands; callsign at boot
 *    - Transition: timer to PRE_LAUNCH (launch detect preempts, as always)
 *
 * 2. PRE_LAUNCH (Quiet pad):
 *    - Purpose: stay provably alive with minimal PA noise near the GPS
 *      front end
 *    - Behavior: heartbeat every PRELAUNCH_HEARTBEAT_INTERVAL_SEC; GPS
 *      audit copy every PRELAUNCH_GPS_AUDIT_INTERVAL_SEC; no fused stream
 *    - Transition: Moves to LAUNCH when IMU launch detection confirms launch
 *
 * 3. LAUNCH (Critical Phase):
 *    - Purpose: Maximum transmission rate during flight for recovery
 *    - Behavior: Continuous transmission (as fast as possible)
 *    - Data Format: inertial trace + fused packets back to back
 *    - Duration: POST_LAUNCH_DURATION_SEC after launch detection
 *    - Transition: Moves to POST_LAUNCH afterwards
 *
 * 4. POST_LAUNCH (Recovery Mode):
 *    - Purpose: High-frequency recovery transmissions
 *    - Behavior: Inertial trace every POST_LAUNCH_PACKET_INTERVAL_SEC +
 *      fused packets at FUSED_TX_INTERVAL_MS (the PA duty cycle at SF10
 *      doesn't tolerate free-running both streams for 10 minutes)
 *    - Duration: POST_LAUNCH_RECOVERY_DURATION_SEC, then BATTERY_SAVE
 *    - Landing detect drops to BATTERY_SAVE early
 *
 * 5. BATTERY_SAVE (Extended Recovery):
 *    - Purpose: Conserve battery for extended recovery operations
 *    - Behavior: Transmit full GPS packet every BATTERY_SAVE_INTERVAL_SEC
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Duration: Indefinite (until power exhaustion)
 *
 * Certified flight events (flight_events.cpp): apogee, drogue deploy, main
 * deploy and landing are detected from fused kinematics and announced as
 * PACKET_TYPE_FLIGHT_EVENT one-shots, repeated FLIGHT_EVENT_REPEATS times
 * so a single RF null can't erase them.
 *
 * Flight anomaly layer: ballistic descent / tumble / IMU loss / GPS outage
 * latch an anomaly level while airborne. While active the inertial-trace
 * cadence tightens to ANOM_BEACON_INTERVAL_S and the anomaly code
 * re-announces every ANOM_EVENT_REPEAT_MS; all-clear for ANOM_CLEAR_MS
 * returns cadence to normal.
 *
 * Additional Features:
 * - Launch detection (one-shot edge from launch_detect_is_launched(), which
 *   this state machine exclusively owns) can trigger from ANY state
 * - Callsign transmission every CALLSIGN_TRANSMIT_INTERVAL_SEC (FCC compliance)
 * - Launch detection: BNO085 IMU sustained-acceleration detection
 * - GPS configuration: Optimized NMEA sentences for minimal data
 */
/* beacon_state_t and all state -> timing mappings live in
 * include/flight_cadence.h so the host tests pin them (see
 * transmitter/tests/test_flight_cadence.cpp). */

// Beacon state machine variables
#if BENCH_TEST_FORCE_LAUNCH
// Bench test: boot straight into continuous-TX LAUNCH mode. See config.h.
volatile beacon_state_t beacon_state = BEACON_STATE_LAUNCH;
volatile uint8_t transmit_beacon_flag = 1;
volatile uint8_t transmit_callsign_flag = 1;
volatile uint8_t transmit_fast_flag = 1;
#else
volatile beacon_state_t beacon_state = BEACON_STATE_TURN_ON;
volatile uint8_t transmit_beacon_flag = 1;
volatile uint8_t transmit_callsign_flag = 1;
volatile uint8_t transmit_fast_flag = 0;
#endif
volatile uint32_t system_time_seconds = 0;
volatile uint32_t last_transmission_time = 0;
volatile uint32_t time_since_last_callsign_tx = 0;
volatile uint32_t post_launch_start_time = 0;

// Counter for 1-second timer
volatile uint16_t ms_counter = 0;

// Timer interrupt handler - called every 1ms
void timer_isr_handler(void) {
    // Increment millisecond counter
    ms_counter++;
    
    // Check if 1 second (1000 counts) has elapsed
    if (ms_counter >= 1000) {
        // Increment system time in seconds
        system_time_seconds++;
        
        // Reset millisecond counter
        ms_counter = 0;
        
        // Debug heartbeat every 30 seconds
        if (system_time_seconds % 30 == 0) {
            Serial.print(F("[Timer] System time: "));
            Serial.print(system_time_seconds);
            Serial.println(F("s"));
        }
        
        // Beacon state machine timing logic - intervals owned by
        // flight_cadence (0 = continuous, used by LAUNCH). An active
        // anomaly tightens the inertial-trace cadence to
        // ANOM_BEACON_INTERVAL_S (never loosens it).
        uint32_t time_since_last_tx = system_time_seconds - last_transmission_time;
        uint32_t interval_s = flight_cadence_beacon_interval_s(beacon_state);
        if (interval_s > ANOM_BEACON_INTERVAL_S && flight_events_anomaly_active()) {
            interval_s = ANOM_BEACON_INTERVAL_S;
        }
        if (interval_s == 0 || time_since_last_tx >= interval_s) {
            transmit_beacon_flag = 1;
        }
    }
}

/* ------------------------------------------------------------------------
 * Hardware watchdog (SAMD51 WDT, ~16 s period).
 *
 * The beacon flies on a rocket: a firmware hang - wedged I2C to the IMU,
 * a stuck UART wait, any unforeseen lockup - with no watchdog means the
 * beacon goes silent exactly when it is unreachable, and the rocket is
 * lost. A 16 s reset costs one missed transmission; a hang costs the
 * airframe. The WDT runs from the internal 1.024 kHz ultra-low-power
 * oscillator, independent of the CPU clock.
 *
 * Fed once per loop() pass. The longest legitimate blocking stretch is
 * ~2 s (IMU settle in launch_detect_init, which runs before the WDT is
 * armed), so 16 s only fires on a genuine lockup.
 * ---------------------------------------------------------------------- */
static void watchdog_init(void) {
    WDT->CTRLA.bit.ENABLE = 0;
    while (WDT->SYNCBUSY.bit.ENABLE);
    WDT->CONFIG.bit.PER = WDT_CONFIG_PER_CYC16384_Val;  /* 16384/1024Hz = 16 s */
    WDT->CTRLA.bit.ENABLE = 1;
    while (WDT->SYNCBUSY.bit.ENABLE);
}

static void watchdog_feed(void) {
    /* Skip the feed if the previous CLEAR is still synchronizing: writing
     * during sync is an error case; the next loop pass feeds instead. */
    if (!WDT->SYNCBUSY.bit.CLEAR) {
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY_Val;
    }
}

// Timer setup for 1ms interrupts
void timer_init(void) {
    // Use Arduino's built-in timer for 1ms interrupts
    // We'll call timer_isr_handler() from loop() instead of using hardware timer
    // This is simpler and sufficient for this application
}

/* Reset-cause forensics: captured before anything can disturb RSTC, sent
 * out with every heartbeat (V3 field). The receiver logs a row when a
 * cause other than plain POR kicks in mid-flight (brown-out or WD hung). */
volatile uint8_t g_boot_rcause = 0;

void setup() {
    g_boot_rcause = RSTC->RCAUSE.reg;   /* survives the reset, capture first */

    // Initialize USB Serial for debugging (optional)
    Serial.begin(115200);

    /* A WDT-forced reboot must be visible: it means the firmware hung in
     * the field. */
    if (g_boot_rcause & RSTC_RCAUSE_WDT) {  /* .bit.WDT collides with the WDT macro */
        Serial.println(F("[Boot] *** Recovered from WATCHDOG RESET (firmware hang) ***"));
    } else if (g_boot_rcause & (RSTC_RCAUSE_BODCORE | RSTC_RCAUSE_BODVDD)) {
        Serial.println(F("[Boot] *** Recovered from BROWN-OUT (supply sag) ***"));
    }
    
    // Initialize hardware
    // No need to disable watchdog on SAMD51 - not enabled by default
    
    // Initialize UART first before using it
    uart_init();
    
    // Initialize radio
    radio_init();    
    
    // Initialize GPS
    gps_init(); 

    // Initialize timer for beacon state machine
    timer_init();
    
    // Initialize launch detection system
    launch_detect_init();

    // Initialize GPS+IMU fusion layer.  Runs regardless of
    // IMU_FUSION_ENABLED so residuals can be logged for tuning; the master
    // switch only gates whether fused packets are transmitted.
    nav_init();

    flight_events_init();

    flight_log_init();

    delay(1000);  // Let everything stabilize

    /* transmit_fast_flag (not transmit_beacon_flag): the parameter means
     * "radio already enabled, skip enable/disable", which is only true in
     * the fast/LAUNCH phase. Passing the pending-TX flag here skipped
     * radio_enable() whenever a beacon TX happened to be queued. */
    beacon_transmit_callsign(transmit_fast_flag);
    time_since_last_callsign_tx = system_time_seconds;

    // Poll GPS for data
    gps_poll_rx();

    /* Armed last: everything above may legitimately block for seconds. */
    watchdog_init();
}

void loop() {
    watchdog_feed();

    // Update timer (simulate 1ms interrupt) - catch up if we missed milliseconds
    static uint32_t last_millis = 0;
    uint32_t current_millis = millis();
    uint32_t elapsed_ms = current_millis - last_millis;
    
    // Call timer handler for each elapsed millisecond
    for (uint32_t i = 0; i < elapsed_ms; i++) {
        timer_isr_handler();
    }
    last_millis = current_millis;
    
    // Update launch detection system (also drains IMU samples)
    launch_detect_update(system_time_seconds, ms_counter);

    // Run the EKF predict step against whatever IMU data just arrived.
    // Safe to call every loop iteration; dt is computed internally.
    nav_predict();

    // Poll GPS for data every loop iteration
    gps_poll_rx();
    
    /* GPS-altitude launch fallback: covers a dead IMU, which would otherwise
     * pin the beacon at pad cadence for the whole flight. Fed once per
     * second with valid-fix altitudes only. */
    static uint32_t last_fallback_feed_s = 0;
    if (launch_detect_get_state() != LAUNCH_STATE_CONFIRMED
        && system_time_seconds != last_fallback_feed_s) {
        const GPSCoordinates_t* c = gps_get_current_coordinates();
        if (c->valid && c->fix_quality >= 1) {
            last_fallback_feed_s = system_time_seconds;
            launch_detect_gps_fallback_update(atof(c->altitude), system_time_seconds);
        }
    }

    /* Landing detection: once post-launch, quiet accel + stable altitude
     * drops the beacon to BATTERY_SAVE cadence early - more recovery time
     * on the same battery. The LANDED event is certified with redundancy:
     * post-topple antenna geometry is random, so three spaced copies. */
    if (beacon_state == BEACON_STATE_LAUNCH || beacon_state == BEACON_STATE_POST_LAUNCH) {
        static uint32_t last_landing_feed_s = 0;
        if (system_time_seconds != last_landing_feed_s) {
            last_landing_feed_s = system_time_seconds;
            const GPSCoordinates_t* c = gps_get_current_coordinates();
            bool gps_ok = c->valid && c->fix_quality >= 1;
            if (landing_detect_update(gps_ok ? atof(c->altitude) : 0.0f,
                                      gps_ok, system_time_seconds)) {
                beacon_state = BEACON_STATE_BATTERY_SAVE;
                transmit_beacon_flag = 1;
                transmit_fast_flag = 0;
                radio_disable();
                beacon_queue_flight_event(FLIGHT_EVENT_LANDED, 0, FLIGHT_EVENT_REPEATS);
                flight_log_event(millis(), 4 /* landed */);
            }
        }
    }

    /* Airborne life-cycle + anomaly detection (flight_events.cpp): fused
     * kinematics + gyro + sensor health fed every loop pass. One-shot
     * edges queue as redundant FLIGHT_EVENT packets; the anomaly LEVEL
     * tightens the trace cadence via the 1 Hz tick and re-announces below. */
    static uint32_t last_anom_announce_ms = 0;
    if (launch_detect_get_state() == LAUNCH_STATE_CONFIRMED) {
        NavFused_t f;
        nav_get_fused(&f);
        float gx, gy, gz;
        launch_detect_get_gyro_rads(&gx, &gy, &gz);

        flight_events_input_t ev_in;
        ev_in.airborne         = !launch_detect_has_landed();
        ev_in.v_d_ms           = f.valid ? f.v_d   : 0.0f;
        ev_in.alt_m            = f.valid ? f.alt_m : 0.0f;
        ev_in.nav_valid        = f.valid ? 1 : 0;
        ev_in.gyro_mag_rads    = sqrtf(gx * gx + gy * gy + gz * gz);
        ev_in.imu_degraded     = f.sensor_degraded ? 1 : 0;
        ev_in.gps_fix_age_ms   = gps_get_fix_age_ms();
        ev_in.t_since_launch_s = launch_detect_get_time_since_launch(system_time_seconds);

        uint32_t now_evt_ms = millis();
        int16_t ev_value = 0;
        uint8_t ev = flight_events_feed(&ev_in, now_evt_ms, &ev_value);
        if (ev != FLIGHT_EVENT_NONE) {
            beacon_queue_flight_event(ev, ev_value, FLIGHT_EVENT_REPEATS);
            flight_log_event(now_evt_ms, 5 /* flight event / anomaly */);
            if (FLIGHT_EVENT_IS_ANOMALY(ev)) {
                last_anom_announce_ms = now_evt_ms;
            }
        }

        /* In-anomaly re-announce: the active code with live descent rate,
         * until the all-clear hysteresis releases the level. */
        if (flight_events_anomaly_active() &&
            (now_evt_ms - last_anom_announce_ms) >= ANOM_EVENT_REPEAT_MS) {
            last_anom_announce_ms = now_evt_ms;
            beacon_queue_flight_event(flight_events_anomaly_code(),
                                      (int16_t)lroundf(f.v_d * 100.0f), 1);
        }
    }

    // Handle launch detection state transitions - can trigger from ANY state
    if (launch_detect_is_launched()) {
        // Launch detected - transition to launch state from any state
        beacon_state = BEACON_STATE_LAUNCH;
        transmit_beacon_flag = 1;
        transmit_fast_flag = 1;

        // Enable radio
        radio_enable();

        // Add a delay to ensure the radio is ready
        delay(10);

        /* Certified T0 first, so even a shred-one-second-later records the
         * one timestamp that turns "silent after packet N" into "failed at
         * T0+xx ms". Also arm the on-chip flight recorder (~250 ms of
         * row-erases; the radio is idle anyway this early) and mark the
         * opening. */
        beacon_transmit_launch_t0(system_time_seconds, /*fast=*/1);
        flight_log_arm();
        flight_log_event(millis(), 3 /* launch */);
    }
    
    // TURN_ON boot grace -> quiet pad once the operator-feedback minute ends.
    // (Launch detect preempts from any state, including this one.)
    if (beacon_state == BEACON_STATE_TURN_ON &&
        flight_cadence_should_leave_turn_on(system_time_seconds)) {
        beacon_state = BEACON_STATE_PRE_LAUNCH;
        transmit_beacon_flag = 1;
    }

    if (beacon_state == BEACON_STATE_LAUNCH) {
        // Check if we should transition to post-launch state
        uint32_t time_since_launch = launch_detect_get_time_since_launch(system_time_seconds);
        if (flight_cadence_should_leave_launch(time_since_launch)) {
            beacon_state = BEACON_STATE_POST_LAUNCH;
            post_launch_start_time = system_time_seconds;
            transmit_beacon_flag = 1;
            transmit_fast_flag = 0;

            // Disable radio
            radio_disable(); 
        }
    }
    
    // Check for transition from POST_LAUNCH to BATTERY_SAVE
    if (beacon_state == BEACON_STATE_POST_LAUNCH) {
        uint32_t time_in_post_launch = system_time_seconds - post_launch_start_time;
        if (flight_cadence_should_leave_post_launch(time_in_post_launch)) {
            beacon_state = BEACON_STATE_BATTERY_SAVE;
            transmit_beacon_flag = 1;
            transmit_fast_flag = 0;
        }
    }
    
    // Handle beacon transmission based on state machine
    if (transmit_beacon_flag) {
        Serial.print(F("[Beacon] Flag set, attempting transmission. Time since last: "));
        Serial.println(system_time_seconds - last_transmission_time);

        bool in_flight = (beacon_state == BEACON_STATE_LAUNCH ||
                          beacon_state == BEACON_STATE_POST_LAUNCH);

        if (in_flight) {
            /* Flight window: the raw GPS stream is replaced by the inertial
             * trace (fused still carries position). Seeing axes move beats
             * position-copies position-copies under a mechanical failure; a
             * lost-GPS fused stream degrades into the EKF's honest DR. */
            beacon_transmit_imu_trace(transmit_fast_flag);

            /* And the same trace goes to the crash-survivable chip flash —
             * even if nobody hears it over the air, a post-scavenger probe
             * can read back WHAT the airframe was doing up to the cut. */
            {
                float ax, ay, az, gx, gy, gz;
                launch_detect_get_accel_xyz(&ax, &ay, &az);
                launch_detect_get_gyro_rads(&gx, &gy, &gz);
                flight_log_imu(millis(),
                               (int16_t)lroundf(ax * 100.0f),
                               (int16_t)lroundf(ay * 100.0f),
                               (int16_t)lroundf(az * 100.0f),
                               (int16_t)lroundf(gx * 5729.58f),
                               (int16_t)lroundf(gy * 5729.58f),
                               (int16_t)lroundf(gz * 5729.58f),
                               0);
            }

            const GPSCoordinates_t* c = gps_get_current_coordinates();
            if (c->valid) {
                float lat_d = gps_nmea_to_decimal(c->lat, c->lat_dir);
                float lon_d = gps_nmea_to_decimal(c->lon, c->lon_dir);
                int16_t alt_m = (int16_t)lroundf(atof(c->altitude));
                flight_log_gps(millis(),
                               (int32_t)lroundf(lat_d * 10000000.0f),
                               (int32_t)lroundf(lon_d * 10000000.0f),
                               alt_m,
                               (uint8_t)atoi(c->satellites), c->fix_quality);
            }
            last_transmission_time = system_time_seconds;
        } else if (beacon_state == BEACON_STATE_TURN_ON ||
                   beacon_state == BEACON_STATE_PRE_LAUNCH) {
            /* Pad states: the beacon tick IS the heartbeat - liveness plus
             * GPS health at the state cadence. Position rides the sparse
             * audit path below instead; the fused stream stays off. */
            gps_poll_rx();
            beacon_transmit_heartbeat(gps_get_current_coordinates(),
                                      system_time_seconds,
                                      flight_cadence_heartbeat_interval_s(beacon_state),
                                      transmit_fast_flag);
            last_transmission_time = system_time_seconds;
        } else {
            gps_poll_rx();

#if USE_BINARY_PACKETS
        uint8_t tx_result = beacon_transmit_gps_data_binary(gps_get_current_coordinates(), system_time_seconds, transmit_fast_flag);
#else
        uint8_t tx_result = beacon_transmit_gps_data(gps_get_current_coordinates(), system_time_seconds, transmit_fast_flag);
#endif

        Serial.print(F("[Beacon] Transmission result: "));
        Serial.println(tx_result);

        // Only update transmission timing if transmission was successful
        if (tx_result) {
            last_transmission_time = system_time_seconds;
        } else {
            /* GPS data rejected (no fix / <4 sats) or TX failed: send a
             * rate-limited heartbeat so the receiver still hears us. */
            beacon_transmit_heartbeat(gps_get_current_coordinates(),
                                      system_time_seconds,
                                      flight_cadence_heartbeat_interval_s(beacon_state),
                                      transmit_fast_flag);
        }
        }

        if (!transmit_fast_flag) {
            transmit_beacon_flag = 0;
        }
    }

    /* Pad GPS audit: one full position packet when the first good fix
     * lands (the logged pad coordinate the drift baseline argues from),
     * then a sparse periodic copy while on the pad. Fix quality is
     * pre-checked here so a no-fix pad doesn't spam rejection logs. */
    if (beacon_state == BEACON_STATE_TURN_ON ||
        beacon_state == BEACON_STATE_PRE_LAUNCH) {
        static uint8_t pad_fix_announced = 0;
        static uint32_t last_pad_audit_s = 0;
        const GPSCoordinates_t* c = gps_get_current_coordinates();
        bool fix_ok = c->valid && c->fix_quality >= 1 && atoi(c->satellites) >= 4;
        uint32_t audit_iv_s = flight_cadence_gps_audit_interval_s(beacon_state);
        if (fix_ok && (!pad_fix_announced ||
                       (audit_iv_s != 0 &&
                        system_time_seconds - last_pad_audit_s >= audit_iv_s))) {
            uint8_t r = beacon_transmit_gps_data_binary(c, system_time_seconds,
                                                        transmit_fast_flag);
            if (r) {
                pad_fix_announced = 1;
                last_pad_audit_s = system_time_seconds;
            }
        }
    }

    if (system_time_seconds - time_since_last_callsign_tx >= flight_cadence_callsign_interval_s() && !transmit_fast_flag) {
        beacon_transmit_callsign(transmit_fast_flag);
        time_since_last_callsign_tx = system_time_seconds;
    }

#if IMU_FUSION_ENABLED
    /* Fused-packet cadence: FUSED_TX_INTERVAL_MS in LAUNCH/POST_LAUNCH for
     * smooth interpolated telemetry, FUSED_TX_INTERVAL_IDLE_MS in
     * battery-save, 0 (stream off) in the pad states - a stationary beacon
     * has nothing to fuse, and the PA quiet protects the GPS front end.
     * Kept separate from the GPS-packet TX path so both streams coexist. */
    static uint32_t last_fused_tx_ms = 0;
    uint32_t now_ms = millis();
    uint32_t fused_interval_ms = flight_cadence_fused_interval_ms(beacon_state);
    if (fused_interval_ms != 0 && nav_is_valid() &&
        (now_ms - last_fused_tx_ms >= fused_interval_ms)) {
        beacon_transmit_fused_data(system_time_seconds, transmit_fast_flag);
        last_fused_tx_ms = now_ms;
    }
#endif

    /* Two-way channel (v2 radios): only in pad/recovery phases the beacon
     * has a real radio-quiet gap. Tune: ~1 Hz worth of short listens, so a
     * command sequence sees multiple windows per second. */
    {
        static uint32_t last_listen_ms = 0;
        if (now_ms - last_listen_ms >= 1000) {
            last_listen_ms = now_ms;
            beacon_poll_commands(beacon_state == BEACON_STATE_LAUNCH ||
                                 beacon_state == BEACON_STATE_POST_LAUNCH);
        }
    }

    /* Queued certified-event transmissions: one packet per loop pass max,
     * FLIGHT_EVENT_SPACING_MS between copies of the same edge. */
    beacon_service_flight_events(now_ms, system_time_seconds, transmit_fast_flag);
}
