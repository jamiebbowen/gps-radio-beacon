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

/**
 * Beacon State Machine Documentation
 * =================================
 * 
 * The beacon operates in four distinct states optimized for rocketry
 * applications. All durations/intervals below are set in include/config.h
 * (values differ between TESTING_MODE and production builds).
 * 
 * 1. PRE_LAUNCH (Initial State):
 *    - Purpose: Conserve power while on the launch pad
 *    - Behavior: Transmit GPS data every PRE_LAUNCH_INTERVAL_SEC seconds
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Transition: Moves to LAUNCH when IMU launch detection confirms launch
 * 
 * 2. LAUNCH (Critical Phase):
 *    - Purpose: Maximum transmission rate during flight for recovery
 *    - Behavior: Continuous transmission (as fast as possible)
 *    - Data Format: Fast GPS packet (lat, lon, alt only) for speed
 *    - Duration: POST_LAUNCH_DURATION_SEC after launch detection
 *    - Transition: Moves to POST_LAUNCH afterwards
 * 
 * 3. POST_LAUNCH (Recovery Mode):
 *    - Purpose: High-frequency recovery transmissions
 *    - Behavior: Continuous transmission for maximum recovery chances
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Duration: POST_LAUNCH_RECOVERY_DURATION_SEC, then BATTERY_SAVE
 * 
 * 4. BATTERY_SAVE (Extended Recovery):
 *    - Purpose: Conserve battery for extended recovery operations
 *    - Behavior: Transmit full GPS packet every BATTERY_SAVE_INTERVAL_SEC
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Duration: Indefinite (until power exhaustion)
 * 
 * Additional Features:
 * - Launch detection (one-shot edge from launch_detect_is_launched(), which
 *   this state machine exclusively owns) can trigger from ANY state
 * - Callsign transmission every CALLSIGN_TRANSMIT_INTERVAL_SEC (FCC compliance)
 * - Launch detection: BNO085 IMU sustained-acceleration detection
 * - GPS configuration: Optimized NMEA sentences for minimal data
 */
typedef enum {
    BEACON_STATE_PRE_LAUNCH,      // Pre-launch: interval TX, full packets
    BEACON_STATE_LAUNCH,          // Launch: continuous fast packets
    BEACON_STATE_POST_LAUNCH,     // Post-launch: continuous full packets
    BEACON_STATE_BATTERY_SAVE     // Extended recovery: interval TX, full packets
} beacon_state_t;

// Beacon state machine variables
#if BENCH_TEST_FORCE_LAUNCH
// Bench test: boot straight into continuous-TX LAUNCH mode. See config.h.
volatile beacon_state_t beacon_state = BEACON_STATE_LAUNCH;
volatile uint8_t transmit_beacon_flag = 1;
volatile uint8_t transmit_callsign_flag = 1;
volatile uint8_t transmit_fast_flag = 1;
#else
volatile beacon_state_t beacon_state = BEACON_STATE_PRE_LAUNCH;
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
        
        // Beacon state machine timing logic
        uint32_t time_since_last_tx = system_time_seconds - last_transmission_time;
        
        switch (beacon_state) {
            case BEACON_STATE_PRE_LAUNCH:
                // Pre-launch - transmit every 30 seconds
                if (time_since_last_tx >= PRE_LAUNCH_INTERVAL_SEC) {
                    Serial.print(F("[Timer] Setting beacon flag. Time since last TX: "));
                    Serial.println(time_since_last_tx);
                    transmit_beacon_flag = 1;
                }
                break;
                
            case BEACON_STATE_LAUNCH:
                // Launch detected - continuous sending
                transmit_beacon_flag = 1;
                break;
                
            case BEACON_STATE_POST_LAUNCH:
                // Post-launch mode - continuous sending
                transmit_beacon_flag = 1;
                break;
                
            case BEACON_STATE_BATTERY_SAVE:
                // Battery save mode - transmit every 30 seconds
                if (time_since_last_tx >= BATTERY_SAVE_INTERVAL_SEC) {
                    transmit_beacon_flag = 1;
                }
                break;
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

void setup() {
    // Initialize USB Serial for debugging (optional)
    Serial.begin(115200);
    
    /* A WDT-forced reboot must be visible: it means the firmware hung in
     * the field. RCAUSE survives the reset. */
    if (RSTC->RCAUSE.reg & RSTC_RCAUSE_WDT) {  /* .bit.WDT collides with the WDT macro */
        Serial.println(F("[Boot] *** Recovered from WATCHDOG RESET (firmware hang) ***"));
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
    }
    
    if (beacon_state == BEACON_STATE_LAUNCH) {
        // Check if we should transition to post-launch state
        uint32_t time_since_launch = launch_detect_get_time_since_launch(system_time_seconds);
        if (time_since_launch >= POST_LAUNCH_DURATION_SEC) {
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
        if (time_in_post_launch >= POST_LAUNCH_RECOVERY_DURATION_SEC) {
            beacon_state = BEACON_STATE_BATTERY_SAVE;
            transmit_beacon_flag = 1;
            transmit_fast_flag = 0;
        }
    }
    
    // Handle beacon transmission based on state machine
    if (transmit_beacon_flag) {
        Serial.print(F("[Beacon] Flag set, attempting transmission. Time since last: "));
        Serial.println(system_time_seconds - last_transmission_time);
        
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
                                      system_time_seconds, transmit_fast_flag);
        }

        if (!transmit_fast_flag) {
            transmit_beacon_flag = 0;
        }
    }

    if (system_time_seconds - time_since_last_callsign_tx >= CALLSIGN_TRANSMIT_INTERVAL_SEC && !transmit_fast_flag) {
        beacon_transmit_callsign(transmit_fast_flag);
        time_since_last_callsign_tx = system_time_seconds;
    }

#if IMU_FUSION_ENABLED
    /* Fused-packet cadence: FUSED_TX_INTERVAL_MS in LAUNCH/POST_LAUNCH for
     * smooth interpolated telemetry, once per GPS packet otherwise.  Kept
     * separate from the GPS-packet TX path so both streams coexist. */
    static uint32_t last_fused_tx_ms = 0;
    uint32_t now_ms = millis();
    bool active_phase = (beacon_state == BEACON_STATE_LAUNCH)
                     || (beacon_state == BEACON_STATE_POST_LAUNCH);
    uint32_t fused_interval_ms = active_phase ? FUSED_TX_INTERVAL_MS : 1000u;
    if (nav_is_valid() && (now_ms - last_fused_tx_ms >= fused_interval_ms)) {
        beacon_transmit_fused_data(system_time_seconds, transmit_fast_flag);
        last_fused_tx_ms = now_ms;
    }
#endif
}
