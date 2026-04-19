#include "include/mpu_config.h"
#include "include/packet_format.h"
#include <Arduino.h>
#include "include/config.h"
#include "include/radio.h"
#include "include/gps.h"
#include "include/uart.h"
#include "include/launch_detect.h"
#include "include/beacon.h"

/**
 * Beacon State Machine Documentation
 * =================================
 * 
 * The beacon operates in four distinct states optimized for rocketry applications:
 * 
 * 1. PRE_LAUNCH (Initial State):
 *    - Purpose: Conserve power while on the launch pad
 *    - Behavior: Transmit GPS data every 30 seconds
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Transition: Moves to LAUNCH when launch detection switch is triggered
 * 
 * 2. LAUNCH (Critical Phase):
 *    - Purpose: Maximum transmission rate during flight for recovery
 *    - Behavior: Continuous transmission (as fast as possible)
 *    - Data Format: Fast GPS packet (lat, lon, alt only) for speed
 *    - Duration: 3 minutes (180s) after launch detection
 *    - Transition: Moves to POST_LAUNCH after 3 minutes
 * 
 * 3. POST_LAUNCH (Recovery Mode):
 *    - Purpose: High-frequency recovery transmissions
 *    - Behavior: Continuous transmission for maximum recovery chances
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Duration: 10 minutes (600s), then transitions to BATTERY_SAVE
 *    - Entry: From LAUNCH after 3 minutes
 * 
 * 4. BATTERY_SAVE (Extended Recovery):
 *    - Purpose: Conserve battery for extended recovery operations
 *    - Behavior: Transmit full GPS packet every 30 seconds
 *    - Data Format: Full GPS packet (lat, lon, alt, satellites)
 *    - Duration: Indefinite (until power exhaustion)
 *    - Entry: From POST_LAUNCH after 10 minutes
 * 
 * Additional Features:
 * - Launch detection can trigger from ANY state, resetting to LAUNCH
 * - Callsign transmission: Configurable callsign every 5 minutes (FCC compliance)
 * - Launch detection: 200ms debounce on PA1 switch to ground
 * - Watchdog protection: Regular resets prevent system lockup
 * - GPS configuration: Optimized NMEA sentences for minimal data
 */
typedef enum {
    BEACON_STATE_PRE_LAUNCH,      // Pre-launch: 30s intervals, full packets
    BEACON_STATE_LAUNCH,          // Launch: continuous fast packets for 3min
    BEACON_STATE_POST_LAUNCH,     // Post-launch: continuous full packets for 10min
    BEACON_STATE_BATTERY_SAVE     // Extended recovery: 30s intervals, full packets
} beacon_state_t;

// Beacon state machine variables
volatile beacon_state_t beacon_state = BEACON_STATE_PRE_LAUNCH;
volatile uint8_t transmit_beacon_flag = 1;
volatile uint8_t transmit_callsign_flag = 1;
volatile uint8_t transmit_fast_flag = 0;
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

// Timer setup for 1ms interrupts
void timer_init(void) {
    // Use Arduino's built-in timer for 1ms interrupts
    // We'll call timer_isr_handler() from loop() instead of using hardware timer
    // This is simpler and sufficient for this application
}

void setup() {
    // Initialize USB Serial for debugging (optional)
    Serial.begin(115200);
    
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
    
    delay(1000);  // Let everything stabilize

    beacon_transmit_callsign(transmit_beacon_flag);
    time_since_last_callsign_tx = system_time_seconds;

    // Poll GPS for data
    gps_poll_rx();
}

void loop() {
    // Update timer (simulate 1ms interrupt) - catch up if we missed milliseconds
    static uint32_t last_millis = 0;
    uint32_t current_millis = millis();
    uint32_t elapsed_ms = current_millis - last_millis;
    
    // Call timer handler for each elapsed millisecond
    for (uint32_t i = 0; i < elapsed_ms; i++) {
        timer_isr_handler();
    }
    last_millis = current_millis;
    
    // Update launch detection system
    launch_detect_update(system_time_seconds, ms_counter);
    
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
        }

        if (!transmit_fast_flag) {
            transmit_beacon_flag = 0;
        }
    }

    if (system_time_seconds - time_since_last_callsign_tx >= CALLSIGN_TRANSMIT_INTERVAL_SEC && !transmit_fast_flag) {
        beacon_transmit_callsign(transmit_beacon_flag);
        time_since_last_callsign_tx = system_time_seconds;
    }
}
