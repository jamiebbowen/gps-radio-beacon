#ifndef CONFIG_H
#define CONFIG_H

/**
 * Configuration Header for GPS Radio Beacon
 * =========================================
 * 
 * This file contains compile-time configuration options for the GPS radio beacon.
 * Toggle TESTING_MODE to switch between production and testing configurations.
 */

#define LAUNCH_DEBOUNCE_TIME_MS         200   // 200ms debounce time for launch detection

// Set to 1 for testing mode, 0 for production mode
#define TESTING_MODE 0

#if TESTING_MODE
    // Testing Configuration - Fast intervals for development/testing
    #define PRE_LAUNCH_INTERVAL_SEC         30    // 30 seconds between transmissions in pre-launch
    #define POST_LAUNCH_DURATION_SEC        10    // 10 seconds duration in launch state
    #define POST_LAUNCH_RECOVERY_DURATION_SEC 1200  // 20 minutes duration in post-launch state
    #define BATTERY_SAVE_INTERVAL_SEC       30    // 30 seconds between transmissions in battery save
    #define CALLSIGN_TRANSMIT_INTERVAL_SEC  30    // 30 seconds between callsign transmissions
    
    // Debug features enabled in testing mode
    #define DEBUG_OUTPUT_ENABLED        1     // Enable debug output
    #define INCLUDE_CARRIAGE_RETURNS    1     // Include \r\n for terminal debugging
    
#else
    // Production Configuration - Conservative intervals for flight
    #define PRE_LAUNCH_INTERVAL_SEC         5    // 30 seconds between transmissions in pre-launch
    #define POST_LAUNCH_DURATION_SEC        1   // 1 second duration in launch state
    #define POST_LAUNCH_RECOVERY_DURATION_SEC 600 // 10 minutes duration in post-launch state
    #define BATTERY_SAVE_INTERVAL_SEC       60    // 60 seconds between transmissions in battery save
    #define CALLSIGN_TRANSMIT_INTERVAL_SEC  300   // 5 minutes between callsign transmissions
    
    // Debug features disabled in production mode
    #define DEBUG_OUTPUT_ENABLED        0     // Disable debug output
    #define INCLUDE_CARRIAGE_RETURNS    0     // No \r\n in production (RF efficiency)
    
#endif

#endif // CONFIG_H
