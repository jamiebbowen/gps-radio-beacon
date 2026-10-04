#include "include/flight_cadence.h"
#include "include/config.h"

uint32_t flight_cadence_beacon_interval_s(beacon_state_t state)
{
    switch (state) {
        case BEACON_STATE_TURN_ON:      return TURN_ON_HEARTBEAT_INTERVAL_SEC;
        case BEACON_STATE_LAUNCH:       return 0;   /* continuous */
        case BEACON_STATE_POST_LAUNCH:  return POST_LAUNCH_PACKET_INTERVAL_SEC;
        case BEACON_STATE_BATTERY_SAVE: return BATTERY_SAVE_INTERVAL_SEC;
        case BEACON_STATE_PRE_LAUNCH:
        default:                      return PRELAUNCH_HEARTBEAT_INTERVAL_SEC;
    }
}

uint32_t flight_cadence_fused_interval_ms(beacon_state_t state)
{
    switch (state) {
        case BEACON_STATE_LAUNCH:
        case BEACON_STATE_POST_LAUNCH:  return FUSED_TX_INTERVAL_MS;
        case BEACON_STATE_BATTERY_SAVE: return FUSED_TX_INTERVAL_IDLE_MS;
        case BEACON_STATE_TURN_ON:
        case BEACON_STATE_PRE_LAUNCH:
        default:                        return 0;   /* pad: stream off */
    }
}

uint32_t flight_cadence_heartbeat_interval_s(beacon_state_t state)
{
    switch (state) {
        case BEACON_STATE_TURN_ON:      return TURN_ON_HEARTBEAT_INTERVAL_SEC;
        case BEACON_STATE_PRE_LAUNCH:   return PRELAUNCH_HEARTBEAT_INTERVAL_SEC;
        default:                        return HEARTBEAT_INTERVAL_SEC;
    }
}

uint32_t flight_cadence_gps_audit_interval_s(beacon_state_t state)
{
    return (state == BEACON_STATE_PRE_LAUNCH)
               ? PRELAUNCH_GPS_AUDIT_INTERVAL_SEC
               : 0;
}

uint32_t flight_cadence_callsign_interval_s(void)
{
    return CALLSIGN_TRANSMIT_INTERVAL_SEC;
}

int flight_cadence_should_leave_turn_on(uint32_t uptime_s)
{
    return uptime_s >= TURN_ON_DURATION_SEC;
}

int flight_cadence_should_leave_launch(uint32_t time_since_launch_s)
{
    return time_since_launch_s >= POST_LAUNCH_DURATION_SEC;
}

int flight_cadence_should_leave_post_launch(uint32_t time_in_post_launch_s)
{
    return time_in_post_launch_s >= POST_LAUNCH_RECOVERY_DURATION_SEC;
}
