#include "include/flight_cadence.h"
#include "include/config.h"

uint32_t flight_cadence_beacon_interval_s(beacon_state_t state)
{
    switch (state) {
        case BEACON_STATE_LAUNCH:       return 0;   /* continuous */
        case BEACON_STATE_POST_LAUNCH:  return POST_LAUNCH_PACKET_INTERVAL_SEC;
        case BEACON_STATE_BATTERY_SAVE: return BATTERY_SAVE_INTERVAL_SEC;
        case BEACON_STATE_PRE_LAUNCH:
        default:                      return PRE_LAUNCH_INTERVAL_SEC;
    }
}

uint32_t flight_cadence_fused_interval_ms(beacon_state_t state)
{
    return (state == BEACON_STATE_LAUNCH || state == BEACON_STATE_POST_LAUNCH)
               ? FUSED_TX_INTERVAL_MS
               : FUSED_TX_INTERVAL_IDLE_MS;
}

uint32_t flight_cadence_callsign_interval_s(void)
{
    return CALLSIGN_TRANSMIT_INTERVAL_SEC;
}

int flight_cadence_should_leave_launch(uint32_t time_since_launch_s)
{
    return time_since_launch_s >= POST_LAUNCH_DURATION_SEC;
}

int flight_cadence_should_leave_post_launch(uint32_t time_in_post_launch_s)
{
    return time_in_post_launch_s >= POST_LAUNCH_RECOVERY_DURATION_SEC;
}
