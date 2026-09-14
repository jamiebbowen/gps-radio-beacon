/**
 ******************************************************************************
 * @file           : preflight_mode.c
 * @brief          : Pre-flight go/no-go checklist display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "display_modes/preflight_mode.h"
#include "display.h"
#include "packet_format.h"   /* HB_GPS_* */
#include "rf_receiver.h"     /* RF_Receiver_TestingBuildSuspect */
#include "rf_parser.h"       /* RF_PARSER_MAX_CALLSIGN_LEN */
#include <stdio.h>

/**
 * @brief Display the pre-flight checklist
 *
 * Layout (8 rows x 21 cols):
 *   PRE-FLIGHT CHECK
 *   TX LINK  OK 3s -87dBm
 *   TX GPS   OK 10sat fix1      / CHK sats=0 ACQ
 *   RX GPS   OK 8sat            / CHK no fix
 *   COMPASS  OK                 / CHK
 *   SD CARD  OK                 / WARN no logging
 *   TX IMU   OK                 / CHK dead!  (advisory)
 *   ** READY TO FLY **          / NOT READY
 */
void DisplayMode_Preflight(uint8_t link_ok, uint32_t link_age_s, int16_t rssi,
                           uint8_t tx_fix, uint8_t tx_sats, uint8_t tx_hb_state,
                           uint8_t tx_sensor_degraded,
                           uint8_t rx_fix_ok, uint8_t rx_sats,
                           uint8_t compass_ok, uint8_t sd_ok)
{
    /* Sized for worst-case formatted expansion (sats up to 255, fix up to
     * 255), not just visible columns - display clips to its cell width. */
    char buf[32];

    Display_DrawTextRowCol(0, 4, "PRE-FLIGHT CHECK");

    /* TX link. A healthy beacon within metres on the rail reads way above
     * -75 dBm; anything lower on the pad means antenna/IPEX/polarization
     * trouble - exactly the failure you want to catch BEFORE flight, when
     * the telemetry budget is spent on trees, not free space. Advisory:
     * the verdict still flies on link_ok, the row just refuses to say OK.
     * A TESTING_MODE flash replaces the row with the STARRED callsign
     * (its ~30 s ID cadence tripped the detector): unmissable IF you're
     * about to fly a bench build, harmless if it's a deliberate test. */
    if (link_ok && RF_Receiver_TestingBuildSuspect()) {
        GPS_Data scratch;
        char cs[RF_PARSER_MAX_CALLSIGN_LEN] = "";
        RF_Receiver_GetParsedData(&scratch, cs, sizeof(cs), NULL);
        if (cs[0]) {
            /* 21-col budget: "TST " + "*" + callsign(max 15) + "*" = 21 */
            snprintf(buf, sizeof(buf), "TST *%.15s*", cs);
        } else {
            snprintf(buf, sizeof(buf), "TX LINK  TST BUILD?");
        }
    } else if (link_ok && rssi < -75) {
        snprintf(buf, sizeof(buf), "TX LINK  WEAK %ddBm",
                 (int)rssi);
    } else if (link_ok) {
        snprintf(buf, sizeof(buf), "TX LINK  OK %lus %ddBm",
                 (unsigned long)link_age_s, (int)rssi);
    } else {
        snprintf(buf, sizeof(buf), "TX LINK  CHK no pkts");
    }
    Display_DrawTextRowCol(1, 0, buf);

    /* TX GPS: fix + sats from the last position packet; when there's no fix
     * the heartbeat health field says WHY (wiring vs still acquiring). */
    uint8_t tx_gps_ok = (tx_fix >= 1 && tx_sats >= 4);
    if (tx_gps_ok) {
        snprintf(buf, sizeof(buf), "TX GPS   OK %usat fix%u",
                 (unsigned)tx_sats, (unsigned)tx_fix);
    } else if (tx_hb_state == HB_GPS_NO_DATA) {
        snprintf(buf, sizeof(buf), "TX GPS   CHK wiring!");
    } else if (tx_hb_state == HB_GPS_NO_NMEA) {
        snprintf(buf, sizeof(buf), "TX GPS   CHK garbled");
    } else if (tx_hb_state == HB_GPS_ACQUIRING) {
        snprintf(buf, sizeof(buf), "TX GPS   ACQ %usat", (unsigned)tx_sats);
    } else {
        snprintf(buf, sizeof(buf), "TX GPS   CHK %usat f%u",
                 (unsigned)tx_sats, (unsigned)tx_fix);
    }
    Display_DrawTextRowCol(2, 0, buf);

    /* RX GPS */
    if (rx_fix_ok) {
        snprintf(buf, sizeof(buf), "RX GPS   OK %usat", (unsigned)rx_sats);
    } else {
        snprintf(buf, sizeof(buf), "RX GPS   CHK no fix");
    }
    Display_DrawTextRowCol(3, 0, buf);

    /* Compass */
    Display_DrawTextRowCol(4, 0, compass_ok ? "COMPASS  OK" : "COMPASS  CHK");

    /* SD card (advisory: receiver works without it, but no track log) */
    Display_DrawTextRowCol(5, 0, sd_ok ? "SD CARD  OK" : "SD CARD  WARN nolog");

    /* TX IMU (advisory: a dead BNO085 costs fused smoothing only - the
     * beacon still flies on raw GPS with the altitude-climb fallback.
     * Carried on the fused stream as FUSED_FLAG_SENSOR_DEGRADED; goes
     * "OK" implicitly for beacons running pre-fused firmware.) */
    Display_DrawTextRowCol(6, 0, tx_sensor_degraded ? "TX IMU   CHK dead!"
                                                    : "TX IMU   OK");

    /* Verdict */
    if (link_ok && tx_gps_ok && rx_fix_ok && compass_ok) {
        Display_DrawTextRowCol(7, 2, "** READY TO FLY **");
    } else {
        Display_DrawTextRowCol(7, 3, "NOT READY - CHK");
    }
}
