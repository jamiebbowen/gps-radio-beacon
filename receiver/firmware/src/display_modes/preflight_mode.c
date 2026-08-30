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
 *   (blank)
 *   ** READY TO FLY **          / NOT READY
 */
void DisplayMode_Preflight(uint8_t link_ok, uint32_t link_age_s, int16_t rssi,
                           uint8_t tx_fix, uint8_t tx_sats, uint8_t tx_hb_state,
                           uint8_t rx_fix_ok, uint8_t rx_sats,
                           uint8_t compass_ok, uint8_t sd_ok)
{
    char buf[24];

    Display_DrawTextRowCol(0, 4, "PRE-FLIGHT CHECK");

    /* TX link */
    if (link_ok) {
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

    /* Verdict */
    if (link_ok && tx_gps_ok && rx_fix_ok && compass_ok) {
        Display_DrawTextRowCol(7, 2, "** READY TO FLY **");
    } else {
        Display_DrawTextRowCol(7, 3, "NOT READY - CHK");
    }
}
