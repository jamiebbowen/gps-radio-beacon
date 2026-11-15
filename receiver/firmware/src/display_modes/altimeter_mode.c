/**
 * @file altimeter_mode.c
 * @brief Commercial altimeters riding the beacon's maxima recap: per input
 *        the recognised format, the apogee and the charges it reported
 *        fired.
 */
#include "display_modes/altimeter_mode.h"
#include "display.h"
#include "rf_receiver.h"
#include <stdio.h>

void DisplayMode_Altimeter(void)
{
    static const char *jack[ALTIMETER_INPUTS] = { "J2", "J5", "J6" };
    static const char *fmt[4] = { "?", "SLCF", "RRC3", "EGG" };
    char buffer[32];
    MaximaAltPacket_t a;
    uint32_t age_ms = 0;

    Display_DrawTextRowCol(0, 0, "ALTIMETERS");
    if (!RF_Receiver_PeekAltimeter(&a, &age_ms)) {
        Display_DrawTextRowCol(2, 0, "None relayed yet");
        /* On the pad: what the beacon's heartbeat says it recognised */
        HeartbeatPacket_t hb;
        if (RF_Receiver_GetLastHeartbeat(&hb, &age_ms) && (hb.alt_inputs & 0x3F)) {
            snprintf(buffer, sizeof(buffer), "Pad, %lus ago%s",
                     (unsigned long)(age_ms / 1000u),
                     (hb.alt_inputs & HB_ALT_STALE) ? " (quiet)" : "");
            Display_DrawTextRowCol(3, 0, buffer);
            uint8_t prow = 4;
            for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
                uint8_t f = HB_ALT_FMT(hb.alt_inputs, i);
                if (f == 0) continue;
                snprintf(buffer, sizeof(buffer), "%s %s", jack[i], fmt[f]);
                Display_DrawTextRowCol(prow++, 0, buffer);
            }
        }
        return;
    }
    snprintf(buffer, sizeof(buffer), "Heard %lus ago", (unsigned long)(age_ms / 1000u));
    Display_DrawTextRowCol(1, 0, buffer);

    uint8_t row = 2;
    for (uint8_t i = 0; i < ALTIMETER_INPUTS; i++) {
        if (!(a.alt_inputs & (1u << i))) continue;
        const MaximaAltEntry_t *e = &a.alt[i];
        snprintf(buffer, sizeof(buffer), "%s %s%s", jack[i],
                 fmt[e->status & ALT_ST_FMT_MASK],
                 (e->status & ALT_ST_STALE) ? " old" : "");
        Display_DrawTextRowCol(row++, 0, buffer);
        snprintf(buffer, sizeof(buffer), " Apo %ldft %c%c%c",
                 (long)e->apogee_4ft * ALT_FT_PER_UNIT,
                 (e->status & ALT_ST_DROGUE) ? 'D' : '-',
                 (e->status & ALT_ST_MAIN) ? 'M' : '-',
                 (e->status & ALT_ST_AUX) ? 'A' : '-');
        Display_DrawTextRowCol(row++, 0, buffer);
    }
}
