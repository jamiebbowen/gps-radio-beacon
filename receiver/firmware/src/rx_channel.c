/**
 * @file rx_channel.c
 * @brief Channel selection and its breadcrumbs (see rx_channel.h)
 */
#include "rx_channel.h"
#include "rf_receiver.h"
#include "sd_card.h"
#include "lora.h"        /* LORA_CHANNEL_FREQ_MHZ */
#include <stdio.h>

void RxChannel_StartScan(uint8_t sd_ok)
{
  /* A beacon already in its quiet pad sends one packet a minute, which the
   * 62 s first dwell on the remembered channel outwaits (a 6.5 s-per-
   * channel lap catches it ~1 time in 8). */
  uint8_t last_ch = 0;
  if (sd_ok && SD_Card_LoadLastChannel(&last_ch) == SD_CARD_OK) {
    RF_Receiver_StartScanFrom(last_ch);
  } else {
    RF_Receiver_StartScan();
  }
}

uint8_t RxChannel_ManualStep(BeaconTrack_t *beacon, uint8_t sd_ok)
{
  /* A manual pick overrides the boot scan */
  RF_Receiver_StopScan();
  uint8_t before = RF_Receiver_GetChannel();
  uint8_t after = RF_Receiver_NextChannel();
  if (after == before) return 0;

  /* Everything received so far belongs to the previous rocket */
  BeaconTrack_NewContext(beacon);
  if (sd_ok) {
    char ch_msg[48];
    snprintf(ch_msg, sizeof(ch_msg), "RF manual CH%u %.2fMHz",
             (unsigned)after, LORA_CHANNEL_FREQ_MHZ(after));
    SD_Card_LogEvent(ch_msg);
    (void)SD_Card_SaveLastChannel(after);   /* next boot scans here first */
  }
  return 1;
}

uint8_t RxChannel_ServiceScan(uint8_t sd_ok)
{
  if (!RF_Receiver_ScanUpdate()) return 0;
  if (sd_ok) {
    char ch_msg[48];
    uint8_t locked_ch = RF_Receiver_GetChannel();
    snprintf(ch_msg, sizeof(ch_msg), "RF scan locked CH%u %.2fMHz",
             (unsigned)locked_ch, LORA_CHANNEL_FREQ_MHZ(locked_ch));
    /* A lock proves a beacon is on the air; the lazily-created log file
     * may not exist yet (no NAV row before the first position packet) */
    SD_Card_EnsureLogFile();
    SD_Card_LogEvent(ch_msg);
    (void)SD_Card_SaveLastChannel(locked_ch);   /* next boot scans here first */
  }
  return 1;
}
