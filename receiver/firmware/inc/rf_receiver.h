/**
 * @file rf_receiver.h
 * @brief RF receiver driver header for E22-400M33S LoRa module (SX1268)
 */

#ifndef __RF_RECEIVER_H
#define __RF_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gps.h"
#include "packet_format.h"  /* HeartbeatPacket_t */
#include <stdint.h>

/* RF status codes */
#define RF_OK       0
#define RF_ERROR    1
#define RF_TIMEOUT  2
#define RF_BUSY     3

/* MCU-side peripheral init failures (distinct from the LoRa chip's 0xB*
 * codes so the boot screen identifies which side of the SPI bus is dead) */
#define RF_ERR_SPI_INIT     0xE1
#define RF_ERR_DMA_TX_INIT  0xE2
#define RF_ERR_DMA_RX_INIT  0xE3

/* RF data quality thresholds */
#define RF_DATA_STALE_TIMEOUT_MS  30000  /* 30 seconds - mark data as stale */
#define RF_MIN_RSSI_DBM          -120    /* Minimum acceptable RSSI in dBm */
#define RF_MIN_SNR_DB            -10     /* Minimum acceptable SNR in dB */

/* Timing calibration */
#define RF_BAUD_FUDGE_FACTOR_DEFAULT 1000  /* Default = 1.000 (no adjustment) */
#define RF_BAUD_FUDGE_MIN            950   /* Min = 0.950 (-5%) */
#define RF_BAUD_FUDGE_MAX            1050  /* Max = 1.050 (+5%) */

/* RF packet structure */
typedef struct {
  uint8_t packet_id;      /* Packet ID (incremented for each packet) */
  GPS_Data gps_data;      /* GPS data from transmitter */
  uint16_t battery_mv;    /* Battery voltage in millivolts */
  uint16_t checksum;      /* Packet checksum */
} RF_Packet;

/* Function prototypes */
uint8_t RF_Receiver_Init(void);
uint8_t RF_Receiver_DataAvailable(void);
uint8_t RF_Receiver_GetGPSData(GPS_Data *gps_data);
void RF_Receiver_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void RF_Receiver_GetDiagnostics(uint32_t *bytes_received, uint8_t *header_matches, uint8_t *last_bytes);
void RF_Receiver_GetExtendedDiagnostics(uint16_t *checksum_errors);
uint32_t RF_Receiver_GetLoRaPacketCount(void);
void RF_Receiver_GetIRQDiagnostics(uint32_t *irq_checks, uint16_t *last_irq, uint8_t *device_mode, uint8_t *spi_test, uint8_t *busy_state);
void RF_Receiver_GetAsciiBuffer(char *buffer, uint16_t max_len);
uint16_t RF_Receiver_GetLastPacketASCII(char *buffer, uint16_t max_len);
uint8_t RF_Receiver_GetParsedData(GPS_Data *gps_data, char *callsign, uint16_t callsign_size, uint8_t *checksum);

/* Multi-rocket channel selection (see channel plan in lora.h).
 * Switching channels drops any pending packet and resets the last-packet
 * timestamp, since data on the old channel belongs to a different rocket. */
uint8_t RF_Receiver_SetChannel(uint8_t channel);
uint8_t RF_Receiver_GetChannel(void);
uint8_t RF_Receiver_NextChannel(void);  /* Cycle to next channel; returns the now-active channel */

/* No-fix heartbeat from the beacon (one-shot; 1 = new heartbeat copied) */
uint8_t RF_Receiver_GetHeartbeat(HeartbeatPacket_t *hb);

/* Non-consuming heartbeat access for display (1 = a heartbeat has been
 * heard on the current channel; hb/age_ms filled if non-NULL) */
uint8_t RF_Receiver_GetLastHeartbeat(HeartbeatPacket_t *hb, uint32_t *age_ms);

/* Flight-window forensics: inertial trace (one-shot; 1 = new trace) and
 * certified T0 declaration (one-shot; 1 = a new T0 was packaged). The IMU
 * row goes straight to the SD log from main.c; T0 fires one event row. */
uint8_t RF_Receiver_GetImuTrace(ImuTracePacket_t *imu);
uint8_t RF_Receiver_GetLaunchT0(LaunchT0Packet_t *t0);

/* Certified flight events: apogee / drogue / main / landed one-shots and
 * anomaly (re-)announcements. One-shot read: 1 = a new event packet. main.c
 * logs every one to the SD card (events repeat by design, so a log row per
 * packet is correct). */
uint8_t RF_Receiver_GetFlightEvent(FlightEventPacket_t *evt);

/* Running flight maxima (recurring while airborne; each copy is a full
 * snapshot so every row stands alone) and the beacon's boot-time firmware
 * identity. One-shot reads; main.c logs both. */
uint8_t RF_Receiver_GetMaxima(MaximaPacket_t *mx);
uint8_t RF_Receiver_GetHello(HelloPacket_t *hello);

/* Two-way scaffolding (v2 board radios; ignored on v1): operator triggers
 * a PING from a test-mode button press; the received one-shot ACK state is
 * readable to the caller and cleared with ConsumeAckFlag. Packets dropped
 * to foreign beacons never mark the latch. */
uint8_t RF_Receiver_SendCommand(uint8_t cmd_code, uint8_t param);
uint8_t RF_Receiver_GetLastAck(AckPacket_t *ack, uint32_t *age_ms);
uint8_t RF_Receiver_ConsumeAckFlag(void);

/* Ambient noise-floor monitor. The floor is the 25th percentile of ~1 Hz
 * GetRssiInst samples taken while the radio idles in continuous RX; the
 * alert engages at RF_NOISE_ALERT_DBM and clears at RF_NOISE_CLEAR_DBM.
 * A clean 433 MHz channel at BW62.5 reads around -113..-128 dBm; a floor
 * at -100 dBm or above eats ~20 dB of link budget. */
#define RF_NOISE_ALERT_DBM  -100  /* engage alert at/above this floor */
#define RF_NOISE_CLEAR_DBM  -105  /* release alert at/below this floor */
uint8_t RF_Receiver_GetNoiseFloor(int16_t *nf_dbm);
uint8_t RF_Receiver_NoiseAlert(void);
uint32_t RF_Receiver_GetWedgesRecovered(void);

/* In-link floor estimate (min inst-RSSI over the recent ~30 s): valid on a
 * live link where the alert estimate is parked quiet-gated. */
uint8_t RF_Receiver_GetLiveNoiseFloor(int16_t *nf_dbm);
/* Completed radio-deaf window after a wedge streak (one-shot; main.c logs
 * it so blackout reads can attribute silence to receiver vs beacon). */
uint8_t RF_Receiver_TakeWedgeReport(uint32_t *count, uint32_t *last_deaf_ms,
                                    uint32_t *total_deaf_ms);
/* Head bytes of the last CRC-failed frame (one-shot) so "garbage heard"
 * rows show whether it looked like our beacon dying at the margin or a
 * foreign network. */
uint8_t RF_Receiver_TakeCrcDump(uint8_t *out, uint8_t max_len, uint8_t *total_len);

/* Airframe binding / foreign-beacon filter. The receiver binds to the
 * first rocket_id heard on the tuned channel and drops V2 position packets
 * from any other airframe. The binding resets on a manual channel change;
 * it survives the auto re-scan (a beacon lost mid-flight doesn't change
 * identities). Legacy V1 packets carry no ID and always pass. */
uint8_t  RF_Receiver_GetBoundRocketId(void);   /* 0xFF = unbound */
uint32_t RF_Receiver_GetForeignDrops(void);

/* 1 = the tuned beacon ID's at a ~30 s cadence: it was flashed with
 * TESTING_MODE=1 (production IDs every 5 min). Preflight advisory. */
uint8_t RF_Receiver_TestingBuildSuspect(void);

/* Boot-time per-channel noise sweep: tunes every rocket channel, takes a
 * handful of GetRssiInst samples, and reports the per-channel median via
 * nf_dbm_out[LORA_CHANNEL_COUNT] (untouched entries are left at the
 * caller's init value; 1 is a good "invalid" sentinel). Discriminates
 * narrowband junk (one hot channel) from broadband/front-end overload
 * (everything lifted) - the distinction that decides whether the fix is
 * "move the beacon's channel" or "move the receiver". Radio is left on
 * the channel it started on; returns channels measured (0 on total
 * failure). The caller re-arms any in-flight channel scan afterwards. */
uint8_t RF_Receiver_NoiseSweep(int16_t *nf_dbm_out);
/** Tick at which the last sweep measured channel `ch` (for honest
 *  timestamping when the logger prints results after the sweep). */
uint32_t RF_Receiver_GetSweepTick(uint8_t ch);

/* Boot-time channel scan: hop channels until a CRC-valid packet is heard */
void    RF_Receiver_StartScan(void);
void    RF_Receiver_StopScan(void);     /* Cancel (e.g. manual channel pick) */
uint8_t RF_Receiver_IsScanning(void);
uint8_t RF_Receiver_ScanUpdate(void);   /* Poll from main loop; 1 = just locked */

/* Timing calibration functions */
void RF_Receiver_SetBaudFudgeFactor(uint16_t fudge_factor);
uint16_t RF_Receiver_GetBaudFudgeFactor(void);
void RF_Receiver_OutputCalibrationSignal(uint32_t duration_ms);

/* Signal quality and data age functions */
void RF_Receiver_GetSignalQuality(int16_t *rssi, int8_t *snr);
uint8_t RF_Receiver_IsDataStale(uint32_t current_time_ms);
uint32_t RF_Receiver_GetLastPacketTime(void);
uint8_t RF_Receiver_IsSignalQualityGood(void);

/* Packet loss diagnostics */
void RF_Receiver_GetPacketLossDiagnostics(uint32_t *irq_count, uint32_t *lora_packets, uint32_t *duplicates);
uint32_t RF_Receiver_GetCrcErrors(void);

#ifdef __cplusplus
}
#endif

#endif /* __RF_RECEIVER_H */
