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
#include <stdint.h>

/* RF status codes */
#define RF_OK       0
#define RF_ERROR    1
#define RF_TIMEOUT  2
#define RF_BUSY     3

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

#ifdef __cplusplus
}
#endif

#endif /* __RF_RECEIVER_H */
