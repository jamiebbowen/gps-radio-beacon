/**
 * @file rf_parser.h
 * @brief RF packet parser header for GPS radio beacon (LoRa)
 * @note LoRa handles packet framing and CRC validation automatically
 */

#ifndef __RF_PARSER_H
#define __RF_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gps.h"
#include <stdint.h>

/* Parser Constants */
#define RF_PARSER_BUFFER_SIZE   128
#define RF_PARSER_MAX_CALLSIGN_LEN 16
#define RF_PARSER_OK            0
#define RF_PARSER_ERROR         1

/**
 * @brief Initialize RF parser
 * @retval Status code
 */
uint8_t RF_Parser_Init(void);

/**
 * @brief Parse ASCII packet into GPS data structure
 * @param packet ASCII packet string to parse
 * @retval Status code (RF_PARSER_OK if successful, RF_PARSER_ERROR otherwise)
 */
uint8_t RF_Parser_ParseAsciiPacket(const char *packet);

/**
 * @brief Parse binary GPS packet
 * @param data Pointer to binary packet data
 * @param length Length of packet in bytes
 * @retval Status code (RF_PARSER_OK if successful, RF_PARSER_ERROR otherwise)
 */
uint8_t RF_Parser_ParseBinaryPacket(const uint8_t *data, uint16_t length);

/**
 * @brief Get parsed data from the last valid packet
 * @param gps_data Pointer to GPS_Data structure to fill
 * @param callsign Buffer to store callsign
 * @param callsign_size Size of callsign buffer
 * @param checksum Pointer to store checksum (deprecated, unused with LoRa)
 * @retval Status (1 if data available, 0 if not)
 */
uint8_t RF_Parser_GetParsedData(GPS_Data *gps_data, char *callsign, uint16_t callsign_size, uint8_t *checksum);

/**
 * @brief Get checksum error count (deprecated - LoRa has built-in CRC)
 * @param checksum_errors Pointer to store checksum error count (always returns 0)
 * @retval None
 */
void RF_Parser_GetChecksumErrors(uint16_t *checksum_errors);

/**
 * @brief Get the last valid packet from the parser
 * @param buffer Buffer to store the last valid packet
 * @param max_len Maximum length of the buffer
 * @retval Length of the packet stored in the buffer
 */
uint16_t RF_Parser_GetLastValidPacket(char *buffer, uint16_t max_len);

/**
 * @brief Reset parser state
 * @retval None
 */
void RF_Parser_Reset(void);

/**
 * @brief Get diagnostics from the parser
 * @param attempts Pointer to store number of attempts
 * @param successes Pointer to store number of successes
 * @param failures Pointer to store number of failures
 * @retval None
 */
void RF_Parser_GetDiagnostics(uint32_t *attempts, uint32_t *successes, uint32_t *failures);
/**
 * @brief Get detailed failure diagnostics
 * @param null_packet Pointer to store null packet failures
 * @param insufficient_fields Pointer to store insufficient fields failures
 * @param checksum_invalid Pointer to store checksum failures (deprecated, always 0)
 * @param marker_not_found Pointer to store marker failures (deprecated, always 0)
 * @param callsign_too_short Pointer to store callsign failures (deprecated, always 0)
 * @retval None
 */
void RF_Parser_GetDetailedFailures(uint32_t *null_packet, uint32_t *insufficient_fields, 
                                   uint32_t *checksum_invalid, uint32_t *marker_not_found,
                                   uint32_t *callsign_too_short);

/**
 * @brief Get the last raw packet from the parser
 * @param buffer Buffer to store the last raw packet
 * @param max_len Maximum length of the buffer
 * @retval Length of the packet stored in the buffer
 */
uint16_t RF_Parser_GetLastRawPacket(char *buffer, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __RF_PARSER_H */
