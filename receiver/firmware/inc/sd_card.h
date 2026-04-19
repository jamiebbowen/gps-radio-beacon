/**
 ******************************************************************************
 * @file           : sd_card.h
 * @brief          : SD card storage module header
 ******************************************************************************
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "gps.h"

/* Defines -------------------------------------------------------------------*/
#define SD_CARD_TIMEOUT         5000    /* 5 second timeout */
#define SD_CARD_MAX_FILENAME    32      /* Maximum filename length */
#define SD_CARD_BUFFER_SIZE     512     /* Buffer size for operations */

/* SD Card Status */
typedef enum {
    SD_CARD_OK = 0,
    SD_CARD_ERROR,
    SD_CARD_TIMEOUT_ERROR,
    SD_CARD_NOT_PRESENT,
    SD_CARD_WRITE_PROTECTED,
    SD_CARD_MOUNT_ERROR,
    SD_CARD_FILE_ERROR
} SD_Card_Status;

/* SD Card Information Structure */
typedef struct {
    uint8_t is_present;
    uint8_t is_initialized;
    uint8_t is_mounted;
    uint32_t total_size_mb;
    uint32_t free_space_mb;
    uint32_t files_logged;
    uint32_t bytes_written;
    uint32_t write_errors;
    uint8_t detect_pin_state;
    uint8_t cmd0_response;  /* For debugging - stores actual CMD0 response */
    char current_log_file[SD_CARD_MAX_FILENAME];
} SD_Card_Info;

/* Log Entry Types */
typedef enum {
    LOG_TYPE_GPS_LOCAL = 0,
    LOG_TYPE_GPS_REMOTE,
    LOG_TYPE_RF_PACKET,
    LOG_TYPE_COMPASS,
    LOG_TYPE_SYSTEM_EVENT,
    LOG_TYPE_ERROR
} Log_Entry_Type;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initialize SD card module
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Init(void);

/**
 * @brief Deinitialize SD card module
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_DeInit(void);

/**
 * @brief Check if SD card is present
 * @retval true if present, false otherwise
 */
bool SD_Card_IsPresent(void);

/**
 * @brief Get SD card information
 * @param info Pointer to SD_Card_Info structure
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_GetInfo(SD_Card_Info *info);

/**
 * @brief Create a new log file with timestamp
 * @param filename Buffer to store created filename
 * @param max_len Maximum length of filename buffer
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_CreateLogFile(char *filename, uint32_t max_len);

/**
 * @brief Log GPS data to SD card
 * @param gps_data Pointer to GPS data structure
 * @param log_type Type of GPS log (local or remote)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogGPS(GPS_Data *gps_data, Log_Entry_Type log_type);

/**
 * @brief Log RF packet data to SD card
 * @param packet_data Pointer to packet data
 * @param packet_length Length of packet data
 * @param rssi Signal strength (if available)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogRFPacket(const char *packet_data, uint32_t packet_length, int16_t rssi);

/**
 * @brief Log compass data to SD card
 * @param heading Compass heading in degrees
 * @param raw_x Raw magnetometer X value
 * @param raw_y Raw magnetometer Y value
 * @param raw_z Raw magnetometer Z value
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogCompass(float heading, int16_t raw_x, int16_t raw_y, int16_t raw_z);

/**
 * @brief Log system event to SD card
 * @param event_message Event description string
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogEvent(const char *event_message);

/**
 * @brief Log error message to SD card
 * @param error_message Error description string
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogError(const char *error_message);

/**
 * @brief Log a navigation data point on each RF packet reception
 * @param beacon_gps Pointer to remote (beacon) GPS data
 * @param base_gps Pointer to local (base station) GPS data
 * @param distance_km Distance to beacon in km
 * @param bearing_deg Absolute bearing to beacon (degrees)
 * @param heading_deg Current compass heading (degrees)
 * @param rssi RSSI of received packet in dBm
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogNavigation(GPS_Data *beacon_gps, GPS_Data *base_gps,
                                     float distance_km, float bearing_deg,
                                     float heading_deg, int16_t rssi);

/**
 * @brief Save last known beacon location to BEACON.TXT for persistence across power cycles
 * @param lat Beacon latitude in decimal degrees
 * @param lon Beacon longitude in decimal degrees
 * @param alt Beacon altitude in meters
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_SaveLastBeacon(float lat, float lon, float alt);

/**
 * @brief Load last known beacon location from BEACON.TXT
 * @param lat Output: beacon latitude in decimal degrees
 * @param lon Output: beacon longitude in decimal degrees
 * @param alt Output: beacon altitude in meters
 * @retval SD_CARD_OK if loaded successfully, SD_CARD_ERROR if file missing or corrupt
 */
SD_Card_Status SD_Card_LoadLastBeacon(float *lat, float *lon, float *alt);

/**
 * @brief Flush any pending writes to SD card
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Flush(void);

/**
 * @brief Get current timestamp string for logging
 * @param timestamp_str Buffer to store timestamp
 * @param max_len Maximum length of timestamp buffer
 * @retval None
 */
void SD_Card_GetTimestamp(char *timestamp_str, uint32_t max_len);

/**
 * @brief Format SD card (WARNING: Erases all data)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Format(void);

/**
 * @brief Write/read-back self-test: writes a known pattern, closes, reopens,
 *        verifies content, then deletes the test file.
 * @retval SD_CARD_OK if write and read-back match, SD_CARD_ERROR otherwise
 */
SD_Card_Status SD_Card_SelfTest(void);
void SD_Card_GetSelfTestError(uint8_t *step, uint8_t *fres);

/**
 * @brief Save BNO055 compass calibration to COMPCAL.BIN
 * @param data  22-byte buffer from Compass_GetCalibrationData()
 * @param len   Must be >= 22
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_SaveCompassCal(const uint8_t *data, uint8_t len);

/**
 * @brief Load BNO055 compass calibration from COMPCAL.BIN
 * @param data  Output buffer, at least 22 bytes
 * @param len   Must be >= 22
 * @retval SD_CARD_OK on success, SD_CARD_ERROR if file missing or corrupt
 */
SD_Card_Status SD_Card_LoadCompassCal(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
