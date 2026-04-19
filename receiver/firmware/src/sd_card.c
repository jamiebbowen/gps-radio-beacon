/**
 ******************************************************************************
 * @file           : sd_card.c
 * @brief          : SD card storage module implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sd_card.h"
#include "main.h"
#include "display.h"
#include "ff.h"
#include "diskio.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* External variables --------------------------------------------------------*/
extern uint8_t SD_Type;

/* Private defines -----------------------------------------------------------*/
#define SD_CARD_LOG_BUFFER_SIZE     256
#define SD_CARD_MAX_RETRIES         3

/* Private variables ---------------------------------------------------------*/
static FATFS fs;
static FIL log_file;
static uint8_t sd_initialized = 0;
static uint8_t fatfs_mounted = 0;
static SD_Card_Info sd_info;
static char SDPath[4]; /* SD logical drive path */
static char log_buffer[SD_CARD_LOG_BUFFER_SIZE];
static uint32_t log_sequence = 0;

/* Private function prototypes -----------------------------------------------*/
static SD_Card_Status SD_Card_WriteLogEntry(const char *entry);
static void SD_Card_UpdateInfo(void);

/**
 * @brief Initialize SD card module
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Init(void)
{
    FRESULT fres;
    
    /* Initialize SD card info structure */
    memset(&sd_info, 0, sizeof(SD_Card_Info));
    
    /* Check if SD card is physically present */
    if (!SD_Card_IsPresent()) {
        sd_info.is_present = 0;
        sd_info.is_mounted = 0;
        sd_info.is_initialized = 0;
        return SD_CARD_NOT_PRESENT;
    }
    
    sd_info.is_present = 1;
    
    /* Link the SD card driver to FATFS */
    if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
        sd_info.is_mounted = 0;
        sd_info.is_initialized = 0;
        sd_info.write_errors = 99; /* Driver link error code */
        return SD_CARD_ERROR;
    }
    
    /* Try to mount the filesystem (up to 3 attempts - card may need settling time) */
    fres = FR_NOT_READY;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
            f_mount(NULL, "", 0); /* Unmount to reset FATFS state before retry */
            HAL_Delay(300);
        }
        fres = f_mount(&fs, "", 1);
        if (fres == FR_OK) break;
        sd_info.write_errors = fres; /* Store last FRESULT for display */
    }
    if (fres != FR_OK) {
        sd_info.is_mounted = 0;
        sd_info.is_initialized = 0;
        return SD_CARD_ERROR;
    }
    
    fatfs_mounted = 1;
    sd_info.is_mounted = 1;
    sd_info.is_initialized = 1;
    sd_initialized = 1;

    /* Update SD card information */
    SD_Card_UpdateInfo();
    
    return SD_CARD_OK;
}

/**
 * @brief Deinitialize SD card module
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_DeInit(void)
{
    /* Close any open files */
    f_close(&log_file);
    
    /* Unmount filesystem */
    if (fatfs_mounted) {
        f_mount(NULL, "", 0);
        fatfs_mounted = 0;
    }
    
    /* Reset info structure */
    memset(&sd_info, 0, sizeof(SD_Card_Info));
    sd_initialized = 0;
    
    return SD_CARD_OK;
}

/**
 * @brief Check if SD card is present
 * @retval true if present, false otherwise
 */
bool SD_Card_IsPresent(void)
{
    /* Read SD card detect pin (active low) */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN);
    
    /* Update info structure */
    sd_info.detect_pin_state = HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN);
    
    /* Get CMD0 response for debugging (stored in SD_Type temporarily) */
    extern uint8_t SD_Type;
    sd_info.cmd0_response = SD_Type;
    
    /* Debug: Store pin state for display */
    sd_info.detect_pin_state = (pin_state == GPIO_PIN_SET) ? 1 : 0;
    
    /* TEMPORARY DEBUG: Try bypassing detect pin - assume card is present */
    /* Comment out the line below once detect pin is working */
    return true;  /* Force detection for testing */
    
    /* Normal detection logic (currently disabled for debug) */
    /* return (pin_state == GPIO_PIN_RESET); */
}

/**
 * @brief Get SD card information
 * @param info Pointer to SD_Card_Info structure
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_GetInfo(SD_Card_Info *info)
{
    if (info == NULL) {
        return SD_CARD_ERROR;
    }
    
    /* Update current info */
    SD_Card_UpdateInfo();
    
    /* Copy info structure */
    memcpy(info, &sd_info, sizeof(SD_Card_Info));
    
    return SD_CARD_OK;
}

/**
 * @brief Create a new log file with timestamp
 * @param filename Buffer to store created filename
 * @param max_len Maximum length of filename buffer
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_CreateLogFile(char *filename, uint32_t max_len)
{
    FRESULT fres;
    
    if (filename == NULL || !sd_initialized || !fatfs_mounted) {
        return SD_CARD_ERROR;
    }
    
    /* Close any existing log file */
    f_close(&log_file);
    
    /* Generate filename with timestamp */
    uint32_t tick = HAL_GetTick();
    snprintf(filename, max_len, "L%07lX.TXT", (unsigned long)(tick & 0x0FFFFFFF));
    
    /* Create and open the new log file */
    fres = f_open(&log_file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        return SD_CARD_ERROR;
    }
    
    /* Write CSV header */
    const char *header = "Timestamp,Type,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,BaseLat,BaseLon,Distance_km,Bearing_deg,Heading_deg,RSSI_dBm\n";
    UINT bytes_written;
    fres = f_write(&log_file, header, strlen(header), &bytes_written);
    if (fres != FR_OK) {
        f_close(&log_file);
        return SD_CARD_ERROR;
    }
    
    /* Sync header immediately so the file is valid even if power is cut before data arrives */
    f_sync(&log_file);
    
    /* Update info */
    strncpy(sd_info.current_log_file, filename, SD_CARD_MAX_FILENAME - 1);
    sd_info.files_logged++;
    sd_info.bytes_written += bytes_written;
    
    return SD_CARD_OK;
}

/**
 * @brief Log GPS data to SD card
 * @param gps_data Pointer to GPS data structure
 * @param log_type Type of GPS log (local or remote)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogGPS(GPS_Data *gps_data, Log_Entry_Type log_type)
{
    if (gps_data == NULL || !sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    char timestamp[32];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));
    
    const char *type_str = (log_type == LOG_TYPE_GPS_LOCAL) ? "GPS_LOCAL" : "GPS_REMOTE";
    
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,%s,%.6f,%.6f,%.1f,%d,%d,%.1f\n",
             timestamp, type_str,
             gps_data->latitude, gps_data->longitude, gps_data->altitude,
             gps_data->fix, gps_data->satellites, gps_data->speed);
    
    return SD_Card_WriteLogEntry(log_buffer);
}

/**
 * @brief Log RF packet data to SD card
 * @param packet_data Pointer to packet data
 * @param packet_length Length of packet data
 * @param rssi Signal strength (if available)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogRFPacket(const char *packet_data, uint32_t packet_length, int16_t rssi)
{
    if (packet_data == NULL || !sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    char timestamp[32];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,RF_PACKET,%d,%s\n",
             timestamp, rssi, packet_data);
    
    return SD_Card_WriteLogEntry(log_buffer);
}

/**
 * @brief Log compass data to SD card
 * @param heading Compass heading in degrees
 * @param raw_x Raw magnetometer X value
 * @param raw_y Raw magnetometer Y value
 * @param raw_z Raw magnetometer Z value
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogCompass(float heading, int16_t raw_x, int16_t raw_y, int16_t raw_z)
{
    if (!sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    char timestamp[32];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,COMPASS,%.1f,%d,%d,%d\n",
             timestamp, heading, raw_x, raw_y, raw_z);
    
    return SD_Card_WriteLogEntry(log_buffer);
}

/**
 * @brief Log system event to SD card
 * @param event_message Event description string
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogEvent(const char *event_message)
{
    if (event_message == NULL || !sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    char timestamp[32];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,EVENT,%s\n",
             timestamp, event_message);
    
    return SD_Card_WriteLogEntry(log_buffer);
}

/**
 * @brief Log error message to SD card
 * @param error_message Error description string
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogError(const char *error_message)
{
    if (error_message == NULL || !sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    char timestamp[32];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,ERROR,%s\n",
             timestamp, error_message);
    
    return SD_Card_WriteLogEntry(log_buffer);
}

/**
 * @brief Flush any pending writes to SD card
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Flush(void)
{
    if (!sd_initialized) {
        return SD_CARD_ERROR;
    }
    
    /* For simulation, just return OK */
    return SD_CARD_OK;
}

/**
 * @brief Get current timestamp string for logging
 * @param timestamp_str Buffer to store timestamp
 * @param max_len Maximum length of timestamp buffer
 * @retval None
 */
void SD_Card_GetTimestamp(char *timestamp_str, uint32_t max_len)
{
    if (timestamp_str == NULL) {
        return;
    }
    
    uint32_t tick = HAL_GetTick();
    uint32_t seconds = tick / 1000;
    uint32_t milliseconds = tick % 1000;
    
    snprintf(timestamp_str, max_len, "%lu.%03lu", seconds, milliseconds);
}

/**
 * @brief Format SD card (WARNING: Erases all data)
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Format(void)
{
    FRESULT fres;
    BYTE work[FF_MAX_SS]; /* Work area for f_mkfs */
    
    if (!sd_info.is_present) {
        return SD_CARD_NOT_PRESENT;
    }
    
    /* Close any open files */
    f_close(&log_file);
    
    /* Unmount filesystem */
    if (fatfs_mounted) {
        f_mount(NULL, "", 0);
        fatfs_mounted = 0;
    }
    
    /* Format the drive */
    fres = f_mkfs("", FM_FAT32, 0, work, sizeof(work));
    if (fres != FR_OK) {
        return SD_CARD_ERROR;
    }
    
    /* Remount filesystem */
    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        return SD_CARD_ERROR;
    }
    
    fatfs_mounted = 1;
    sd_info.is_mounted = 1;
    
    /* Reset counters */
    sd_info.files_logged = 0;
    sd_info.bytes_written = 0;
    sd_info.write_errors = 0;
    memset(sd_info.current_log_file, 0, sizeof(sd_info.current_log_file));
    
    /* Update filesystem info */
    SD_Card_UpdateInfo();
    
    return SD_CARD_OK;
}

/**
 * @brief Log a navigation data point on each RF packet reception
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_LogNavigation(GPS_Data *beacon_gps, GPS_Data *base_gps,
                                     float distance_km, float bearing_deg,
                                     float heading_deg, int16_t rssi)
{
    if (!sd_initialized) {
        return SD_CARD_ERROR;
    }

    char timestamp[16];
    SD_Card_GetTimestamp(timestamp, sizeof(timestamp));

    /* Beacon fields — use 0 defaults when pointer is NULL */
    float b_lat = 0.0f, b_lon = 0.0f, b_alt = 0.0f;
    int   b_sats = 0;
    if (beacon_gps != NULL) {
        b_lat  = beacon_gps->latitude;
        b_lon  = beacon_gps->longitude;
        b_alt  = beacon_gps->altitude;
        b_sats = beacon_gps->satellites;
    }

    /* Base station fields */
    float s_lat = 0.0f, s_lon = 0.0f;
    if (base_gps != NULL) {
        s_lat = base_gps->latitude;
        s_lon = base_gps->longitude;
    }

    snprintf(log_buffer, sizeof(log_buffer),
             "%s,NAV,%.6f,%.6f,%.1f,%d,%.6f,%.6f,%.3f,%.1f,%.1f,%d\n",
             timestamp,
             b_lat, b_lon, b_alt, b_sats,
             s_lat, s_lon,
             distance_km, bearing_deg, heading_deg, (int)rssi);

    SD_Card_Status status = SD_Card_WriteLogEntry(log_buffer);

    /* Rate-limit f_sync to ~5s intervals. Post-launch the TX spams packets
     * at ~1.72s cadence; calling f_sync() every packet would block the main
     * loop for ~10-50ms each time and starve the display/RF tasks.
     * The write itself is buffered by FatFS so data still lands in RAM on
     * every packet; we just defer the flush-to-FAT operation. */
    static uint32_t last_sync_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_sync_ms >= 5000U) {
        f_sync(&log_file);
        last_sync_ms = now_ms;
    }

    return status;
}

/**
 * @brief Save last known beacon location to BEACON.TXT
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_SaveLastBeacon(float lat, float lon, float alt)
{
    if (!sd_initialized || !fatfs_mounted) {
        return SD_CARD_ERROR;
    }

    FIL beacon_file;
    FRESULT fres = f_open(&beacon_file, "BEACON.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        return SD_CARD_ERROR;
    }

    char buf[48];
    snprintf(buf, sizeof(buf), "%.7f,%.7f,%.1f\n", lat, lon, alt);

    UINT bw;
    fres = f_write(&beacon_file, buf, strlen(buf), &bw);
    f_sync(&beacon_file);
    f_close(&beacon_file);

    return (fres == FR_OK && bw == strlen(buf)) ? SD_CARD_OK : SD_CARD_ERROR;
}

/**
 * @brief Load last known beacon location from BEACON.TXT
 * @retval SD_CARD_OK if loaded, SD_CARD_ERROR if file missing or corrupt
 */
SD_Card_Status SD_Card_LoadLastBeacon(float *lat, float *lon, float *alt)
{
    if (lat == NULL || lon == NULL || alt == NULL) {
        return SD_CARD_ERROR;
    }
    if (!sd_initialized || !fatfs_mounted) {
        return SD_CARD_ERROR;
    }

    FIL beacon_file;
    FRESULT fres = f_open(&beacon_file, "BEACON.TXT", FA_READ);
    if (fres != FR_OK) {
        return SD_CARD_ERROR;  /* File doesn't exist yet */
    }

    char buf[48];
    UINT br;
    fres = f_read(&beacon_file, buf, sizeof(buf) - 1, &br);
    f_close(&beacon_file);

    if (fres != FR_OK || br == 0) {
        return SD_CARD_ERROR;
    }
    buf[br] = '\0';

    /* Parse "lat,lon,alt" */
    float parsed_lat, parsed_lon, parsed_alt;
    if (sscanf(buf, "%f,%f,%f", &parsed_lat, &parsed_lon, &parsed_alt) != 3) {
        return SD_CARD_ERROR;
    }

    /* Sanity check */
    if (parsed_lat < -90.0f || parsed_lat > 90.0f ||
        parsed_lon < -180.0f || parsed_lon > 180.0f) {
        return SD_CARD_ERROR;
    }

    *lat = parsed_lat;
    *lon = parsed_lon;
    *alt = parsed_alt;
    return SD_CARD_OK;
}

/* Private defines for compass calibration persistence */
#define COMPCAL_FILENAME    "COMPCAL.BIN"   /* 8.3 filename */
#define COMPCAL_MAGIC_0     0xB0
#define COMPCAL_MAGIC_1     0x55
#define COMPCAL_DATA_LEN    22              /* ACC(6)+MAG(6)+GYR(6)+radii(4) */
#define COMPCAL_FILE_LEN    25              /* magic(2) + data(22) + checksum(1) */

/**
 * @brief Save BNO055 calibration data to COMPCAL.BIN on the SD card
 * @param data  22-byte calibration buffer from Compass_GetCalibrationData()
 * @param len   Must be >= COMPCAL_DATA_LEN
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_SaveCompassCal(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < COMPCAL_DATA_LEN) return SD_CARD_ERROR;
    if (!sd_initialized || !fatfs_mounted)      return SD_CARD_ERROR;

    uint8_t buf[COMPCAL_FILE_LEN];
    buf[0] = COMPCAL_MAGIC_0;
    buf[1] = COMPCAL_MAGIC_1;
    memcpy(&buf[2], data, COMPCAL_DATA_LEN);
    uint8_t csum = 0;
    for (uint8_t i = 0; i < COMPCAL_DATA_LEN; i++) csum ^= data[i];
    buf[COMPCAL_FILE_LEN - 1] = csum;

    FIL f;
    FRESULT fres = f_open(&f, COMPCAL_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) return SD_CARD_ERROR;

    UINT bw;
    fres = f_write(&f, buf, COMPCAL_FILE_LEN, &bw);
    f_sync(&f);
    f_close(&f);

    return (fres == FR_OK && bw == COMPCAL_FILE_LEN) ? SD_CARD_OK : SD_CARD_ERROR;
}

/**
 * @brief Load BNO055 calibration data from COMPCAL.BIN
 * @param data  Output buffer of at least COMPCAL_DATA_LEN bytes
 * @param len   Must be >= COMPCAL_DATA_LEN
 * @retval SD_CARD_OK on success, SD_CARD_ERROR if file missing or corrupt
 */
SD_Card_Status SD_Card_LoadCompassCal(uint8_t *data, uint8_t len)
{
    if (data == NULL || len < COMPCAL_DATA_LEN) return SD_CARD_ERROR;
    if (!sd_initialized || !fatfs_mounted)      return SD_CARD_ERROR;

    FIL f;
    FRESULT fres = f_open(&f, COMPCAL_FILENAME, FA_READ);
    if (fres != FR_OK) return SD_CARD_ERROR;

    uint8_t buf[COMPCAL_FILE_LEN];
    UINT br;
    fres = f_read(&f, buf, COMPCAL_FILE_LEN, &br);
    f_close(&f);

    if (fres != FR_OK || br != COMPCAL_FILE_LEN)         return SD_CARD_ERROR;
    if (buf[0] != COMPCAL_MAGIC_0 || buf[1] != COMPCAL_MAGIC_1) return SD_CARD_ERROR;

    uint8_t csum = 0;
    for (uint8_t i = 0; i < COMPCAL_DATA_LEN; i++) csum ^= buf[2 + i];
    if (csum != buf[COMPCAL_FILE_LEN - 1]) return SD_CARD_ERROR;

    memcpy(data, &buf[2], COMPCAL_DATA_LEN);
    return SD_CARD_OK;
}

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief Deinitialize SD card interface
 * @retval SD_Card_Status
 */
SD_Card_Status SD_Card_Deinit(void)
{
    if (!sd_initialized) {
        return SD_CARD_OK; /* Already deinitialized */
    }
    
    /* Close any open files */
    f_close(&log_file);
    
    /* Unmount filesystem */
    f_mount(NULL, "", 0);
    fatfs_mounted = 0;
    sd_info.is_mounted = 0;
    
    return SD_CARD_OK;
}

/**
 * @brief Write log entry to SD card
 * @param entry Log entry string
 * @retval SD_Card_Status
 */
static SD_Card_Status SD_Card_WriteLogEntry(const char *entry)
{
    FRESULT fres;
    UINT bytes_written;
    
    if (entry == NULL || !sd_initialized || !fatfs_mounted) {
        return SD_CARD_ERROR;
    }
    
    /* Write entry to file */
    fres = f_write(&log_file, entry, strlen(entry), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(entry)) {
        sd_info.write_errors++;
        return SD_CARD_ERROR;
    }
    
    /* Don't sync - buffered writes prevent blocking during LoRa RX */
    /* f_sync(&log_file); */
    
    /* Update statistics */
    sd_info.bytes_written += bytes_written;
    log_sequence++;
    
    return SD_CARD_OK;
}

/* Self-test diagnostics - accessible via SD_Card_GetSelfTestError() */
static uint8_t self_test_step = 0;   /* which step failed (1-6) */
static uint8_t self_test_fres = 0;   /* FatFs FRESULT at failure */

/**
 * @brief Retrieve last self-test failure details
 * @param step  Output: step number (1=open_w, 2=write, 3=sync, 4=open_r, 5=read, 6=verify)
 * @param fres  Output: FatFs FRESULT at failure (0=FR_OK)
 */
void SD_Card_GetSelfTestError(uint8_t *step, uint8_t *fres)
{
    if (step) *step = self_test_step;
    if (fres) *fres = self_test_fres;
}

/**
 * @brief Write/read-back self-test - checks every step and records failure point
 * @retval SD_CARD_OK if pattern matches, SD_CARD_ERROR otherwise
 */
SD_Card_Status SD_Card_SelfTest(void)
{
    self_test_step = 0;
    self_test_fres = 0;

    if (!sd_initialized || !fatfs_mounted) {
        self_test_step = 1;
        self_test_fres = (uint8_t)FR_NOT_READY;
        return SD_CARD_ERROR;
    }

    static const char TEST_PATTERN[] = "SD_TEST:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\n";
    const uint32_t pattern_len = sizeof(TEST_PATTERN) - 1;
    static char readback[sizeof(TEST_PATTERN)];
    memset(readback, 0, sizeof(readback));

    static FIL f;
    UINT bw, br;
    FRESULT fres;

#define ST_SHOW(step_num) do { \
        char _st[16]; \
        Display_Clear(); \
        Display_DrawTextRowCol(3, 0, "SD Test"); \
        snprintf(_st, sizeof(_st), "Step %d", (step_num)); \
        Display_DrawTextRowCol(4, 0, _st); \
        Display_Update(); \
    } while(0)

    /* Step 1: open for write */
    ST_SHOW(1);
    self_test_step = 1;
    fres = f_open(&f, "SD_TEST.TMP", FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) { self_test_fres = (uint8_t)fres; return SD_CARD_ERROR; }

    /* Step 2: write pattern */
    ST_SHOW(2);
    self_test_step = 2;
    fres = f_write(&f, TEST_PATTERN, pattern_len, &bw);
    if (fres != FR_OK || bw != pattern_len) {
        f_close(&f);
        self_test_fres = (uint8_t)fres;
        return SD_CARD_ERROR;
    }

    /* Step 3: sync (flush to physical disk) */
    ST_SHOW(3);
    self_test_step = 3;
    fres = f_sync(&f);
    if (fres != FR_OK) { f_close(&f); self_test_fres = (uint8_t)fres; return SD_CARD_ERROR; }

    /* Step 3b: close */
    self_test_step = 33;
    {
        FRESULT cres = f_close(&f);
        if (cres != FR_OK) { self_test_fres = (uint8_t)cres; return SD_CARD_ERROR; }
    }

    /* Step 4: reopen for read */
    ST_SHOW(4);
    self_test_step = 4;
    fres = f_open(&f, "SD_TEST.TMP", FA_READ);
    if (fres != FR_OK) { self_test_fres = (uint8_t)fres; return SD_CARD_ERROR; }

    /* Step 5: read back */
    ST_SHOW(5);
    self_test_step = 5;
    fres = f_read(&f, readback, pattern_len, &br);
    f_close(&f);
    if (fres != FR_OK || br != pattern_len) {
        self_test_fres = (uint8_t)fres;
        return SD_CARD_ERROR;
    }

    /* Step 6: verify content */
    ST_SHOW(6);
    self_test_step = 6;
    if (memcmp(TEST_PATTERN, readback, pattern_len) != 0) {
        self_test_fres = readback[0];
        return SD_CARD_ERROR;
    }

    /* Step 7: delete test file */
    ST_SHOW(7);
    self_test_step = 7;
    f_unlink("SD_TEST.TMP");
    return SD_CARD_OK;

#undef ST_SHOW
}

/**
 * @brief Update SD card information
 * @retval None
 */
static void SD_Card_UpdateInfo(void)
{
    FATFS *pfs;
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT fres;
    
    if (!sd_initialized || !fatfs_mounted) {
        return;
    }
    
    /* Get volume information */
    fres = f_getfree("", &fre_clust, &pfs);
    if (fres == FR_OK) {
        tot_sect = (pfs->n_fatent - 2) * pfs->csize;
        fre_sect = fre_clust * pfs->csize;
        
        /* Convert sectors to MB: 1 MB = 2048 * 512-byte sectors.
         * Avoid tot_sect*512 which overflows uint32 for cards >= 8 GB. */
        sd_info.total_size_mb = tot_sect / 2048;
        sd_info.free_space_mb = fre_sect / 2048;
    }
}
