/**
 ******************************************************************************
 * @file           : sd_card.c
 * @brief          : SD card storage module - LittleFS backend.
 *
 * Why LittleFS (replaces FatFS, Apr 2026):
 *   FAT32 has no notion of atomic commits. A sudden power loss between the
 *   data-block write and the FAT update can leave the filesystem in a state
 *   where the directory entry, FAT chain, and actual data are inconsistent
 *   - which is exactly the corruption this rocket repeatedly suffered.
 *
 *   LittleFS is a log-structured, copy-on-write filesystem designed for
 *   embedded flash / SD. Every lfs_file_sync writes a new commit block; the
 *   old one is only invalidated once the new one is durable. A power loss
 *   at any instant leaves the filesystem mountable at the last committed
 *   state - the partial write at the tail is simply discarded.
 *
 * Linux / macOS / Windows readability:
 *   LittleFS is NOT natively mountable. Use `tools/lfs_extract/` (host-side
 *   program) to copy files off the card onto a normal filesystem. Top-level
 *   Makefile target: `make extract DEV=/dev/sdX OUT=./flight_data`.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sd_card.h"
#include "main.h"
#include "display.h"
#include "lfs.h"
#include "lfs_sd_bd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define SD_CARD_LOG_BUFFER_SIZE     256

#define COMPCAL_FILENAME    "COMPCAL.BIN"
#define COMPCAL_MAGIC_0     0xB0
#define COMPCAL_MAGIC_1     0x55
#define COMPCAL_DATA_LEN    22
#define COMPCAL_FILE_LEN    25

#define BEACON_FILENAME     "BEACON.TXT"

/* Private variables ---------------------------------------------------------*/
static lfs_t         lfs;
static lfs_file_t    log_file;
static uint8_t       sd_initialized = 0;
static uint8_t       lfs_mounted    = 0;
static uint8_t       log_file_open  = 0;
static SD_Card_Info  sd_info;
static char          log_buffer[SD_CARD_LOG_BUFFER_SIZE];
static uint32_t      log_sequence   = 0;

/* Static per-file cache for the log file so LittleFS never calls malloc.
 * Size MUST equal cfg.cache_size (4096, defined in lfs_sd_bd.c). A full-block
 * cache means a cluster of CSV rows accumulates in RAM and is flushed with a
 * single 4 KiB program operation, rather than 8 separate 512 B ops. */
static uint8_t log_file_cache[4096];
static struct lfs_file_config log_file_cfg = {
    .buffer   = log_file_cache,
    .attrs    = NULL,
    .attr_count = 0,
};

/* Shared ad-hoc file config for short-lived opens (BEACON.TXT, COMPCAL.BIN,
 * self-test file). Safe because we never open two of these concurrently; the
 * log file has its own dedicated cache above. Under LFS_NO_MALLOC every file
 * open must supply its own cache buffer, so lfs_file_opencfg is the only
 * available open API. */
static uint8_t adhoc_file_cache[4096];
static struct lfs_file_config adhoc_file_cfg = {
    .buffer   = adhoc_file_cache,
    .attrs    = NULL,
    .attr_count = 0,
};
#define LFS_OPEN(lfs, f, path, flags) \
    lfs_file_opencfg((lfs), (f), (path), (flags), &adhoc_file_cfg)

/* Navigation log CSV schema. Must stay in sync.
 *
 * Columns (1-based):
 *    1  Timestamp (SSS.mmm since boot)   2  Type ("NAV")
 *    3  PktSrc ("GPS"/"FUS")             4-6  Beacon lat/lon/alt
 *    7  BeaconSats                       8-10 VN/VE/VD  (fused only)
 *   11  FusedAge_ds                     12  FusedFlags (hex byte)
 *   13-15 Base lat/lon/alt              16  Distance_km
 *   17-18 Bearing/Heading deg           19-20 RSSI dBm / SNR dB
 */
static const char SD_LOG_HEADER[] =
    "Timestamp,Type,PktSrc,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,"
    "VN_ms,VE_ms,VD_ms,FusedAge_ds,FusedFlags,"
    "BaseLat,BaseLon,BaseAlt_m,Distance_km,Bearing_deg,Heading_deg,"
    "RSSI_dBm,SNR_dB\n";
static const char SD_NAV_ROW_FMT[] =
    "%s,NAV,%s,%.6f,%.6f,%.1f,%d,"
    "%.2f,%.2f,%.2f,%d,%02X,"
    "%.6f,%.6f,%.1f,%.3f,%.1f,%.1f,%d,%d\n";

/* Private function prototypes -----------------------------------------------*/
static SD_Card_Status SD_Card_WriteLogEntry(const char *entry);
static void           SD_Card_UpdateInfo(void);
static SD_Card_Status sd_open_new_log_file(char *filename_out, uint32_t max_len);

/* ========================================================================= */
/* Init / deinit / presence                                                  */
/* ========================================================================= */

SD_Card_Status SD_Card_Init(void)
{
    memset(&sd_info, 0, sizeof(sd_info));

    /* Detect-pin read kept for its diagnostic side effects (pin state +
     * CMD0 response into sd_info). IsPresent() deliberately always returns
     * true - the detect wiring is unreliable and a false "no card" would
     * lock out the user - so there is intentionally no NOT_PRESENT bail. */
    (void)SD_Card_IsPresent();
    sd_info.is_present = 1;

    /* 1. Bring up the card at the SD-SPI level and fill in lfs_sd_cfg.
     *    Return codes from lfs_sd_bd_init() are mapped to distinct sentinels
     *    so the display's "Err: N" code pinpoints which init step failed:
     *       91 = SD_initialize (CMD0/ACMD41 sequence)
     *       92 = SD_ioctl GET_SECTOR_COUNT (CMD9)
     *       99 = any other / unknown */
    int bd_err = lfs_sd_bd_init();
    if (bd_err != 0) {
        sd_info.write_errors = (bd_err == -1) ? 91
                             : (bd_err == -2) ? 92
                             : 99;
        return SD_CARD_ERROR;
    }

    /* 2. Try to mount. Retry a few times before concluding the card needs
     *    formatting - a single bad SPI read should NEVER nuke existing flight
     *    data. Only after three consecutive mount failures do we assume the
     *    card is genuinely blank / FAT-formatted and reformat it to LittleFS.
     *    (Format itself also produces a clean superblock, so after format we
     *    attempt mount once more and surface a distinct error code if even
     *    that fails.) */
    int err = -1;
    for (int attempt = 0; attempt < 3 && err != 0; attempt++) {
        if (attempt > 0) HAL_Delay(50);  /* let the bus settle */
        err = lfs_mount(&lfs, &lfs_sd_cfg);
    }
    if (err) {
        Display_Clear();
        Display_DrawTextRowCol(2, 0, "SD: formatting");
        Display_DrawTextRowCol(3, 0, "as LittleFS...");
        Display_Update();

        err = lfs_format(&lfs, &lfs_sd_cfg);
        if (err) {
            sd_info.write_errors = 13; /* format failed - SPI or card dead */
            return SD_CARD_ERROR;
        }
        err = lfs_mount(&lfs, &lfs_sd_cfg);
        if (err) {
            sd_info.write_errors = 1;  /* mount after format failed */
            return SD_CARD_ERROR;
        }
    }

    lfs_mounted         = 1;
    sd_initialized      = 1;
    sd_info.is_mounted  = 1;
    sd_info.is_initialized = 1;

    SD_Card_UpdateInfo();
    return SD_CARD_OK;
}

SD_Card_Status SD_Card_DeInit(void)
{
    if (log_file_open) {
        lfs_file_close(&lfs, &log_file);
        log_file_open = 0;
    }
    if (lfs_mounted) {
        lfs_unmount(&lfs);
        lfs_mounted = 0;
    }
    memset(&sd_info, 0, sizeof(sd_info));
    sd_initialized = 0;
    return SD_CARD_OK;
}

bool SD_Card_IsPresent(void)
{
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN);
    sd_info.detect_pin_state = (pin_state == GPIO_PIN_SET) ? 1 : 0;

    extern uint8_t SD_Type;
    sd_info.cmd0_response = SD_Type;

    /* Detect-pin bypass preserved from FatFS era - the wiring is unreliable
     * and false "no card" readings would lock out the user. */
    return true;
}

SD_Card_Status SD_Card_GetInfo(SD_Card_Info *info)
{
    if (info == NULL) return SD_CARD_ERROR;
    SD_Card_UpdateInfo();
    memcpy(info, &sd_info, sizeof(SD_Card_Info));
    return SD_CARD_OK;
}

static void SD_Card_UpdateInfo(void)
{
    if (!sd_initialized || !lfs_mounted) return;

    /* Total size: reported by the block-device layer (physical card size). */
    sd_info.total_size_mb = lfs_sd_bd_capacity_mb();

    /* Free: block_count - allocated. lfs_fs_size returns allocated blocks. */
    lfs_ssize_t used = lfs_fs_size(&lfs);
    if (used >= 0) {
        /* blocks * (block_size / 1024 / 1024) = blocks * 4 / 1024 */
        uint32_t total_blocks = lfs_sd_cfg.block_count;
        uint32_t free_blocks  = (total_blocks > (uint32_t)used)
                                ? (total_blocks - (uint32_t)used) : 0;
        /* 4 KiB blocks ⇒ MiB = blocks / 256 */
        sd_info.free_space_mb = free_blocks / 256U;
    }
}

/* ========================================================================= */
/* Log file creation (lazy)                                                  */
/* ========================================================================= */

/**
 * Scan the root for the highest existing L<N>.TXT and open L<N+1>.TXT.
 * LittleFS has no 8.3 restriction, but we keep the old naming scheme so
 * existing post-flight tooling still works.
 */
static SD_Card_Status sd_open_new_log_file(char *filename_out, uint32_t max_len)
{
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;

    if (log_file_open) {
        lfs_file_close(&lfs, &log_file);
        log_file_open = 0;
    }

    /* Scan directory */
    unsigned long next_seq = 1;
    lfs_dir_t dir;
    struct lfs_info info;
    if (lfs_dir_open(&lfs, &dir, "/") == 0) {
        while (lfs_dir_read(&lfs, &dir, &info) > 0) {
            const char *fn = info.name;
            if (info.type != LFS_TYPE_REG) continue;
            if (fn[0] != 'L' && fn[0] != 'l') continue;
            if (fn[1] == '\0') continue;
            const char *p = fn + 1;
            while (*p >= '0' && *p <= '9') p++;
            if (p == fn + 1) continue;
            if (!(p[0] == '.' &&
                  (p[1] == 'T' || p[1] == 't') &&
                  (p[2] == 'X' || p[2] == 'x') &&
                  (p[3] == 'T' || p[3] == 't') &&
                  p[4] == '\0')) continue;
            unsigned long n = strtoul(fn + 1, NULL, 10);
            if (n + 1 > next_seq) next_seq = n + 1;
        }
        lfs_dir_close(&lfs, &dir);
    }

    char filename[16];
    snprintf(filename, sizeof(filename), "L%04lu.TXT", next_seq);

    int err = lfs_file_opencfg(&lfs, &log_file, filename,
                               LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                               &log_file_cfg);
    if (err) {
        sd_info.write_errors = (uint32_t)(-err);
        return SD_CARD_ERROR;
    }

    lfs_ssize_t written = lfs_file_write(&lfs, &log_file, SD_LOG_HEADER,
                                         sizeof(SD_LOG_HEADER) - 1);
    if (written < 0) {
        lfs_file_close(&lfs, &log_file);
        log_file_open = 0;
        return SD_CARD_ERROR;
    }

    /* Sync so the header + file existence is durable before any data. */
    lfs_file_sync(&lfs, &log_file);

    log_file_open = 1;
    strncpy(sd_info.current_log_file, filename, SD_CARD_MAX_FILENAME - 1);
    sd_info.current_log_file[SD_CARD_MAX_FILENAME - 1] = '\0';
    sd_info.files_logged++;
    sd_info.bytes_written += (uint32_t)written;

    if (filename_out && max_len) {
        strncpy(filename_out, filename, max_len - 1);
        filename_out[max_len - 1] = '\0';
    }
    return SD_CARD_OK;
}

SD_Card_Status SD_Card_CreateLogFile(char *filename, uint32_t max_len)
{
    if (filename == NULL || max_len == 0) return SD_CARD_ERROR;
    if (log_file_open) {
        strncpy(filename, sd_info.current_log_file, max_len - 1);
        filename[max_len - 1] = '\0';
        return SD_CARD_OK;
    }
    return sd_open_new_log_file(filename, max_len);
}

/* ========================================================================= */
/* Log write helper                                                          */
/* ========================================================================= */

static SD_Card_Status SD_Card_WriteLogEntry(const char *entry)
{
    /* (No NULL guard: this helper is static and every call site passes a
     * freshly formatted stack buffer.) */
    if (!sd_initialized || !lfs_mounted) {
        return SD_CARD_ERROR;
    }
    /* Drop silently if no log file yet - prevents no-RF boots from creating
     * artifacts on the card. */
    if (!log_file_open) return SD_CARD_OK;

    size_t len = strlen(entry);
    lfs_ssize_t w = lfs_file_write(&lfs, &log_file, entry, len);
    if (w < 0 || (size_t)w != len) {
        sd_info.write_errors++;
        return SD_CARD_ERROR;
    }
    sd_info.bytes_written += (uint32_t)w;
    log_sequence++;
    return SD_CARD_OK;
}

/* ========================================================================= */
/* Public log APIs                                                           */
/* ========================================================================= */

void SD_Card_GetTimestamp(char *timestamp_str, uint32_t max_len)
{
    if (timestamp_str == NULL) return;
    uint32_t tick = HAL_GetTick();
    snprintf(timestamp_str, max_len, "%lu.%03lu",
             (unsigned long)(tick / 1000), (unsigned long)(tick % 1000));
}

SD_Card_Status SD_Card_LogGPS(GPS_Data *gps_data, Log_Entry_Type log_type)
{
    if (gps_data == NULL || !sd_initialized) return SD_CARD_ERROR;
    char ts[32];
    SD_Card_GetTimestamp(ts, sizeof(ts));
    const char *type_str = (log_type == LOG_TYPE_GPS_LOCAL) ? "GPS_LOCAL" : "GPS_REMOTE";
    snprintf(log_buffer, sizeof(log_buffer),
             "%s,%s,%.6f,%.6f,%.1f,%d,%d,%.1f\n",
             ts, type_str,
             gps_data->latitude, gps_data->longitude, gps_data->altitude,
             gps_data->fix, gps_data->satellites, gps_data->speed);
    return SD_Card_WriteLogEntry(log_buffer);
}

SD_Card_Status SD_Card_LogRFPacket(const char *packet_data, uint32_t packet_length, int16_t rssi)
{
    (void)packet_length;
    if (packet_data == NULL || !sd_initialized) return SD_CARD_ERROR;
    char ts[32];
    SD_Card_GetTimestamp(ts, sizeof(ts));
    snprintf(log_buffer, sizeof(log_buffer), "%s,RF_PACKET,%d,%s\n", ts, rssi, packet_data);
    return SD_Card_WriteLogEntry(log_buffer);
}

SD_Card_Status SD_Card_LogCompass(float heading, int16_t x, int16_t y, int16_t z)
{
    if (!sd_initialized) return SD_CARD_ERROR;
    char ts[32];
    SD_Card_GetTimestamp(ts, sizeof(ts));
    snprintf(log_buffer, sizeof(log_buffer), "%s,COMPASS,%.1f,%d,%d,%d\n", ts, heading, x, y, z);
    return SD_Card_WriteLogEntry(log_buffer);
}

/* EVENT/ERROR entries sync immediately, like nav rows. Only nav rows used
 * to sync, so a no-fix pad session (scan lock + heartbeats, never a nav
 * row) kept every event in the littlefs cache and power-off reverted the
 * file to its last-synced state: a header and nothing else. Field cards
 * showed exactly that - header-only L000N.TXT files from sessions where
 * the beacon was demonstrably heard (the log file is only created on
 * proof of beacon). Events are rare (<=1 per 5 s heartbeat), so the
 * ~50 ms commit is trivial. */
SD_Card_Status SD_Card_LogEvent(const char *msg)
{
    if (msg == NULL || !sd_initialized) return SD_CARD_ERROR;
    char ts[32];
    SD_Card_GetTimestamp(ts, sizeof(ts));
    snprintf(log_buffer, sizeof(log_buffer), "%s,EVENT,%s\n", ts, msg);
    SD_Card_Status status = SD_Card_WriteLogEntry(log_buffer);
    if (status == SD_CARD_OK && log_file_open) {
        lfs_file_sync(&lfs, &log_file);
    }
    return status;
}

SD_Card_Status SD_Card_LogError(const char *msg)
{
    if (msg == NULL || !sd_initialized) return SD_CARD_ERROR;
    char ts[32];
    SD_Card_GetTimestamp(ts, sizeof(ts));
    snprintf(log_buffer, sizeof(log_buffer), "%s,ERROR,%s\n", ts, msg);
    SD_Card_Status status = SD_Card_WriteLogEntry(log_buffer);
    if (status == SD_CARD_OK && log_file_open) {
        lfs_file_sync(&lfs, &log_file);
    }
    return status;
}

SD_Card_Status SD_Card_Flush(void)
{
    if (!sd_initialized) return SD_CARD_ERROR;
    if (log_file_open) lfs_file_sync(&lfs, &log_file);
    return SD_CARD_OK;
}

/**
 * @brief Open the log file now if it isn't open yet.
 *
 * The log file is normally created lazily by SD_Card_LogNavigation on the
 * first position packet, and SD_Card_WriteLogEntry silently drops entries
 * until then (so no-RF boots leave no artifact on the card). Call this
 * before logging an event that proves a real beacon is on the air (heard
 * heartbeat/callsign, channel-scan lock) so those pre-fix entries are kept.
 */
SD_Card_Status SD_Card_EnsureLogFile(void)
{
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;
    if (log_file_open) return SD_CARD_OK;
    return sd_open_new_log_file(NULL, 0);
}

/* ========================================================================= */
/* Navigation logging (primary data path)                                    */
/* ========================================================================= */

SD_Card_Status SD_Card_LogNavigation(GPS_Data *beacon_gps, GPS_Data *base_gps,
                                     float distance_km, float bearing_deg,
                                     float heading_deg, int16_t rssi, int8_t snr)
{
    if (!sd_initialized) return SD_CARD_ERROR;

    if (!log_file_open) {
        if (sd_open_new_log_file(NULL, 0) != SD_CARD_OK) return SD_CARD_ERROR;
    }

    char ts[16];
    SD_Card_GetTimestamp(ts, sizeof(ts));

    float b_lat = 0, b_lon = 0, b_alt = 0;
    int   b_sats = 0;
    const char *pkt_src = "GPS";
    float vn = 0, ve = 0, vd = 0;
    int   age_ds = 0;
    uint8_t fused_flags = 0;
    if (beacon_gps) {
        b_lat = beacon_gps->latitude;
        b_lon = beacon_gps->longitude;
        b_alt = beacon_gps->altitude;
        b_sats = beacon_gps->satellites;
        if (beacon_gps->is_fused) {
            pkt_src = "FUS";
            vn = beacon_gps->v_north;
            ve = beacon_gps->v_east;
            vd = beacon_gps->v_down;
            age_ds = beacon_gps->fused_age_ds;
            fused_flags = 0x01;
            if (beacon_gps->fused_dr)          fused_flags |= 0x02;
            if (beacon_gps->fused_gps_fresh)   fused_flags |= 0x04;
            if (beacon_gps->fused_imu_healthy) fused_flags |= 0x08;
            if (beacon_gps->launch_detected)   fused_flags |= 0x10;
        } else if (beacon_gps->launch_detected) {
            fused_flags = 0x10;
        }
    }
    float s_lat = 0, s_lon = 0, s_alt = 0;
    if (base_gps) {
        s_lat = base_gps->latitude;
        s_lon = base_gps->longitude;
        s_alt = base_gps->altitude;
    }

    snprintf(log_buffer, sizeof(log_buffer), SD_NAV_ROW_FMT,
             ts, pkt_src,
             b_lat, b_lon, b_alt, b_sats,
             vn, ve, vd, age_ds, (unsigned)fused_flags,
             s_lat, s_lon, s_alt,
             distance_km, bearing_deg, heading_deg, (int)rssi, (int)snr);

    SD_Card_Status status = SD_Card_WriteLogEntry(log_buffer);

    /* Sync after every row so no packet is ever lost to power-off / brown-out.
     * Earlier this was batched (5 s cap / 2 s idle) to reduce SD churn, but
     * observed logs showed short sessions losing almost all their packets
     * because the rocket was powered down inside the sync window. With SPI
     * now running at ~1.3 MHz the commit pair is ~50 ms - trivial compared
     * to packet cadence (~1-3 Hz), and safely inside the 500 ms display
     * update budget. Flight data > throughput here. */
    if (status == SD_CARD_OK) {
        lfs_file_sync(&lfs, &log_file);
    }
    return status;
}

/* ========================================================================= */
/* Beacon persistence (atomic by virtue of LittleFS COW)                     */
/* ========================================================================= */

SD_Card_Status SD_Card_SaveLastBeacon(float lat, float lon, float alt)
{
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;

    lfs_file_t f;
    int err = LFS_OPEN(&lfs, &f, BEACON_FILENAME,
                       LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err) return SD_CARD_ERROR;

    char buf[48];
    int n = snprintf(buf, sizeof(buf), "%.7f,%.7f,%.1f\n", lat, lon, alt);
    if (n < 0 || n >= (int)sizeof(buf)) { lfs_file_close(&lfs, &f); return SD_CARD_ERROR; }

    lfs_ssize_t w = lfs_file_write(&lfs, &f, buf, n);
    int close_err = lfs_file_close(&lfs, &f);  /* close flushes + commits */

    /* LittleFS commits on close: if power dies mid-close, the prior valid
     * version of BEACON.TXT remains readable on the next boot. No temp-file
     * rename dance is needed - this is the core crash-safety property. */

    return (w == n && close_err == 0) ? SD_CARD_OK : SD_CARD_ERROR;
}

SD_Card_Status SD_Card_LoadLastBeacon(float *lat, float *lon, float *alt)
{
    if (!lat || !lon || !alt) return SD_CARD_ERROR;
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;

    lfs_file_t f;
    int err = LFS_OPEN(&lfs, &f, BEACON_FILENAME, LFS_O_RDONLY);
    if (err) return SD_CARD_ERROR;

    char buf[48];
    lfs_ssize_t r = lfs_file_read(&lfs, &f, buf, sizeof(buf) - 1);
    lfs_file_close(&lfs, &f);
    if (r <= 0) return SD_CARD_ERROR;
    buf[r] = '\0';

    float pl, po, pa;
    if (sscanf(buf, "%f,%f,%f", &pl, &po, &pa) != 3) return SD_CARD_ERROR;
    if (pl < -90.0f || pl > 90.0f || po < -180.0f || po > 180.0f) return SD_CARD_ERROR;
    *lat = pl; *lon = po; *alt = pa;
    return SD_CARD_OK;
}

/* ========================================================================= */
/* Compass calibration persistence                                           */
/* ========================================================================= */

SD_Card_Status SD_Card_SaveCompassCal(const uint8_t *data, uint8_t len)
{
    if (!data || len < COMPCAL_DATA_LEN) return SD_CARD_ERROR;
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;

    uint8_t buf[COMPCAL_FILE_LEN];
    buf[0] = COMPCAL_MAGIC_0;
    buf[1] = COMPCAL_MAGIC_1;
    memcpy(&buf[2], data, COMPCAL_DATA_LEN);
    uint8_t csum = 0;
    for (uint8_t i = 0; i < COMPCAL_DATA_LEN; i++) csum ^= data[i];
    buf[COMPCAL_FILE_LEN - 1] = csum;

    lfs_file_t f;
    int err = LFS_OPEN(&lfs, &f, COMPCAL_FILENAME,
                       LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err) return SD_CARD_ERROR;

    lfs_ssize_t w = lfs_file_write(&lfs, &f, buf, COMPCAL_FILE_LEN);
    int close_err = lfs_file_close(&lfs, &f);
    return (w == COMPCAL_FILE_LEN && close_err == 0) ? SD_CARD_OK : SD_CARD_ERROR;
}

SD_Card_Status SD_Card_LoadCompassCal(uint8_t *data, uint8_t len)
{
    if (!data || len < COMPCAL_DATA_LEN) return SD_CARD_ERROR;
    if (!sd_initialized || !lfs_mounted) return SD_CARD_ERROR;

    lfs_file_t f;
    int err = LFS_OPEN(&lfs, &f, COMPCAL_FILENAME, LFS_O_RDONLY);
    if (err) return SD_CARD_ERROR;

    uint8_t buf[COMPCAL_FILE_LEN];
    lfs_ssize_t r = lfs_file_read(&lfs, &f, buf, COMPCAL_FILE_LEN);
    lfs_file_close(&lfs, &f);
    if (r != COMPCAL_FILE_LEN) return SD_CARD_ERROR;
    if (buf[0] != COMPCAL_MAGIC_0 || buf[1] != COMPCAL_MAGIC_1) return SD_CARD_ERROR;
    uint8_t csum = 0;
    for (uint8_t i = 0; i < COMPCAL_DATA_LEN; i++) csum ^= buf[2 + i];
    if (csum != buf[COMPCAL_FILE_LEN - 1]) return SD_CARD_ERROR;
    memcpy(data, &buf[2], COMPCAL_DATA_LEN);
    return SD_CARD_OK;
}

/* ========================================================================= */
/* Format (destroys all data)                                                */
/* ========================================================================= */

SD_Card_Status SD_Card_Format(void)
{
    if (!sd_info.is_present) return SD_CARD_NOT_PRESENT;

    if (log_file_open) { lfs_file_close(&lfs, &log_file); log_file_open = 0; }
    if (lfs_mounted)   { lfs_unmount(&lfs); lfs_mounted = 0; }

    if (lfs_format(&lfs, &lfs_sd_cfg) != 0) return SD_CARD_ERROR;
    if (lfs_mount(&lfs, &lfs_sd_cfg) != 0)  return SD_CARD_ERROR;

    lfs_mounted = 1;
    sd_info.is_mounted = 1;
    sd_info.files_logged  = 0;
    sd_info.bytes_written = 0;
    sd_info.write_errors  = 0;
    memset(sd_info.current_log_file, 0, sizeof(sd_info.current_log_file));
    SD_Card_UpdateInfo();
    return SD_CARD_OK;
}

/* ========================================================================= */
/* Self-test (write, sync, close, reopen, read, verify, delete)              */
/* ========================================================================= */

static uint8_t self_test_step = 0;
static uint8_t self_test_fres = 0;

void SD_Card_GetSelfTestError(uint8_t *step, uint8_t *fres)
{
    if (step) *step = self_test_step;
    if (fres) *fres = self_test_fres;
}

SD_Card_Status SD_Card_SelfTest(void)
{
    self_test_step = 0;
    self_test_fres = 0;

    if (!sd_initialized || !lfs_mounted) {
        self_test_step = 1;
        self_test_fres = 3; /* NOT_READY sentinel for the display decoder */
        return SD_CARD_ERROR;
    }

    static const char TEST_PATTERN[] = "SD_TEST:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\n";
    const int pattern_len = (int)sizeof(TEST_PATTERN) - 1;
    static char readback[sizeof(TEST_PATTERN)];
    memset(readback, 0, sizeof(readback));

    lfs_file_t f;
    lfs_ssize_t w, r;
    int err;

#define ST(step_num) do { \
        self_test_step = (step_num); \
        char _st[16]; \
        Display_Clear(); \
        Display_DrawTextRowCol(3, 0, "SD Test"); \
        snprintf(_st, sizeof(_st), "Step %d", (step_num)); \
        Display_DrawTextRowCol(4, 0, _st); \
        Display_Update(); \
    } while (0)

    ST(1);
    err = LFS_OPEN(&lfs, &f, "SD_TEST.TMP",
                   LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err) { self_test_fres = (uint8_t)(-err); return SD_CARD_ERROR; }

    ST(2);
    w = lfs_file_write(&lfs, &f, TEST_PATTERN, pattern_len);
    if (w != pattern_len) { lfs_file_close(&lfs, &f); self_test_fres = 1; return SD_CARD_ERROR; }

    ST(3);
    err = lfs_file_sync(&lfs, &f);
    if (err) { lfs_file_close(&lfs, &f); self_test_fres = (uint8_t)(-err); return SD_CARD_ERROR; }

    self_test_step = 33;
    err = lfs_file_close(&lfs, &f);
    if (err) { self_test_fres = (uint8_t)(-err); return SD_CARD_ERROR; }

    ST(4);
    err = LFS_OPEN(&lfs, &f, "SD_TEST.TMP", LFS_O_RDONLY);
    if (err) { self_test_fres = (uint8_t)(-err); return SD_CARD_ERROR; }

    ST(5);
    r = lfs_file_read(&lfs, &f, readback, pattern_len);
    lfs_file_close(&lfs, &f);
    if (r != pattern_len) { self_test_fres = 1; return SD_CARD_ERROR; }

    ST(6);
    if (memcmp(TEST_PATTERN, readback, pattern_len) != 0) {
        self_test_fres = (uint8_t)readback[0];
        return SD_CARD_ERROR;
    }

    ST(7);
    lfs_remove(&lfs, "SD_TEST.TMP");
    return SD_CARD_OK;

#undef ST
}
