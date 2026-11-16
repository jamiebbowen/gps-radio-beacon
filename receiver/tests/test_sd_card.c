/**
 * @file test_sd_card.c
 * @brief Host-side unit tests for the SD-card logging module.
 *
 * Runs the REAL LittleFS (linked from lib/littlefs) over the real block
 * device (lfs_sd_bd.c) on the shared sd_diskio card fake (fake_sd_card.h):
 * the firmware's geometry and failure codes, end to end. This exercises
 * the actual on-card format: mount-retry-then-format, lazy log-file
 * creation with sequence numbering, every log API's CSV output,
 * beacon/compass-cal persistence (including corruption rejection), format,
 * and the self-test.
 *
 * Fault knobs on the card simulate a dead card (init errors 91/92), a card
 * that cannot be written (format error 13), a card that cannot be read
 * (error 14), and a bus that went marginal at its clock (the step-down).
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "sd_card.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/sd_card.c"

/* Test-side teardown/sync. The firmware never unmounts or force-flushes at
 * runtime (power-off is the teardown; SD_Card_ServiceSync paces syncs), so
 * tests drive the module statics directly. */
static void sd_deinit(void)
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
}

static SD_Card_Status sd_flush(void)
{
    if (!sd_initialized) return SD_CARD_ERROR;
    if (log_file_open) lfs_file_sync(&lfs, &log_file);
    sd_dirty = 0;
    sd_dirty_rows = 0;
    return SD_CARD_OK;
}

/* ------------------------------------------------------------------ */
/* The card: the shared sd_diskio fake under the real lfs_sd_bd.c      */
/* ------------------------------------------------------------------ */

#include "fake_sd_card.h"

/* ------------------------------------------------------------------ */
/* HAL / display fakes                                                 */
/* ------------------------------------------------------------------ */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    return GPIO_PIN_SET;
}
void HAL_Delay(uint32_t ms) { (void)ms; }

void Display_Clear(void) {}
void Display_Update(void) {}
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
    (void)row; (void)col; (void)text;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void wipe_card(void)
{
    /* Close/unmount first: on target you cannot wipe a mounted card, and
     * wiping under a live mount leaves stale open-file state behind. */
    sd_deinit();
    fake_sd_reset();                               /* a healthy card...  */
    fake_sd_wipe(0xFF);                            /* ...erased          */
}

/** Read a whole file off the card into buf; returns length or -1. */
static int read_file(const char *path, char *buf, int max)
{
    lfs_file_t f;
    if (lfs_file_opencfg(&lfs, &f, path, LFS_O_RDONLY, &adhoc_file_cfg) != 0)
        return -1;
    lfs_ssize_t r = lfs_file_read(&lfs, &f, buf, max - 1);
    lfs_file_close(&lfs, &f);
    if (r < 0) return -1;
    buf[r] = '\0';
    return (int)r;
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_bd_failure_codes)
{
    /* Each block-device failure maps to a distinct display code so the
     * "Err: N" screen pinpoints the failed init step */
    fake_sd_reset();
    fake_sd.absent = 1;                    /* CMD0/ACMD41 never answered */
    CHECK(SD_Card_Init() == SD_CARD_ERROR);
    CHECK(sd_info.write_errors == 91);

    fake_sd_reset();
    fake_sd.csd_result = RES_ERROR;        /* CMD9 failed */
    CHECK(SD_Card_Init() == SD_CARD_ERROR);
    CHECK(sd_info.write_errors == 92);

    fake_sd_reset();
    CHECK(SD_Card_GetInfo(NULL) == SD_CARD_ERROR);
}

TEST(test_init_blank_card_formats_and_mounts)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);      /* 3 mount fails -> format */

    SD_Card_Info info;
    CHECK(SD_Card_GetInfo(&info) == SD_CARD_OK);
    CHECK(info.is_mounted == 1);
    CHECK(info.total_size_mb == FAKE_SD_SECTORS / 2048u);   /* 4 MiB card */
    CHECK(info.free_space_mb > 0);            /* 1024 blocks, few used */
    CHECK(SD_Card_IsPresent() == true);
}

TEST(test_init_format_failure_paths)
{
    /* Card refuses writes: format fails -> error 13 */
    wipe_card();
    fake_sd.read_only = 1;
    CHECK(SD_Card_Init() == SD_CARD_ERROR);
    CHECK(sd_info.write_errors == 13);
    fake_sd.read_only = 0;

    /* NOTE: the sibling branch (format OK but the immediate re-mount
     * fails, error code 1) is not reachable through a behavioral fake:
     * littlefs validates its final superblock commit with a read-back,
     * so any fault injected before format returns fails the format
     * itself (-> 13), and no block-device call happens between format
     * returning and mount reading. The branch guards against media that
     * lie about persisting writes - kept as defensive code. */
}

TEST(test_init_never_formats_a_card_holding_littlefs)
{
    /* A card that holds LittleFS but will not mount right now (marginal
     * contact, slow after a brown-out, a corrupt metadata pair) keeps its
     * flight logs: no auto-format, error 14 - extract it on a PC. */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);                /* blank: formats */
    /* The superblock magic the check looks for, pinned to a real format:
     * "littlefs" after the revision count and the superblock tag */
    CHECK(memcmp(&fake_sd_storage[8], "littlefs", 8) == 0 ||
          memcmp(&fake_sd_storage[LFS_SD_BLOCK_SIZE + 8], "littlefs", 8) == 0);
    CHECK(SD_Card_SaveLastBeacon(39.5f, -105.2f, 1655.0f) == SD_CARD_OK);
    sd_deinit();

    /* Unreadable: the mount and the superblock check both fail */
    fake_sd.read_fails = UINT32_MAX;
    CHECK(SD_Card_Init() == SD_CARD_ERROR);
    CHECK(sd_info.write_errors == 14);
    fake_sd.read_fails = 0;
    CHECK(SD_Card_Init() == SD_CARD_OK);
    float lat = 0, lon = 0, alt = 0;
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_OK);
    CHECK(lat > 39.4f && lat < 39.6f);
    sd_deinit();

    /* Readable, superblock there, metadata pair corrupt (both revision
     * counts flipped): still not formatted */
    uint8_t rev0 = fake_sd_storage[0], rev1 = fake_sd_storage[LFS_SD_BLOCK_SIZE];
    fake_sd_storage[0] ^= 0xFF;
    fake_sd_storage[LFS_SD_BLOCK_SIZE] ^= 0xFF;
    CHECK(SD_Card_Init() == SD_CARD_ERROR);
    CHECK(sd_info.write_errors == 14);
    fake_sd_storage[0] = rev0;
    fake_sd_storage[LFS_SD_BLOCK_SIZE] = rev1;
    CHECK(SD_Card_Init() == SD_CARD_OK);
    lat = 0;
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_OK);
    CHECK(lat > 39.4f && lat < 39.6f);
}

TEST(test_remount_preserves_files)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_SaveLastBeacon(39.5f, -105.2f, 1655.0f) == SD_CARD_OK);

    /* Power cycle: existing filesystem mounts without formatting */
    sd_deinit();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    float lat, lon, alt;
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_OK);
    CHECK(lat > 39.4f && lat < 39.6f);
    CHECK(lon < -105.1f && lon > -105.3f);
}

TEST(test_log_file_lazy_creation_and_sequencing)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* Entries before any log file exist are silently dropped: a no-RF
     * boot must leave no artifact on the card */
    CHECK(SD_Card_LogEvent("before file") == SD_CARD_OK);
    char buf[256];
    CHECK(read_file("L0001.TXT", buf, sizeof(buf)) == -1);

    /* First nav row creates L0000.TXT with the CSV header */
    Test_SetTick(12345);
    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.891f; beacon.longitude = -105.112f;
    beacon.altitude = 1800.0f; beacon.satellites = 9;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.25f, 270.0f, 45.0f,
                                5.0f, -2.0f, -92, 6) == SD_CARD_OK);

    char big[2048];
    CHECK(sd_flush() == SD_CARD_OK);   /* deferred-sync model: explicit flush before readback */
    CHECK(read_file("L0001.TXT", big, sizeof(big)) > 0);  /* seq starts at 1 */
    CHECK(strstr(big, "Timestamp,Type,PktSrc") == big);   /* header first */
    CHECK(strstr(big, "12.345,NAV,GPS,39.89") != NULL);   /* row followed */
    /* LOG_CONTRACT header: receiver/tools/tests/test_tools.py parses with this header */
    CHECK(strstr(big, "Timestamp,Type,PktSrc,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,VN_ms,VE_ms,VD_ms,FusedAge_ds,FusedFlags,BaseLat,BaseLon,BaseAlt_m,Distance_km,Bearing_deg,Heading_deg,RSSI_dBm,SNR_dB,Pitch_deg,Roll_deg,BaseSats,BaseHDOP\n") == big);
    /* LOG_CONTRACT nav: receiver/tools/tests/test_tools.py parses this exact row */
    CHECK(strstr(big, "12.345,NAV,GPS,39.890999,-105.112000,1800.0,9,0.00,0.00,0.00,0,00,0.000000,0.000000,0.0,1.250,270.0,45.0,-92,6,5.0,-2.0,0,0.0\n") != NULL);

    /* The write-time tracker: reads cleanly and clears on read. (Fake
     * ticks can't measure real bursts - only the semantics are pinned.) */
    {
        Test_SetTick(12345 + 37);
        Test_SetTickAutoAdvance(1);                       /* 1 ms/call */
        (void)SD_Card_LogNavigation(&beacon, NULL, 1.0f, 10.0f, 20.0f,
                                    0.0f, 0.0f, -90, 5);
        Test_SetTickAutoAdvance(0);
        CHECK(SD_Card_TakeMaxWriteMs() != 0);
        CHECK(SD_Card_TakeMaxWriteMs() == 0);             /* read clears */
    }

    /* Second boot: sequence advances to L0002.TXT */
    sd_deinit();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(strcmp(sd_info.current_log_file, "L0002.TXT") == 0);

    /* EnsureLogFile with one already open is a no-op */
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(strcmp(sd_info.current_log_file, "L0002.TXT") == 0);
}

TEST(test_all_log_row_formats)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    Test_SetTick(2500);

    GPS_Data gps; memset(&gps, 0, sizeof(gps));
    gps.latitude = 40.0f; gps.longitude = -105.0f; gps.altitude = 1600.0f;
    gps.fix = 1; gps.satellites = 7; gps.speed = 3.5f;


    CHECK(SD_Card_LogEvent("scan lock CH2") == SD_CARD_OK);
    CHECK(SD_Card_LogEvent(NULL) == SD_CARD_ERROR);
    CHECK(SD_Card_LogError("test error") == SD_CARD_OK);
    CHECK(SD_Card_LogError(NULL) == SD_CARD_ERROR);
    CHECK(sd_flush() == SD_CARD_OK);

    /* Fused-packet nav row carries velocities + FUS tag */
    GPS_Data fused; memset(&fused, 0, sizeof(fused));
    fused.latitude = 39.9f; fused.longitude = -105.1f;
    fused.is_fused = 1; fused.v_north = 12.5f; fused.v_east = -3.25f;
    fused.v_down = 40.0f; fused.fused_age_ds = 3;
    fused.fused_gps_fresh = 1;   /* flags -> 0x05 in the CSV */
    GPS_Data base; memset(&base, 0, sizeof(base));
    base.latitude = 39.7f; base.longitude = -105.0f; base.altitude = 1650.0f;
    CHECK(SD_Card_LogNavigation(&fused, &base, 2.5f, 180.0f, 90.0f,
                                12.0f, -4.0f, -88, 8) == SD_CARD_OK);

    /* Inertial-trace row: in-flight forensics at flight cadence. RSSI and
     * SNR both ride the row - during boost the noise floor rises and RSSI
     * alone can't show it. */
    ImuTracePacket_t imu; memset(&imu, 0, sizeof(imu));
    imu.ts_ms = (uint16_t)(512 | IMU_TS_V3_FLAG);  /* V3 marker is masked */
    imu.accel_x_cg = 55;  imu.accel_y_cg = -125; imu.accel_z_cg = 980;
    imu.gyro_x_dds = 57;  imu.gyro_y_dds = -114; imu.gyro_z_dds = 12;
    imu.peak_accel_mg = 1350; imu.temp_c10 = 231;
    CHECK(SD_Card_LogImuTrace(&imu, 3, -95, -12) == SD_CARD_OK);
    CHECK(SD_Card_LogImuTrace(NULL, 3, 0, 0) == SD_CARD_ERROR);

    /* BASE (operator-position walk forensics) row */
    base.hdop = 0.9f;
    CHECK(SD_Card_LogBase(&base) == SD_CARD_OK);
    CHECK(SD_Card_LogBase(NULL) == SD_CARD_ERROR);

    char big[4096];
    CHECK(sd_flush() == SD_CARD_OK);
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, "EVENT,scan lock CH2") != NULL);
    CHECK(strstr(big, "ERROR,test error") != NULL);
    CHECK(strstr(big, "NAV,FUS,39.9") != NULL);
    CHECK(strstr(big, "12.50,-3.25,40.00") != NULL);      /* velocities */
    CHECK(strstr(big, ",IMU,512,55,-125,980,57,-114,12,1350,231,-95,-12,3\n")
          != NULL);                                       /* full IMU row */
    /* LOG_CONTRACT imu: receiver/tools/tests/test_tools.py parses this exact row */
    CHECK(strstr(big, "2.500,IMU,512,55,-125,980,57,-114,12,1350,231,-95,-12,3\n") != NULL);
    CHECK(strstr(big, ",BASE,") != NULL);
    CHECK(strstr(big, "1650.0,0,0.9") != NULL);   /* alt, sats, hdop */
}

TEST(test_raw_nav_row_flags_carry_beacon_fix_type)
{
    /* Raw (non-fused) NAV rows put the beacon's claimed fix type in the
     * FusedFlags low nibble so post-flight rows can be weighted by it. */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    Test_SetTick(3000);

    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.5f; beacon.longitude = -105.2f; beacon.altitude = 900.0f;
    beacon.fix = 1;                       /* beacon claims a GPS fix */
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 0, 0, 0, 0, 0, -90, 5) == SD_CARD_OK);

    /* Same row with launch + landed set: nibble survives both flags */
    beacon.launch_detected = 1;
    beacon.fused_landed = 1;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 0, 0, 0, 0, 0, -90, 5) == SD_CARD_OK);

    /* No-fix beacon: nibble stays 0 */
    GPS_Data nofix = beacon;
    nofix.fix = 0;
    CHECK(SD_Card_LogNavigation(&nofix, NULL, 0, 0, 0, 0, 0, -90, 5) == SD_CARD_OK);

    char big[2048];
    CHECK(sd_flush() == SD_CARD_OK);
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, "0.00,0.00,0.00,0,01,") != NULL);   /* fix only     */
    CHECK(strstr(big, "0.00,0.00,0.00,0,31,") != NULL);   /* fix|launch|landed */
    CHECK(strstr(big, "0.00,0.00,0.00,0,30,") != NULL);   /* no fix, both flags */
}

TEST(test_apis_reject_when_uninitialized)
{
    sd_deinit();

    GPS_Data gps; memset(&gps, 0, sizeof(gps));
    float lat, lon, alt;

    CHECK(SD_Card_LogEvent("x") == SD_CARD_ERROR);
    CHECK(SD_Card_LogError("x") == SD_CARD_ERROR);
    CHECK(SD_Card_LogNavigation(&gps, NULL, 0, 0, 0, 0, 0, 0, 0) == SD_CARD_ERROR);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_ERROR);
    CHECK(SD_Card_SaveLastBeacon(1, 2, 3) == SD_CARD_ERROR);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);
    CHECK(SD_Card_SelfTest() == SD_CARD_ERROR);

    uint8_t step, fres;
    SD_Card_GetSelfTestError(&step, &fres);
    CHECK(step == 1 && fres == 3);            /* NOT_READY sentinel */
    SD_Card_GetSelfTestError(NULL, NULL);     /* must not crash */
}

TEST(test_beacon_persistence_validation)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    float lat, lon, alt;
    CHECK(SD_Card_LoadLastBeacon(NULL, &lon, &alt) == SD_CARD_ERROR);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR); /* no file */

    CHECK(SD_Card_SaveLastBeacon(39.5f, -105.2f, 1655.0f) == SD_CARD_OK);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_OK);

    /* Corrupt content: not three floats */
    lfs_file_t f;
    CHECK(lfs_file_opencfg(&lfs, &f, BEACON_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "garbage", 7);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);

    /* sscanf %f parses "nan": all three fields must be finite-checked
     * (NaN slips the lat/lon range checks; alt has no range check) */
    CHECK(lfs_file_opencfg(&lfs, &f, BEACON_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "nan,39.5,1655.0", 15);
    lfs_file_close(&lfs, &f);
    lat = lon = alt = 0.0f;
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);
    CHECK(lfs_file_opencfg(&lfs, &f, BEACON_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "39.5,-105.2,nan", 15);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);
    CHECK(lfs_file_opencfg(&lfs, &f, BEACON_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "39.5,-105.2,inf", 15);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);

    /* Out-of-range coordinates rejected */
    CHECK(SD_Card_SaveLastBeacon(99.0f, 0.5f, 100.0f) == SD_CARD_OK);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);

    /* Empty file rejected */
    CHECK(lfs_file_opencfg(&lfs, &f, BEACON_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadLastBeacon(&lat, &lon, &alt) == SD_CARD_ERROR);
}

TEST(test_last_channel_persistence)
{
    /* The boot scan starts on the channel the receiver last locked: a
     * beacon in its quiet pad sends only one packet a minute */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    uint8_t ch = 0xEE;
    CHECK(SD_Card_LoadLastChannel(&ch) == SD_CARD_ERROR);     /* none yet */
    fake_sd.read_only = 1;                 /* a card refusing writes says so */
    CHECK(SD_Card_SaveLastChannel(5) == SD_CARD_ERROR);
    fake_sd.read_only = 0;
    CHECK(SD_Card_SaveLastChannel(LORA_CHANNEL_COUNT) == SD_CARD_ERROR);
    CHECK(SD_Card_SaveLastChannel(5) == SD_CARD_OK);
    CHECK(SD_Card_LoadLastChannel(&ch) == SD_CARD_OK && ch == 5);
    CHECK(SD_Card_LoadLastChannel(NULL) == SD_CARD_ERROR);

    /* Survives a power cycle */
    sd_deinit();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    ch = 0;
    CHECK(SD_Card_LoadLastChannel(&ch) == SD_CARD_OK && ch == 5);

    /* A corrupt checksum is rejected */
    lfs_file_t f;
    uint8_t raw[LASTCH_FILE_LEN];
    CHECK(lfs_file_opencfg(&lfs, &f, LASTCH_FILENAME, LFS_O_RDONLY,
                           &adhoc_file_cfg) == 0);
    CHECK(lfs_file_read(&lfs, &f, raw, LASTCH_FILE_LEN) == LASTCH_FILE_LEN);
    lfs_file_close(&lfs, &f);
    raw[LASTCH_FILE_LEN - 1] ^= 0x01;
    CHECK(lfs_file_opencfg(&lfs, &f, LASTCH_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, raw, LASTCH_FILE_LEN);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadLastChannel(&ch) == SD_CARD_ERROR);
}

TEST(test_quatlock_persistence)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    uint8_t conv = 0;
    CHECK(SD_Card_LoadQuatLock(&conv) == SD_CARD_ERROR);          /* none yet */
    CHECK(SD_Card_SaveQuatLock(0) == SD_CARD_ERROR);              /* 0 = unlocked */
    CHECK(SD_Card_SaveQuatLock(9) == SD_CARD_ERROR);              /* out of range */
    CHECK(SD_Card_SaveQuatLock(5) == SD_CARD_OK);
    CHECK(SD_Card_LoadQuatLock(&conv) == SD_CARD_OK);
    CHECK(conv == 5);

    /* Corrupt the checksum: rejected */
    lfs_file_t f;
    uint8_t raw[QLOCK_FILE_LEN];
    CHECK(lfs_file_opencfg(&lfs, &f, QLOCK_FILENAME, LFS_O_RDONLY,
                           &adhoc_file_cfg) == 0);
    CHECK(lfs_file_read(&lfs, &f, raw, QLOCK_FILE_LEN) == QLOCK_FILE_LEN);
    lfs_file_close(&lfs, &f);
    raw[QLOCK_FILE_LEN - 1] ^= 0x01;
    CHECK(lfs_file_opencfg(&lfs, &f, QLOCK_FILENAME,
                           LFS_O_WRONLY | LFS_O_TRUNC, &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, raw, QLOCK_FILE_LEN);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadQuatLock(&conv) == SD_CARD_ERROR);
}

TEST(test_deferred_sync_powercut_staleness_bound)
{
    /* Deferred-sync model contract (post-0e28d47): what survives a power
     * cut is whatever was SYNCED, and the unsynced tail is bounded.
     * Simulate the cut by unmounting the fs without a file close/flush -
     * the lfs cache state is what the card physically holds at that
     * moment. Pins both halves of the bound. */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.9f; beacon.longitude = -105.1f;

    /* Row 1 lands and is flushed (its tick value marks it) */
    Test_SetTick(1000);
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 0.1f, 1.0f, 0.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);
    CHECK(sd_flush() == SD_CARD_OK);

    /* Rows 2-6 go in with NO flush - the deferred-sync window */
    for (int i = 0; i < 5; i++) {
        Test_SetTick(2000 + (uint32_t)i * 100);
        CHECK(SD_Card_LogNavigation(&beacon, NULL, 0.2f + (float)i, 2.0f,
                                    0.0f, 0.0f, 0.0f, -90, 5) == SD_CARD_OK);
    }

    /* Power cut: unmount without close/flush */
    CHECK(lfs_unmount(&lfs) == 0);
    lfs_mounted = 0;
    log_file_open = 0;
    sd_initialized = 0;

    /* Remount fresh module state over the surviving image (no close -
     * the dirty cache was genuinely lost, like the power cut did). */
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* The flushed row from BEFORE the cut lives in the original file
     * (a new session-file would start a fresh L0002.TXT) */
    char big[4096];
    CHECK(read_file("L0001.TXT", big, sizeof(big)) > 0);
    CHECK(strstr(big, ",0.100,") != NULL);    /* flushed row survives */
    CHECK(strstr(big, ",0.200,") == NULL);    /* unsynced tail gone     */
}

TEST(test_self_test_pass_and_write_failure)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_SelfTest() == SD_CARD_OK);

    uint8_t step, fres;
    SD_Card_GetSelfTestError(&step, &fres);
    CHECK(step == 7 && fres == 0);            /* reached the final step */

    /* Write failure is reported with the failing step + lfs errno
     * (open with O_CREAT already needs a metadata write, so step 1) */
    fake_sd.read_only = 1;
    CHECK(SD_Card_SelfTest() == SD_CARD_ERROR);
    SD_Card_GetSelfTestError(&step, &fres);
    CHECK(step >= 1 && fres != 0);
    fake_sd.read_only = 0;
}

TEST(test_rotation_launchflag_and_write_failure)
{
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* Opening a second log file closes the first (sd_card.c:241-242) */
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(log_file_open == 1);
    CHECK(sd_open_new_log_file() == SD_CARD_OK);           /* rotation */
    CHECK(log_file_open == 1);

    /* Launch flag without fused data: FusedFlags column shows 0x10 (:481) */
    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.9f; beacon.longitude = -105.1f;
    beacon.launch_detected = 1;                            /* is_fused = 0 */
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 10.0f, 20.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);

    /* Landed latch on the raw (non-fused) stream adds 0x20 to the flags */
    beacon.fused_landed = 1;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 11.0f, 20.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);

    CHECK(sd_flush() == SD_CARD_OK);
    char big[2048];
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, "NAV,GPS,") != NULL);
    CHECK(strstr(big, ",10,") != NULL);                  /* FusedFlags %02X */
    CHECK(strstr(big, ",30,") != NULL);                  /* 0x10 launch | 0x20 landed */

    /* NOTE: the littlefs-failure branches (mount-after-format, opencfg /
     * header-write / mid-session write errors) are correct defensive error
     * handling but can't be driven faithfully through the card fake:
     * a failed prog leaves littlefs' program cache dirty, and the next
     * operation trips LFS_ASSERT(pcache->block == LFS_BLOCK_NULL), and the
     * self-test readback-mismatch branch can't be driven either (metadata
     * pairs relocate off blocks 0-1, so targeted garbage reads corrupt the
     * mount, not just the test file). Kept as defensive code; covered by
     * review, not by behavioral test. */
}

TEST(test_write_recovers_lost_mount)
{
    wipe_card();
    Test_SetTick(1000);
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);

    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.9f; beacon.longitude = -105.1f;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 10.0f, 20.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);
    CHECK(sd_flush() == SD_CARD_OK);

    /* Simulate a recovered contact bounce: mount + handle gone, block
     * device healthy again, littlefs state clean. (A prog-failure
     * mid-write is NOT simulated here - a failed prog leaves littlefs'
     * cache dirty, see the note in test_rotation_launchflag_and_write_failure.)
     * Statics are visible because this file #includes sd_card.c. */
    CHECK(lfs_file_close(&lfs, &log_file) == 0);
    log_file_open = 0;
    CHECK(lfs_unmount(&lfs) == 0);
    lfs_mounted = 0;
    sd_info.is_mounted = 0;

    /* Next write climbs the recovery ladder (mount + append reopen) and lands */
    Test_SetTick(60000);
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 11.0f, 21.0f, 0.0f, 0.0f, -88, 6)
          == SD_CARD_OK);
    CHECK(lfs_mounted == 1 && log_file_open == 1);

    /* Backoff: lose the mount again immediately - recovery is rate-limited
     * (SD_RECOVERY_BACKOFF_MS) so this write fails fast instead of
     * thrashing the card on every entry... */
    CHECK(lfs_file_close(&lfs, &log_file) == 0);
    log_file_open = 0;
    CHECK(lfs_unmount(&lfs) == 0);
    lfs_mounted = 0;
    sd_info.is_mounted = 0;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 12.0f, 22.0f, 0.0f, 0.0f, -88, 6)
          == SD_CARD_ERROR);
    CHECK(lfs_mounted == 0);

    /* ...and recovers once the backoff has elapsed */
    Test_SetTick(60000 + SD_RECOVERY_BACKOFF_MS + 1000);
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 1.0f, 13.0f, 23.0f, 0.0f, 0.0f, -88, 6)
          == SD_CARD_OK);
    CHECK(lfs_mounted == 1 && log_file_open == 1);

    /* The recovered rows are in the file; the backoff-denied row is not */
    CHECK(sd_flush() == SD_CARD_OK);
    char big[2048];
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, ",11.0,") != NULL);
    CHECK(strstr(big, ",12.0,") == NULL);
    CHECK(strstr(big, ",13.0,") != NULL);
}

TEST(test_dirty_card_boot_preserves_history)
{
    /* Launch-day case: card is NOT freshly formatted - it already carries
     * several past sessions plus persistence files. Boot must not wipe or
     * renumber anything; the new session must land in max_seq+1. */
    wipe_card();
    Test_SetTick(1000);
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* Simulate three prior boot sessions by hand-rolling the files */
    lfs_file_t f;
    for (int i = 1; i <= 3; i++) {
        char name[16], body[32];
        snprintf(name, sizeof(name), "L%04d.TXT", i);
        snprintf(body, sizeof(body), "session-%d-payload\n", i);
        CHECK(lfs_file_opencfg(&lfs, &f, name, LFS_O_WRONLY | LFS_O_CREAT,
                               &adhoc_file_cfg) == 0);
        lfs_file_write(&lfs, &f, body, strlen(body));
        lfs_file_close(&lfs, &f);
    }
    /* and the persistence files a reused card already holds */
    CHECK(lfs_file_opencfg(&lfs, &f, "QLOCK.BIN", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "\x02\x00", 2);
    lfs_file_close(&lfs, &f);
    CHECK(lfs_file_opencfg(&lfs, &f, "BEACON.TXT", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "{\"callsign\":\"KE0MZS\"}", 21);
    lfs_file_close(&lfs, &f);

    /* Cycle without reformat - dirty card boots clean-init */
    sd_deinit();
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* New log goes to L0004, not a renumber or a reuse */
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(strcmp(sd_info.current_log_file, "L0004.TXT") == 0);

    /* Old content must be byte-identical after the new session starts */
    sd_flush();
    char buf[512];
    char small[64];
    CHECK(read_file("L0001.TXT", small, sizeof(small)) > 11);
    CHECK(strstr(small, "session-1-payload") != NULL);
    CHECK(read_file("QLOCK.BIN", small, sizeof(small)) >= 1);
    CHECK(read_file("BEACON.TXT", small, sizeof(small)) >= 10);

    /* The new log is open and writable */
    CHECK(SD_Card_LogEvent("dirty-card boot event") == SD_CARD_OK);
    CHECK(sd_flush() == SD_CARD_OK);
    int nr = read_file("L0004.TXT", buf, sizeof(buf));
    CHECK(nr > 10);
    CHECK(strstr(buf, "dirty-card boot event") != NULL);
}

TEST(test_declination_loader)
{
    wipe_card();
    Test_SetTick(1000);
    CHECK(SD_Card_Init() == SD_CARD_OK);

    /* Absent file: default survives */
    float deg = -999.0f;
    CHECK(SD_Card_LoadDeclination(&deg) == SD_CARD_ERROR);
    CHECK(deg == -999.0f);

    /* Valid site override (e.g., mid-Nevada) */
    lfs_file_t f;
    CHECK(lfs_file_opencfg(&lfs, &f, "DECLIN.TXT", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "11.9\n", 5);
    lfs_file_close(&lfs, &f);
    CHECK(SD_Card_LoadDeclination(&deg) == SD_CARD_OK);
    CHECK(deg > 11.89f && deg < 11.91f);

    /* Garbage content must not override */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(lfs_file_opencfg(&lfs, &f, "DECLIN.TXT", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "garbage", 7);
    lfs_file_close(&lfs, &f);
    deg = -999.0f;
    CHECK(SD_Card_LoadDeclination(&deg) == SD_CARD_ERROR);
    CHECK(deg == -999.0f);

    /* Out of physical range is rejected too */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(lfs_file_opencfg(&lfs, &f, "DECLIN.TXT", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "95.0", 4);
    lfs_file_close(&lfs, &f);
    deg = -999.0f;
    CHECK(SD_Card_LoadDeclination(&deg) == SD_CARD_ERROR);
    CHECK(deg == -999.0f);

    /* "nan" parses successfully (endp advances) and slips the range check
     * since NaN comparisons are false - must be rejected explicitly. */
    wipe_card();
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(lfs_file_opencfg(&lfs, &f, "DECLIN.TXT", LFS_O_WRONLY | LFS_O_CREAT,
                           &adhoc_file_cfg) == 0);
    lfs_file_write(&lfs, &f, "nan", 3);
    lfs_file_close(&lfs, &f);
    deg = -999.0f;
    CHECK(SD_Card_LoadDeclination(&deg) == SD_CARD_ERROR);
    CHECK(deg == -999.0f);
}

/* Occurrences of needle in buf[0..n), NUL bytes treated as ordinary data */
static int count_in(const char *buf, int n, const char *needle)
{
    int k = 0, m = (int)strlen(needle);
    for (int i = 0; i + m <= n; i++) if (memcmp(buf + i, needle, (size_t)m) == 0) k++;
    return k;
}

TEST(test_new_log_file_fault_sweep)
{
    /* The card dies after 1, 2, 3... more programs while a session file
     * is being created (directory commit, then the header sync): every
     * failure must leave no half-open handle and no session name, and a
     * healthy card afterwards must still get a file. */
    int failures = 0, created = 0;
    for (int n = 1; n <= 16 && !created; n++) {
        wipe_card();
        CHECK(SD_Card_Init() == SD_CARD_OK);
        fake_sd.write_fail_from = fake_sd.writes + (uint32_t)n;
        SD_Card_Status st = SD_Card_EnsureLogFile();
        fake_sd.write_fail_from = 0;
        if (st == SD_CARD_OK) {
            created = 1;
        } else {
            failures++;
            CHECK(log_file_open == 0);
            CHECK(sd_info.current_log_file[0] == '\0');
        }
    }
    CHECK(failures >= 2);          /* both the open and the sync can fail */
    CHECK(created == 1);
}

TEST(test_recovery_when_session_file_vanished)
{
    /* The ladder's full climb: the handle is lost AND the session file is
     * gone, so the append-reopen fails, the mount is cycled, and the
     * reopen fails again - EnsureLogFile reports the error (it never
     * starts a second file mid-session), and the next write is refused
     * while the recovery backoff holds. */
    wipe_card();
    Test_SetTick(100000);
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(SD_Card_LogEvent("row in the doomed file") == SD_CARD_OK);
    lfs_file_close(&lfs, &log_file);
    log_file_open = 0;
    CHECK(lfs_remove(&lfs, sd_info.current_log_file) == 0);
    last_recovery_ms = 0;
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_ERROR);
    CHECK(lfs_mounted == 1);                      /* remounted fine */
    CHECK(log_file_open == 0);
    CHECK(SD_Card_LogEvent("refused during backoff") == SD_CARD_ERROR);
}

TEST(test_io_error_retry_keeps_the_row)
{
    /* The third consecutive write error steps the bus down and logs it.
     * That diagnostic used to be formatted into the same static log_buffer
     * the failing row lived in, so the retry wrote the diagnostic (with the
     * row's length: stale bytes and a NUL) - the row itself was lost while
     * the call still returned OK. */
    wipe_card();
    fake_sd.min_good_div = 4;              /* a board whose bus holds /4 */
    Test_SetTick(100000);
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(fake_sd.div == 4);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(SD_Card_LogEvent("baseline row") == SD_CARD_OK);
    sd_io_err_streak = 0;
    fake_sd.stepdowns = 0;

    /* The bus goes marginal at /4 (it now needs /8), reads and writes
     * alike. Long rows until the card is needed (the small log file moves
     * out of the metadata inline area) fail twice in a row - recovery held
     * off by its backoff - then a third, long enough to need the card too,
     * fails with recovery allowed. */
    fake_sd.min_good_div = 8;
    char row[200];
    for (int i = 0; i < 80 && sd_io_err_streak < 2; i++) {
        last_recovery_ms = HAL_GetTick();
        snprintf(row, sizeof(row), "ROW%02d-%0140d", i, 0);
        (void)SD_Card_LogEvent(row);
    }
    CHECK(sd_io_err_streak == 2);
    Test_SetTick(200000);
    last_recovery_ms = 0;
    snprintf(row, sizeof(row), "CRITICAL-ROW-THAT-MUST-SURVIVE-%0140d", 0);
    CHECK(SD_Card_LogEvent(row) == SD_CARD_OK);
    CHECK(fake_sd.stepdowns == 1);
    CHECK(fake_sd.div == 8);               /* one rung slower: works again */
    CHECK(sd_flush() == SD_CARD_OK);

    static char big[16384];
    int n = read_file(sd_info.current_log_file, big, sizeof(big));
    CHECK(n > 0);
    CHECK(memchr(big, '\0', (size_t)n) == NULL);          /* no NUL bytes */
    CHECK(count_in(big, n, "CRITICAL-ROW-THAT-MUST-SURVIVE") == 1);
    CHECK(count_in(big, n, "SD bus stepped down") == 1);
}

TEST(test_failed_sync_counts_and_recovers)
{
    /* lfs_file_sync failures were ignored: no error count, no recovery,
     * and the rows they carried silently vanished. */
    wipe_card();
    Test_SetTick(100000);
    CHECK(SD_Card_Init() == SD_CARD_OK);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(SD_Card_LogEvent("row before the bad sync") == SD_CARD_OK);
    uint32_t errs = sd_info.write_errors;
    last_recovery_ms = 0;

    fake_sd.read_only = 1;
    Test_SetTick(100000 + SD_SYNC_TIME_CAP_MS + 1);   /* force a commit */
    SD_Card_ServiceSync(1);
    fake_sd.read_only = 0;
    CHECK(sd_info.write_errors == errs + 1);          /* counted */
    CHECK(sd_dirty == 1);                             /* still pending */

    /* The handle was re-established: logging carries on */
    CHECK(SD_Card_LogEvent("row after recovery") == SD_CARD_OK);
    CHECK(sd_flush() == SD_CARD_OK);
    static char big[4096];
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, "row after recovery") != NULL);
    (void)SD_Card_TakeSyncCount();                    /* leave no residue */
}

TEST(test_io_error_streak_steps_bus_down)
{
    /* sd_card.c #include'd: drive the streak helper directly. Three write
     * errors in a row must ask the diskio ladder to step down once; a good
     * write resets the streak (so isolated transients don't throttle). */
    fake_sd.stepdowns = 0;
    sd_io_err_streak = 0;
    sd_note_write_err();
    sd_note_write_err();
    CHECK(fake_sd.stepdowns == 0);            /* no throttle on 2 transients */
    sd_io_err_streak = 0;                  /* good write in between */
    sd_note_write_err();
    sd_note_write_err();
    sd_note_write_err();
    CHECK(fake_sd.stepdowns == 1);            /* 3 consecutive -> one step */
}

TEST(test_io_stepup_requires_streak_and_cooldown)
{
    /* Asymmetric: step-up waits (a) 60 consecutive OK writes and (b) a
     * 2-minute cooldown since the last step-down. An incident shouldn't
     * see an instant re-fast. The bus here holds /4; an incident slowed
     * it to /8. */
    wipe_card();
    fake_sd.min_good_div = 4;
    Test_SetTick(100000);
    CHECK(SD_Card_Init() == SD_CARD_OK && fake_sd.div == 4);
    CHECK(SD_Card_EnsureLogFile() == SD_CARD_OK);
    CHECK(SD_StepSpeedDown() == 8);
    fake_sd.stepup_tries = 0;
    sd_io_ok_streak = 0;
    sd_last_stepdown_ms = 100000;

    /* Cooldown not met: 62 clean writes 100 ms after the step-down */
    Test_SetTick(100100);
    for (int i = 0; i < 62; i++) sd_note_write_ok();
    CHECK(fake_sd.stepup_tries == 0 && fake_sd.div == 8);

    /* Cooldown met: the next clean write tries one rung up and keeps it */
    Test_SetTick(100000 + SD_IO_STEPUP_COOLDOWN_MS + 1000);
    sd_note_write_ok();
    CHECK(fake_sd.stepup_tries == 1 && fake_sd.div == 4);
    /* Another 120 back-to-back: the try opened a new quiet window */
    for (int i = 0; i < 120; i++) sd_note_write_ok();
    CHECK(fake_sd.stepup_tries == 1);

    CHECK(sd_flush() == SD_CARD_OK);
    static char big[4096];
    int n = read_file(sd_info.current_log_file, big, sizeof(big));
    char row[64];
    snprintf(row, sizeof(row), ",EVENT,SD bus stepped up to /4 after %u clean writes\n",
             (unsigned)SD_IO_OK_STEPUP_STREAK);
    CHECK(n > 0 && count_in(big, n, row) == 1);
}

TEST(test_timestamp_format)
{
    char ts[32];
    Test_SetTick(83456);
    SD_Card_GetTimestamp(ts, sizeof(ts));
    CHECK(strcmp(ts, "83.456") == 0);
    SD_Card_GetTimestamp(NULL, 0);            /* must not crash */

    /* Internal guard: WriteLogEntry with no mounted fs (the public APIs all
     * guard earlier, so this is only reachable from inside the module) */
    sd_deinit();
    CHECK(SD_Card_WriteLogEntry("x") == SD_CARD_ERROR);
}

/* ------------------------------------------------------------------ */

TEST(test_big_sync_burst_logged_once)
{
    /* A >500 ms lfs_file_sync leaves a forensic "SD sync burst Nms" event
     * row so post-flight analysis can attribute main-loop stalls. Drive
     * the duration by auto-advancing the tick across the sync window. */
    wipe_card();
    Test_SetTick(1000);
    CHECK(SD_Card_Init() == SD_CARD_OK);

    GPS_Data beacon; memset(&beacon, 0, sizeof(beacon));
    beacon.latitude = 39.9f; beacon.longitude = -105.1f;
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 0.1f, 1.0f, 0.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);
    CHECK(sd_flush() == SD_CARD_OK);
    CHECK(sd_dirty == 0);

    /* Dirty a row, then force the sync; every HAL_GetTick call inside
     * ServiceSync advances 600 ms so the measured window exceeds 500. */
    Test_SetTick(5000);
    CHECK(SD_Card_LogNavigation(&beacon, NULL, 0.2f, 2.0f, 0.0f,
                                0.0f, 0.0f, -90, 5) == SD_CARD_OK);
    sd_dirty_rows = SD_SYNC_ROW_CAP;            /* force, don't wait idle */
    Test_SetTickAutoAdvance(600);
    SD_Card_ServiceSync(0);
    Test_SetTickAutoAdvance(0);

    /* The burst event was queued as a row; flush and look for it. */
    CHECK(sd_flush() == SD_CARD_OK);
    char big[4096];
    CHECK(read_file(sd_info.current_log_file, big, sizeof(big)) > 0);
    CHECK(strstr(big, "SD sync burst") != NULL);

    /* TakeSyncCount returns syncs-since-last-call and clears */
    CHECK(SD_Card_TakeSyncCount() == 1);
    CHECK(SD_Card_TakeSyncCount() == 0);
}

int main(void)
{
    Test_SetTick(1000);

    run_test_init_bd_failure_codes();
    run_test_init_blank_card_formats_and_mounts();
    run_test_init_format_failure_paths();
    run_test_init_never_formats_a_card_holding_littlefs();
    run_test_remount_preserves_files();
    run_test_log_file_lazy_creation_and_sequencing();
    run_test_all_log_row_formats();
    run_test_raw_nav_row_flags_carry_beacon_fix_type();
    run_test_apis_reject_when_uninitialized();
    run_test_beacon_persistence_validation();
    run_test_last_channel_persistence();
    run_test_quatlock_persistence();
    run_test_deferred_sync_powercut_staleness_bound();
    run_test_self_test_pass_and_write_failure();
    run_test_rotation_launchflag_and_write_failure();
    run_test_write_recovers_lost_mount();
    run_test_dirty_card_boot_preserves_history();
    run_test_declination_loader();
    run_test_new_log_file_fault_sweep();
    run_test_recovery_when_session_file_vanished();
    run_test_io_error_retry_keeps_the_row();
    run_test_failed_sync_counts_and_recovers();
    run_test_io_error_streak_steps_bus_down();
    run_test_io_stepup_requires_streak_and_cooldown();
    run_test_timestamp_format();
    run_test_big_sync_burst_logged_once();

    return TEST_SUMMARY();
}
