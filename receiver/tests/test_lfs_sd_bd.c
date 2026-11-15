/**
 * test_lfs_sd_bd.c - the LittleFS block device over the SD sector driver
 *                    (lfs_sd_bd.c)
 *
 * Every flight log reaches the card through this file: LittleFS's
 * (block, off, size) callbacks onto sd_diskio.c's sector reads and writes,
 * the bounded retries, and the card geometry. test_sd_card.c runs sd_card.c
 * over a RAM block device of its own, so until now this layer was covered
 * by the firmware compile alone.
 *
 * The card is faked at the sd_diskio API with sd_diskio.c's contract:
 * RES_PARERR for a zero count, RES_NOTRDY before SD_initialize, LBA
 * (sector) addressing, 512-byte sectors, and an address past the end
 * rejected. The real LittleFS runs on top with the firmware's real
 * geometry (4 KiB blocks and caches, 128-byte lookahead).
 */
#include <stdio.h>
#include <string.h>

#include "../firmware/src/lfs_sd_bd.c"
#include "test_harness.h"

/* ---- sd_diskio fakes: a 4 MiB card --------------------------------- */
#define CARD_SECTORS (4u * 2048u)     /* 1024 LittleFS blocks */
static uint8_t card[CARD_SECTORS * 512u];

uint8_t SD_Type = 0;                  /* 0 until SD_initialize succeeds */
static DSTATUS fake_init_status  = 0;
static DRESULT fake_count_result = RES_OK;
static DWORD   fake_sector_count = CARD_SECTORS;
static int     fast_speed_calls  = 0;
static int     csd_after_fast    = 0;   /* CMD9 issued at the fast clock */
static unsigned long reads = 0, writes = 0;
static int     read_fails = 0, write_fails = 0;   /* next N calls fail */
static int     write_fail_all = 0;
static DWORD   last_sector = 0;
static UINT    last_count = 0;

DSTATUS SD_initialize(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    SD_Type = fake_init_status ? 0 : 12;          /* SDHC: block addressed */
    return fake_init_status;
}

DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    if (cmd != GET_SECTOR_COUNT) return RES_PARERR;
    if (fast_speed_calls) csd_after_fast = 1;
    if (fake_count_result != RES_OK) return fake_count_result;
    *(DWORD *)buff = fake_sector_count;
    return RES_OK;
}

void SD_SetFastSpeed(void) { fast_speed_calls++; }

DRESULT SD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    reads++;
    last_sector = sector;
    last_count = count;
    if (read_fails > 0) { read_fails--; return RES_ERROR; }
    if (sector + count > CARD_SECTORS) return RES_ERROR;   /* R1 address error */
    memcpy(buff, &card[sector * 512u], count * 512u);
    return RES_OK;
}

DRESULT SD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    writes++;
    last_sector = sector;
    last_count = count;
    if (write_fail_all) return RES_ERROR;
    if (write_fails > 0) { write_fails--; return RES_ERROR; }
    if (sector + count > CARD_SECTORS) return RES_ERROR;
    memcpy(&card[sector * 512u], buff, count * 512u);
    return RES_OK;
}

static void reset_card(void)
{
    memset(card, 0xFF, sizeof(card));
    SD_Type = 0;
    fake_init_status = 0;
    fake_count_result = RES_OK;
    fake_sector_count = CARD_SECTORS;
    fast_speed_calls = csd_after_fast = 0;
    reads = writes = 0;
    read_fails = write_fails = write_fail_all = 0;
    last_sector = 0;
    last_count = 0;
}

/* The log file's cache, as sd_card.c supplies it (cache_size bytes) */
static uint8_t file_cache[4096];
static const struct lfs_file_config file_cfg = { .buffer = file_cache };

TEST(test_init_failures_leave_no_card)
{
    reset_card();
    fake_init_status = STA_NOINIT;                 /* CMD0/ACMD41 failed */
    CHECK(lfs_sd_bd_init() == -1);
    CHECK(lfs_sd_bd_capacity_mb() == 0);

    reset_card();
    fake_count_result = RES_ERROR;                 /* CMD9 failed */
    CHECK(lfs_sd_bd_init() == -2);
    CHECK(lfs_sd_bd_capacity_mb() == 0);

    reset_card();
    fake_sector_count = 0;                         /* CSD claims an empty card */
    CHECK(lfs_sd_bd_init() == -2);
    CHECK(lfs_sd_bd_capacity_mb() == 0);
    CHECK(fast_speed_calls == 0);                  /* no speed-up for a dead card */
}

TEST(test_init_sizes_the_filesystem_to_the_card)
{
    reset_card();
    fake_sector_count = 31116288u;                 /* a "16 GB" SDHC card */
    CHECK(lfs_sd_bd_init() == 0);
    CHECK(lfs_sd_cfg.block_count == 31116288u / 8u);
    CHECK(lfs_sd_bd_capacity_mb() == 15193u);
    /* CMD9 runs at the slow init clock, then the bus speeds up, once */
    CHECK(fast_speed_calls == 1 && csd_after_fast == 0);

    /* A tail shorter than a block is never addressed */
    reset_card();
    fake_sector_count = CARD_SECTORS + 5u;
    CHECK(lfs_sd_bd_init() == 0);
    CHECK(lfs_sd_cfg.block_count == CARD_SECTORS / 8u);

    /* A later failed init forgets the old card */
    fake_init_status = STA_NOINIT;
    CHECK(lfs_sd_bd_init() == -1);
    CHECK(lfs_sd_bd_capacity_mb() == 0);
}

TEST(test_blocks_map_onto_sectors)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    uint8_t buf[1536];

    for (unsigned i = 0; i < 1536u; i++) card[26u * 512u + i] = (uint8_t)i;
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 3, 1024, buf, 1536) == 0);
    CHECK(last_sector == 3u * 8u + 2u && last_count == 3);
    CHECK(memcmp(buf, &card[26u * 512u], 1536) == 0);

    /* The card's last sector: block 1023, the block's last 512 bytes */
    memset(buf, 0xA5, 512);
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 1023, 3584, buf, 512) == 0);
    CHECK(last_sector == CARD_SECTORS - 1u && last_count == 1);
    CHECK(card[(CARD_SECTORS - 1u) * 512u] == 0xA5);

    /* The card's own FTL erases; a sync has nothing to flush */
    unsigned long r = reads, w = writes;
    CHECK(lfs_sd_cfg.erase(&lfs_sd_cfg, 5) == 0);
    CHECK(lfs_sd_cfg.sync(&lfs_sd_cfg) == 0);
    CHECK(reads == r && writes == w);
}

TEST(test_misaligned_requests_never_reach_the_card)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    uint8_t buf[1024];
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 0, 100, buf, 512) == LFS_ERR_IO);
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 0, 0, buf, 100) == LFS_ERR_IO);
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 0, 100, buf, 512) == LFS_ERR_IO);
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 0, 0, buf, 700) == LFS_ERR_IO);
    CHECK(reads == 0 && writes == 0);
}

TEST(test_transient_errors_are_retried_a_bounded_number_of_times)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    uint8_t buf[512];

    read_fails = LFS_SD_BD_RETRIES - 1;            /* the last try succeeds */
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 1, 0, buf, 512) == 0);
    CHECK(reads == LFS_SD_BD_RETRIES);

    reads = 0;
    read_fails = LFS_SD_BD_RETRIES;                /* a dead card surfaces */
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 1, 0, buf, 512) == LFS_ERR_IO);
    CHECK(reads == LFS_SD_BD_RETRIES);

    memset(buf, 0x5A, sizeof(buf));
    write_fails = LFS_SD_BD_RETRIES - 1;
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 2, 0, buf, 512) == 0);
    CHECK(writes == LFS_SD_BD_RETRIES);
    CHECK(card[16u * 512u] == 0x5A);

    /* A write the card keeps refusing is an I/O error for the caller to
     * handle, not a bad block: the card's FTL remaps its own flash, and
     * LFS_ERR_CORRUPT sends LittleFS hunting for another block instead. */
    writes = 0;
    write_fails = LFS_SD_BD_RETRIES;
    buf[0] = 0x11;
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 3, 0, buf, 512) == LFS_ERR_IO);
    CHECK(writes == LFS_SD_BD_RETRIES);
    CHECK(card[24u * 512u] == 0xFF);               /* nothing landed */
}

TEST(test_real_geometry_round_trip)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    lfs_t lfs;
    CHECK(lfs_format(&lfs, &lfs_sd_cfg) == 0);

    /* The superblock where sd_card.c's no-autoformat probe and the host
     * extractor (lfs_extract: block b at byte b * 4096) both look for it:
     * "littlefs" at byte 8 of block 0 or block 1 */
    CHECK(memcmp(&card[8], "littlefs", 8) == 0 ||
          memcmp(&card[8u * 512u + 8u], "littlefs", 8) == 0);

    CHECK(lfs_mount(&lfs, &lfs_sd_cfg) == 0);
    lfs_file_t f;
    CHECK(lfs_file_opencfg(&lfs, &f, "L0001.TXT",
                           LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &file_cfg) == 0);
    char row[96];
    for (int i = 0; i < 200; i++) {                /* ~3.5 blocks of rows */
        int n = snprintf(row, sizeof(row), "%06d,EVENT,row %d of a session\n", i, i);
        CHECK(lfs_file_write(&lfs, &f, row, (lfs_size_t)n) == n);
    }
    CHECK(lfs_file_close(&lfs, &f) == 0);
    CHECK(lfs_unmount(&lfs) == 0);

    /* Power cycle: a fresh mount reads every row back */
    CHECK(lfs_mount(&lfs, &lfs_sd_cfg) == 0);
    CHECK(lfs_file_opencfg(&lfs, &f, "L0001.TXT", LFS_O_RDONLY, &file_cfg) == 0);
    int bad = 0;
    for (int i = 0; i < 200; i++) {
        char got[96];
        int n = snprintf(row, sizeof(row), "%06d,EVENT,row %d of a session\n", i, i);
        if (lfs_file_read(&lfs, &f, got, (lfs_size_t)n) != n || memcmp(got, row, (size_t)n) != 0)
            bad++;
    }
    CHECK(bad == 0);
    CHECK(lfs_file_close(&lfs, &f) == 0);
    CHECK(lfs_unmount(&lfs) == 0);
}

TEST(test_persistent_write_failure_surfaces_promptly)
{
    /* A card that stops taking writes mid-session (worn out to read-only,
     * a bus that went marginal at the current clock). sd_card.c meets the
     * error with its write-error count, the bus step-down and the handle
     * recovery - but only once LittleFS hands it the error. */
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    lfs_t lfs;
    CHECK(lfs_format(&lfs, &lfs_sd_cfg) == 0);
    CHECK(lfs_mount(&lfs, &lfs_sd_cfg) == 0);
    lfs_file_t f;
    CHECK(lfs_file_opencfg(&lfs, &f, "L0001.TXT",
                           LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &file_cfg) == 0);
    char row[600];
    memset(row, 'x', sizeof(row));
    row[sizeof(row) - 1] = '\n';
    for (int i = 0; i < 10; i++) {
        CHECK(lfs_file_write(&lfs, &f, row, sizeof(row)) == (lfs_ssize_t)sizeof(row));
    }
    CHECK(lfs_file_sync(&lfs, &f) == 0);

    write_fail_all = 1;
    writes = 0;
    int err = 0;
    for (int i = 0; i < 20 && err == 0; i++) {     /* > 2 blocks: must reach the card */
        lfs_ssize_t w = lfs_file_write(&lfs, &f, row, sizeof(row));
        if (w < 0) err = (int)w;
    }
    if (err == 0) err = lfs_file_sync(&lfs, &f);
    /* One retry burst, not a lap of the card: LFS_ERR_CORRUPT here had
     * LittleFS relocate through every free block (3 writes each) and then
     * report LFS_ERR_NOSPC - hours on a 16 GB card, so the 32 s IWDG reset
     * the receiver instead. */
    CHECK(err == LFS_ERR_IO);
    CHECK(writes <= 2u * LFS_SD_BD_RETRIES);
    printf("  %lu SD writes before the error (%d)\n", writes, err);
}

int main(void)
{
    run_test_init_failures_leave_no_card();
    run_test_init_sizes_the_filesystem_to_the_card();
    run_test_blocks_map_onto_sectors();
    run_test_misaligned_requests_never_reach_the_card();
    run_test_transient_errors_are_retried_a_bounded_number_of_times();
    run_test_real_geometry_round_trip();
    run_test_persistent_write_failure_surfaces_promptly();
    return TEST_SUMMARY();
}
