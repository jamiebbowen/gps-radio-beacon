/**
 * test_lfs_sd_bd.c - the LittleFS block device over the SD sector driver
 *                    (lfs_sd_bd.c)
 *
 * Every flight log reaches the card through this file: LittleFS's
 * (block, off, size) callbacks onto sd_diskio.c's sector reads and writes,
 * the bounded retries, and the card geometry. test_sd_card.c used to run
 * sd_card.c over a RAM block device of its own, so this layer was covered
 * by the firmware compile alone.
 *
 * The card is the shared sd_diskio fake (fake_sd_card.h, sd_diskio.c's
 * contract). The real LittleFS runs on top with the firmware's real
 * geometry (4 KiB blocks and caches, 128-byte lookahead).
 *
 * The host tools ride along (tools/lfs_extract, linked in): a card the
 * firmware writes must extract, and a card `make format-card` writes must
 * mount in the firmware - the firmware never formats a card that already
 * holds LittleFS, so a mismatch there is an SD error on launch day.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../firmware/src/lfs_sd_bd.c"
#include "test_harness.h"

int run_extract(const char *device, const char *outdir);   /* lfs_extract.c */
int run_format(const char *device, int force);              /* lfs_format.c */

/* ---- the card: the shared sd_diskio fake, 4 MiB ------------------- */
#include "fake_sd_card.h"

static void reset_card(void)
{
    fake_sd_reset();
    fake_sd_wipe(0xFF);
}

/* The log file's cache, as sd_card.c supplies it (cache_size bytes) */
static uint8_t file_cache[LFS_SD_CACHE_SIZE];
static const struct lfs_file_config file_cfg = { .buffer = file_cache };

TEST(test_init_failures_leave_no_card)
{
    reset_card();
    fake_sd.absent = 1;                    /* CMD0/ACMD41 failed */
    CHECK(lfs_sd_bd_init() == -1);
    CHECK(lfs_sd_bd_capacity_mb() == 0);

    reset_card();
    fake_sd.csd_result = RES_ERROR;        /* CMD9 failed */
    CHECK(lfs_sd_bd_init() == -2);
    CHECK(lfs_sd_bd_capacity_mb() == 0);

    reset_card();
    fake_sd.sector_count = 0;              /* CSD claims an empty card */
    CHECK(lfs_sd_bd_init() == -2);
    CHECK(lfs_sd_bd_capacity_mb() == 0);
    CHECK(fake_sd.fast_speed_calls == 0);  /* no speed-up for a dead card */
}

TEST(test_init_sizes_the_filesystem_to_the_card)
{
    reset_card();
    fake_sd.sector_count = 31116288u;      /* a "16 GB" SDHC card */
    CHECK(lfs_sd_bd_init() == 0);
    CHECK(lfs_sd_cfg.block_count == 31116288u / LFS_SD_SECTORS_PER_BLOCK);
    CHECK(lfs_sd_bd_capacity_mb() == 15193u);
    /* CMD9 runs at the slow init clock, then the bus speeds up, once */
    CHECK(fake_sd.fast_speed_calls == 1 && fake_sd.csd_after_fast == 0);

    /* A tail shorter than a block is never addressed */
    reset_card();
    fake_sd.sector_count = FAKE_SD_SECTORS + 5u;
    CHECK(lfs_sd_bd_init() == 0);
    CHECK(lfs_sd_cfg.block_count == FAKE_SD_SECTORS / LFS_SD_SECTORS_PER_BLOCK);

    /* A later failed init forgets the old card */
    fake_sd.absent = 1;
    CHECK(lfs_sd_bd_init() == -1);
    CHECK(lfs_sd_bd_capacity_mb() == 0);
}

TEST(test_blocks_map_onto_sectors)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    uint8_t buf[1536];

    for (unsigned i = 0; i < 1536u; i++) fake_sd_storage[26u * 512u + i] = (uint8_t)i;
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 3, 1024, buf, 1536) == 0);
    CHECK(fake_sd.last_sector == 3u * 8u + 2u && fake_sd.last_count == 3);
    CHECK(memcmp(buf, &fake_sd_storage[26u * 512u], 1536) == 0);

    /* The card's last sector: block 1023, the block's last 512 bytes */
    memset(buf, 0xA5, 512);
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 1023, 3584, buf, 512) == 0);
    CHECK(fake_sd.last_sector == FAKE_SD_SECTORS - 1u && fake_sd.last_count == 1);
    CHECK(fake_sd_storage[(FAKE_SD_SECTORS - 1u) * 512u] == 0xA5);

    /* The card's own FTL erases; a sync has nothing to flush */
    unsigned long r = fake_sd.reads, w = fake_sd.writes;
    CHECK(lfs_sd_cfg.erase(&lfs_sd_cfg, 5) == 0);
    CHECK(lfs_sd_cfg.sync(&lfs_sd_cfg) == 0);
    CHECK(fake_sd.reads == r && fake_sd.writes == w);
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
    CHECK(fake_sd.reads == 0 && fake_sd.writes == 0);
}

TEST(test_transient_errors_are_retried_a_bounded_number_of_times)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    uint8_t buf[512];

    fake_sd.read_fails = LFS_SD_BD_RETRIES - 1;    /* the last try succeeds */
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 1, 0, buf, 512) == 0);
    CHECK(fake_sd.reads == LFS_SD_BD_RETRIES);

    fake_sd.reads = 0;
    fake_sd.read_fails = LFS_SD_BD_RETRIES;        /* a dead card surfaces */
    CHECK(lfs_sd_cfg.read(&lfs_sd_cfg, 1, 0, buf, 512) == LFS_ERR_IO);
    CHECK(fake_sd.reads == LFS_SD_BD_RETRIES);

    memset(buf, 0x5A, sizeof(buf));
    fake_sd.write_fails = LFS_SD_BD_RETRIES - 1;
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 2, 0, buf, 512) == 0);
    CHECK(fake_sd.writes == LFS_SD_BD_RETRIES);
    CHECK(fake_sd_storage[16u * 512u] == 0x5A);

    /* A write the card keeps refusing is an I/O error for the caller to
     * handle, not a bad block: the card's FTL remaps its own flash, and
     * LFS_ERR_CORRUPT sends LittleFS hunting for another block instead. */
    fake_sd.writes = 0;
    fake_sd.write_fails = LFS_SD_BD_RETRIES;
    buf[0] = 0x11;
    CHECK(lfs_sd_cfg.prog(&lfs_sd_cfg, 3, 0, buf, 512) == LFS_ERR_IO);
    CHECK(fake_sd.writes == LFS_SD_BD_RETRIES);
    CHECK(fake_sd_storage[24u * 512u] == 0xFF);               /* nothing landed */
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
    CHECK(memcmp(&fake_sd_storage[8], "littlefs", 8) == 0 ||
          memcmp(&fake_sd_storage[8u * 512u + 8u], "littlefs", 8) == 0);

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

    fake_sd.read_only = 1;
    fake_sd.writes = 0;
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
    CHECK(fake_sd.writes <= 2u * LFS_SD_BD_RETRIES);
    printf("  %lu SD writes before the error (%d)\n", (unsigned long)fake_sd.writes, err);
}

/* ---- the host tools against the firmware's own cards ---------------- */

static char tmp_image[64], tmp_dir[64];
static void image_save(void)
{
    snprintf(tmp_image, sizeof(tmp_image), "/tmp/lfs_sd_bd_card.XXXXXX");
    int fd = mkstemp(tmp_image);
    CHECK(fd >= 0 && write(fd, fake_sd_storage, sizeof(fake_sd_storage)) == (ssize_t)sizeof(fake_sd_storage));
    close(fd);
}

static void image_load(void)
{
    FILE *f = fopen(tmp_image, "rb");
    CHECK(f && fread(fake_sd_storage, 1, sizeof(fake_sd_storage), f) == sizeof(fake_sd_storage));
    if (f) fclose(f);
}

static void lfs_write_text(lfs_t *lfs, const char *name, const char *text)
{
    lfs_file_t f;
    CHECK(lfs_file_opencfg(lfs, &f, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC,
                           &file_cfg) == 0);
    CHECK(lfs_file_write(lfs, &f, text, (lfs_size_t)strlen(text)) == (lfs_ssize_t)strlen(text));
    CHECK(lfs_file_close(lfs, &f) == 0);
}

TEST(test_a_card_the_firmware_writes_extracts)
{
    reset_card();
    CHECK(lfs_sd_bd_init() == 0);
    lfs_t lfs;
    CHECK(lfs_format(&lfs, &lfs_sd_cfg) == 0);
    CHECK(lfs_mount(&lfs, &lfs_sd_cfg) == 0);
    static char log[3 * LFS_SD_BLOCK_SIZE];       /* spans blocks: CTZ list */
    size_t n = 0;
    for (int i = 0; n + 64 < sizeof(log); i++)
        n += (size_t)snprintf(log + n, sizeof(log) - n, "%d.000,EVENT,row %d\n", i, i);
    lfs_write_text(&lfs, "L0001.TXT", log);
    lfs_write_text(&lfs, "BEACON.TXT", "39.8990000,-104.8850000,1700.0\n");
    CHECK(lfs_unmount(&lfs) == 0);

    image_save();
    snprintf(tmp_dir, sizeof(tmp_dir), "/tmp/lfs_sd_bd_out.XXXXXX");
    CHECK(mkdtemp(tmp_dir) != NULL);
    CHECK(run_extract(tmp_image, tmp_dir) == 0);

    char path[96];
    static char got[sizeof(log)];
    snprintf(path, sizeof(path), "%s/L0001.TXT", tmp_dir);
    FILE *f = fopen(path, "rb");
    size_t r = f ? fread(got, 1, sizeof(got), f) : 0;
    if (f) fclose(f);
    CHECK(r == n && memcmp(got, log, n) == 0);
    unlink(path);
    snprintf(path, sizeof(path), "%s/BEACON.TXT", tmp_dir);
    f = fopen(path, "rb");
    r = f ? fread(got, 1, sizeof(got), f) : 0;
    if (f) fclose(f);
    CHECK(r == 31 && memcmp(got, "39.8990000,-104.8850000,1700.0\n", 31) == 0);
    unlink(path);
    rmdir(tmp_dir);
    unlink(tmp_image);
}

TEST(test_a_card_the_format_tool_writes_mounts)
{
    reset_card();                                 /* a blank card in a reader */
    memset(fake_sd_storage, 0, sizeof(fake_sd_storage));
    image_save();
    CHECK(run_format(tmp_image, 1) == 0);
    image_load();
    unlink(tmp_image);

    CHECK(lfs_sd_bd_init() == 0);                 /* the firmware's own mount */
    lfs_t lfs;
    CHECK(lfs_mount(&lfs, &lfs_sd_cfg) == 0);
    lfs_write_text(&lfs, "L0001.TXT", "0.000,EVENT,first row\n");
    struct lfs_info info;
    CHECK(lfs_stat(&lfs, "L0001.TXT", &info) == 0 && info.size == 22);
    CHECK(lfs_unmount(&lfs) == 0);
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
    run_test_a_card_the_firmware_writes_extracts();
    run_test_a_card_the_format_tool_writes_mounts();
    return TEST_SUMMARY();
}
