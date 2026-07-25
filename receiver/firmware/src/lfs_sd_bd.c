/**
 ******************************************************************************
 * @file    lfs_sd_bd.c
 * @brief   LittleFS block-device wrapper over SD-SPI sector driver.
 *
 * Why this file exists:
 *   LittleFS wants read/prog/erase/sync callbacks with (block, off, size)
 *   addressing. The existing sd_diskio.c exposes sector-based SD_read/SD_write.
 *   This file adapts the two.
 *
 * Geometry choice (tune here, not in sd_card.c):
 *   - SD sector    = 512 B (hard-coded by SD spec)
 *   - LFS_BLOCK    = 4096 B = 8 sectors. This is the unit of wear-leveling.
 *                    4 KiB is a sweet spot: small enough that the metadata
 *                    overhead of the log-structured layout isn't absurd, big
 *                    enough that allocation bitmap stays compact on multi-GB
 *                    cards.
 *   - LFS_CACHE    = 4096 B (one full block). Must divide block_size evenly.
 *   - LOOKAHEAD    = 128 B bitmap → tracks 1024 blocks = 4 MiB per pass.
 *
 * The SD card's internal FTL remaps and erases flash transparently, so the
 * erase callback is a no-op. LittleFS still uses erase count for wear
 * leveling internally (block_cycles), but that's independent of whether the
 * underlying media needs an explicit erase.
 ******************************************************************************
 */

#include "lfs_sd_bd.h"
#include "sd_diskio.h"
#include "ff_gen_drv.h"   /* DSTATUS, DRESULT, GET_SECTOR_COUNT */
#include <string.h>

/* LittleFS geometry. Changing these on a formatted card will make it
 * unreadable — format again if you tune them. */
#define LFS_SD_SECTOR_SIZE   512U
#define LFS_SD_BLOCK_SIZE    4096U
#define LFS_SD_SECTORS_PER_BLOCK  (LFS_SD_BLOCK_SIZE / LFS_SD_SECTOR_SIZE)  /* 8 */
/* cache_size of one full block minimises per-append program operations.
 * Each open file carries one cache of this size; with 2 built-in caches
 * (read + prog) plus 2 file caches (log + adhoc), total LittleFS RAM for
 * caches is 4 * 4 KiB = 16 KiB. Comfortably fits STM32F401CC's 64 KiB RAM. */
#define LFS_SD_CACHE_SIZE    4096U
#define LFS_SD_LOOKAHEAD     128U

/* Static buffers so LittleFS doesn't need malloc. cache_size * 2 is the read
 * cache + prog cache; each open file adds one more cache of the same size
 * (via lfs_file_opencfg in sd_card.c). */
/* Cache buffers: size must equal cfg.cache_size. */
static uint8_t lfs_read_buf[LFS_SD_CACHE_SIZE];
static uint8_t lfs_prog_buf[LFS_SD_CACHE_SIZE];
static uint8_t lfs_lookahead_buf[LFS_SD_LOOKAHEAD];

static uint8_t  sd_ready = 0;
static uint32_t sd_sector_count = 0;  /* from CMD9 GET_SECTOR_COUNT */

/* --------------------------------------------------------------------------
 * LittleFS block-device callbacks
 * -------------------------------------------------------------------------- */

/* Bounded retry count for SD operations at the block-device layer.
 *
 * Running SX1268-adjacent SPI1 at ~1.3 MHz reliably for mount but occasionally
 * drops a sector read or write under bus contention (transmit burst, OLED
 * I2C activity sharing the same 3V3 rail, etc). A single retry almost always
 * succeeds because it's a fresh CMD sequence with fresh CRC, and the cost
 * of 2-3 retries is trivial compared to the cost of LittleFS concluding the
 * card is corrupt and bailing out. Keep the number small so a genuinely
 * dead card still surfaces quickly. */
#define LFS_SD_BD_RETRIES  3

static int bd_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size)
{
    (void)c;
    /* LittleFS always passes aligned (off,size); we treat them as multiples of
     * the sector size so we can translate directly to SD_read. */
    if ((off % LFS_SD_SECTOR_SIZE) != 0 || (size % LFS_SD_SECTOR_SIZE) != 0) {
        return LFS_ERR_IO;
    }
    DWORD sector = (DWORD)block * LFS_SD_SECTORS_PER_BLOCK
                 + (DWORD)(off / LFS_SD_SECTOR_SIZE);
    UINT  count  = size / LFS_SD_SECTOR_SIZE;
    for (int attempt = 0; attempt < LFS_SD_BD_RETRIES; attempt++) {
        DRESULT r = SD_read(0, (BYTE *)buffer, sector, count);
        if (r == RES_OK) return 0;
    }
    return LFS_ERR_IO;
}

static int bd_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size)
{
    (void)c;
    if ((off % LFS_SD_SECTOR_SIZE) != 0 || (size % LFS_SD_SECTOR_SIZE) != 0) {
        return LFS_ERR_IO;
    }
    DWORD sector = (DWORD)block * LFS_SD_SECTORS_PER_BLOCK
                 + (DWORD)(off / LFS_SD_SECTOR_SIZE);
    UINT  count  = size / LFS_SD_SECTOR_SIZE;
    for (int attempt = 0; attempt < LFS_SD_BD_RETRIES; attempt++) {
        DRESULT r = SD_write(0, (const BYTE *)buffer, sector, count);
        if (r == RES_OK) return 0;
    }
    return LFS_ERR_CORRUPT;
}

static int bd_erase(const struct lfs_config *c, lfs_block_t block)
{
    /* SD cards have their own FTL + internal erase; from our perspective a
     * block is always writable. Return success without touching the card. */
    (void)c;
    (void)block;
    return 0;
}

static int bd_sync(const struct lfs_config *c)
{
    /* SD SPI writes are committed on the trailing data-response token, so
     * there's nothing to flush at this layer. */
    (void)c;
    return 0;
}

/* --------------------------------------------------------------------------
 * Public lfs_config + init
 * -------------------------------------------------------------------------- */

struct lfs_config lfs_sd_cfg = {
    .context         = NULL,
    .read            = bd_read,
    .prog            = bd_prog,
    .erase           = bd_erase,
    .sync            = bd_sync,
    .read_size       = LFS_SD_SECTOR_SIZE,
    .prog_size       = LFS_SD_SECTOR_SIZE,
    .block_size      = LFS_SD_BLOCK_SIZE,
    .block_count     = 0,                /* filled by lfs_sd_bd_init() */
    .block_cycles    = 500,              /* typical SD-card-oriented value */
    .cache_size      = LFS_SD_CACHE_SIZE,
    .lookahead_size  = LFS_SD_LOOKAHEAD,
    .read_buffer     = lfs_read_buf,
    .prog_buffer     = lfs_prog_buf,
    .lookahead_buffer = lfs_lookahead_buf,
};

int lfs_sd_bd_init(void)
{
    sd_ready = 0;
    sd_sector_count = 0;

    /* Bring the card up via the FatFS-era init path (we keep SD_initialize /
     * sd_diskio.c because it already has the hard-won HAL/register hybrid
     * logic for reliable SPI transfers). */
    if (SD_initialize(0) != 0) {
        return -1;
    }

    /* Query total sector count so we can size the LittleFS instance. This is
     * a one-shot CMD9 (17 bytes); running it at the slow 82 kHz init clock
     * costs ~2 ms, so there's no point speeding up the SPI first. Reading
     * the CSD *before* switching speeds also means we don't have to worry
     * about HAL_SPI_DeInit/Init transiently disturbing CMD9 reliability. */
    DWORD sectors = 0;
    if (SD_ioctl(0, GET_SECTOR_COUNT, &sectors) != RES_OK || sectors == 0) {
        return -2;
    }
    sd_sector_count = (uint32_t)sectors;

    /* Switch SPI1 from the ~82 kHz init clock to ~2.6 MHz for data-phase
     * transfers. Without this, every LittleFS commit spends ~50 ms per
     * sector at 82 kHz and the main loop falls behind display updates.
     * At 2.6 MHz the same commit takes ~1.6 ms per sector - a ~30x speedup
     * that restores responsive UI even during post-launch traffic bursts. */
    SD_SetFastSpeed();

    /* Whole-card LittleFS; block_count = floor(sectors / sectors_per_block). */
    lfs_sd_cfg.block_count = sd_sector_count / LFS_SD_SECTORS_PER_BLOCK;

    sd_ready = 1;
    return 0;
}

int lfs_sd_bd_is_ready(void)
{
    return sd_ready;
}

uint32_t lfs_sd_bd_capacity_mb(void)
{
    if (!sd_ready) return 0;
    /* sectors * 512 / (1024*1024) = sectors / 2048 */
    return sd_sector_count / 2048U;
}
