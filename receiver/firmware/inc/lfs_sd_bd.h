/**
 ******************************************************************************
 * @file    lfs_sd_bd.h
 * @brief   LittleFS block-device wrapper over the SD-SPI sector driver.
 *
 * Provides a preconfigured struct lfs_config that maps LittleFS's read/prog/
 * erase/sync callbacks onto the existing SD_read/SD_write functions in
 * sd_diskio.c. Also exposes a small handful of helpers used by sd_card.c to
 * initialise the card, probe the card capacity, and format a fresh card when
 * no LittleFS image is found.
 *
 * The SD card's internal FTL handles actual flash erase, so LittleFS "erase"
 * is a no-op at this layer.
 ******************************************************************************
 */

#ifndef LFS_SD_BD_H
#define LFS_SD_BD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lfs.h"

/* The card's LittleFS geometry - the one copy. The firmware's block device,
 * sd_card.c's per-file caches and the host tools that format and extract
 * cards (tools/lfs_extract) all build from these. Changing the block size
 * makes every formatted card unreadable: format them again.
 *   - sector (read/prog) 512 B: the SD sector
 *   - block 4096 B = 8 sectors: the wear-leveling unit. Small enough that
 *     the log-structured metadata overhead stays sane, big enough that the
 *     allocator's view of a multi-GB card stays compact
 *   - cache 4096 B, one full block: each open file carries one, so with the
 *     read + prog caches and two file caches (log + adhoc) LittleFS holds
 *     16 KiB of the STM32F401CC's 64 KiB RAM
 *   - lookahead 128 B: a bitmap of 1024 blocks (4 MiB) per allocator pass */
#define LFS_SD_SECTOR_SIZE        512U
#define LFS_SD_BLOCK_SIZE         4096U
#define LFS_SD_SECTORS_PER_BLOCK  (LFS_SD_BLOCK_SIZE / LFS_SD_SECTOR_SIZE)
#define LFS_SD_CACHE_SIZE         4096U
#define LFS_SD_LOOKAHEAD          128U
#define LFS_SD_BLOCK_CYCLES       500    /* typical SD-card-oriented value */

/* LittleFS checks these in lfs_init() with asserts the firmware compiles
 * out (LFS_NO_ASSERT): check them at compile time instead. */
_Static_assert(LFS_SD_CACHE_SIZE % LFS_SD_SECTOR_SIZE == 0,
               "LittleFS: the cache must be whole sectors");
_Static_assert(LFS_SD_BLOCK_SIZE % LFS_SD_CACHE_SIZE == 0,
               "LittleFS: the cache must divide the block");
_Static_assert(LFS_SD_BLOCK_SIZE >= 128 && LFS_SD_LOOKAHEAD > 0,
               "LittleFS: block and lookahead sizes");

/* Exposed so sd_card.c can pass it to lfs_mount / lfs_format. Populated by
 * lfs_sd_bd_init(). Fields remain valid for the lifetime of the program. */
extern struct lfs_config lfs_sd_cfg;

/**
 * @brief Initialise the SD card and populate lfs_sd_cfg with geometry +
 *        callbacks. Must be called before lfs_mount/lfs_format.
 * @retval 0 on success, negative on SD init failure.
 */
int lfs_sd_bd_init(void);

/**
 * @brief Return card capacity in MiB (for UI display). 0 if unknown.
 */
uint32_t lfs_sd_bd_capacity_mb(void);

#ifdef __cplusplus
}
#endif

#endif /* LFS_SD_BD_H */
