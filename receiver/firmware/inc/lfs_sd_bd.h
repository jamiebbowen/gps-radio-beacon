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
 * Geometry:
 *   - read_size  = prog_size = 512 B (SD sector size)
 *   - block_size = 4096 B (8 sectors per LittleFS block — good tradeoff
 *                  between wear-leveling granularity and metadata overhead)
 *   - cache_size = 512 B (one sector per cache)
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
 * @brief Return true if the card passed its CMD0 / ACMD41 init sequence and
 *        sector I/O is known to work.
 */
int lfs_sd_bd_is_ready(void);

/**
 * @brief Return card capacity in MiB (for UI display). 0 if unknown.
 */
uint32_t lfs_sd_bd_capacity_mb(void);

#ifdef __cplusplus
}
#endif

#endif /* LFS_SD_BD_H */
