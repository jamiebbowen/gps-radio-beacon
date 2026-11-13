/**
 * @file diskio.h
 * @brief Minimal FatFs diskio.h stub for host tests.
 *
 * The real header lives in the untracked STM32CubeF4 checkout - host CI
 * only needs the type/constant surface that sd_diskio.c and ff_gen_drv.h
 * consume. Mirrors FatFs R0.15 names/values.
 */

#ifndef __STUB_DISKIO_H
#define __STUB_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* These types must match the real FatFs integer.h */
typedef uint8_t  BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef uint16_t UINT;
typedef uint32_t LBA_t;

typedef BYTE DSTATUS;

typedef enum {
    RES_OK = 0,     /* 0: Successful */
    RES_ERROR,      /* 1: R/W Error */
    RES_WRPRT,      /* 2: Write Protected */
    RES_NOTRDY,     /* 3: Not Ready */
    RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;

/* Disk Status Bits (DSTATUS) */
#define STA_NOINIT   0x01   /* Drive not initialized */
#define STA_NODISK   0x02   /* No medium in the drive */
#define STA_PROTECT  0x04   /* Write protected */

/* Command code for disk_ioctl() */
#define CTRL_SYNC        0   /* Complete pending write process */
#define GET_SECTOR_COUNT 1   /* Get media size (DWORD*) */
#define GET_SECTOR_SIZE  2   /* Get sector size (WORD*) */
#define GET_BLOCK_SIZE   3   /* Get erase block size (DWORD*) */
#define CTRL_TRIM        4   /* Erase blocks (LBA_t[2]) */
#define MMC_GET_TYPE     10
#define MMC_GET_CSD      11
#define MMC_GET_CID      12
#define MMC_GET_OCR      13
#define MMC_GET_SDSTAT   14
#define ISDIO_READ       55
#define ISDIO_WRITE      56
#define ISDIO_MRITE      57
#define ATA_GET_REV      20
#define ATA_GET_MODEL    21
#define ATA_GET_SN       22

/* ff_gen_drv.h gates its struct members on these (Cube's diskio.h sets them) */
#ifndef _USE_WRITE
#define _USE_WRITE   1
#endif
#ifndef _USE_IOCTL
#define _USE_IOCTL   1
#endif

/* Generic command (not part of the test surface; keep for parity) */
DSTATUS disk_initialize(BYTE pdrv);
DSTATUS disk_status(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#ifdef __cplusplus
}
#endif

#endif /* __STUB_DISKIO_H */
