/**
 * @file fake_sd_card.h
 * @brief The host tests' SD card, faked at the sd_diskio sector API.
 *
 * Every test that needs a card runs the real lfs_sd_bd.c (and the real
 * LittleFS above it) on this fake: test_lfs_sd_bd, test_sd_card and
 * test_main. One card model, so no test can fail differently from the real
 * block device - test_sd_card once ran over a RAM block device of its own
 * whose failed write was LFS_ERR_IO while the real one returned
 * LFS_ERR_CORRUPT, and that difference hid a watchdog-reset loop.
 *
 * Contract, from sd_diskio.c: RES_PARERR for a drive other than 0 or a
 * count of 0, RES_NOTRDY before SD_initialize succeeds, LBA addressing in
 * 512 B sectors, a sector past the end rejected (R1 address error). The
 * write diagnostics main.c shows on a failed self-test: CMD24's R1 (0x00)
 * and the data-response token (0x05 accepted, 0x0D write error). The SPI
 * ladder: SD_SetFastSpeed lands on the fastest divider that verifies,
 * SD_StepSpeedDown goes 2 -> 4 -> 8 -> 16 (and stays), SD_StepSpeedUpTry
 * tries one rung faster and keeps it only if it verifies. "Verifies" and
 * "transfers work" both mean: the divider is min_good_div or slower. This
 * board's wiring only works at /16 (lfs_sd_bd.c), the default.
 *
 * Include once per test binary. A test that keeps the card somewhere else
 * (test_main: shared memory, so it survives a reset) defines
 * FAKE_SD_STORAGE (and FAKE_SD_SECTORS) before including this.
 */
#ifndef FAKE_SD_CARD_H
#define FAKE_SD_CARD_H

#include <string.h>
#include "sd_diskio.h"
#include "lfs_sd_bd.h"

#ifndef FAKE_SD_SECTORS
#define FAKE_SD_SECTORS (4u * 2048u)          /* 4 MiB: 1024 LittleFS blocks */
#endif
#ifndef FAKE_SD_STORAGE
static uint8_t fake_sd_storage[FAKE_SD_SECTORS * LFS_SD_SECTOR_SIZE];
#define FAKE_SD_STORAGE fake_sd_storage
#endif

enum { FAKE_SD_INIT, FAKE_SD_READ, FAKE_SD_WRITE };

static struct {
    /* The card and its bus (knobs) */
    uint8_t  absent;           /* nothing answers CMD0: SD_initialize fails  */
    DRESULT  csd_result;       /* CMD9 (GET_SECTOR_COUNT) outcome            */
    DWORD    sector_count;     /* what the CSD reports                       */
    uint8_t  min_good_div;     /* the bus works at /min_good_div and slower  */
    uint8_t  read_only;        /* every write gets the write-error token     */
    uint32_t read_fails;       /* the next N reads fail                      */
    uint32_t write_fails;      /* the next N writes fail                     */
    uint32_t write_fail_from;  /* 0 = off; N: SD_write call N onwards fails  */
    void   (*on_op)(int op, UINT sectors);  /* e.g. a simulated clock        */
    /* What the firmware did to it */
    uint8_t  div;              /* SPI divider in effect, 0 = init clock      */
    uint8_t  csd_after_fast;   /* CMD9 issued after the speed switch         */
    uint32_t reads, writes, write_errors;
    uint32_t fast_speed_calls, stepdowns, stepup_tries;
    DWORD    last_sector;
    UINT     last_count;
} fake_sd;

uint8_t SD_Type = 0;           /* card type, 0 until SD_initialize succeeds */
uint8_t sd_diag_cmd_resp = 0, sd_diag_data_token = 0, sd_diag_wait_fail = 0;

/* A healthy card that has not been initialized; the storage is kept */
static void fake_sd_reset(void)
{
    memset(&fake_sd, 0, sizeof(fake_sd));
    fake_sd.csd_result = RES_OK;
    fake_sd.sector_count = FAKE_SD_SECTORS;
    fake_sd.min_good_div = 16;
    SD_Type = 0;
    sd_diag_cmd_resp = sd_diag_data_token = sd_diag_wait_fail = 0;
}

/* Every sector to `fill`: 0x00 or 0xFF, as cards differ when erased */
static void fake_sd_wipe(uint8_t fill)
{
    memset(FAKE_SD_STORAGE, fill, (size_t)FAKE_SD_SECTORS * LFS_SD_SECTOR_SIZE);
}

static uint8_t fake_sd_bus_ok(void)
{
    return fake_sd.div == 0 || fake_sd.div >= fake_sd.min_good_div;
}

DSTATUS SD_initialize(BYTE pdrv)
{
    if (pdrv) return STA_NOINIT;
    if (fake_sd.on_op) fake_sd.on_op(FAKE_SD_INIT, 0);
    fake_sd.div = 0;                       /* back on the init clock */
    SD_Type = fake_sd.absent ? 0 : 12;     /* SDHC: block addressed */
    return fake_sd.absent ? STA_NOINIT : 0;
}

DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    if (cmd != GET_SECTOR_COUNT) return RES_PARERR;
    if (fake_sd.div) fake_sd.csd_after_fast = 1;
    if (fake_sd.csd_result != RES_OK) return fake_sd.csd_result;
    *(DWORD *)buff = fake_sd.sector_count;
    return RES_OK;
}

void SD_SetFastSpeed(void)
{
    if (!SD_Type) return;                  /* nothing to speed up on a dead card */
    fake_sd.fast_speed_calls++;
    fake_sd.div = fake_sd.min_good_div;    /* the fastest rung that verifies */
}

uint8_t SD_FastSpeedEnabled(void) { return fake_sd.div; }

uint8_t SD_StepSpeedDown(void)
{
    fake_sd.stepdowns++;
    if (fake_sd.div >= 2 && fake_sd.div < 16) fake_sd.div = (uint8_t)(fake_sd.div * 2);
    return fake_sd.div;
}

uint8_t SD_StepSpeedUpTry(void)
{
    fake_sd.stepup_tries++;
    if (fake_sd.div > 2 && fake_sd.div <= 16 && fake_sd.div / 2 >= fake_sd.min_good_div)
        fake_sd.div = (uint8_t)(fake_sd.div / 2);
    return fake_sd.div;
}

DRESULT SD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    if (fake_sd.on_op) fake_sd.on_op(FAKE_SD_READ, count);
    fake_sd.reads++;
    fake_sd.last_sector = sector;
    fake_sd.last_count = count;
    if (fake_sd.read_fails) { fake_sd.read_fails--; return RES_ERROR; }
    if (!fake_sd_bus_ok() || sector + count > FAKE_SD_SECTORS) return RES_ERROR;
    memcpy(buff, &FAKE_SD_STORAGE[(size_t)sector * LFS_SD_SECTOR_SIZE],
           (size_t)count * LFS_SD_SECTOR_SIZE);
    return RES_OK;
}

DRESULT SD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count) return RES_PARERR;
    if (!SD_Type) return RES_NOTRDY;
    if (fake_sd.on_op) fake_sd.on_op(FAKE_SD_WRITE, count);
    fake_sd.writes++;
    fake_sd.last_sector = sector;
    fake_sd.last_count = count;
    sd_diag_cmd_resp = 0x00;               /* CMD24/25 accepted */
    uint8_t fail = fake_sd.read_only || !fake_sd_bus_ok() ||
                   sector + count > FAKE_SD_SECTORS ||
                   (fake_sd.write_fail_from && fake_sd.writes >= fake_sd.write_fail_from);
    if (fake_sd.write_fails) { fake_sd.write_fails--; fail = 1; }
    if (fail) {
        fake_sd.write_errors++;
        sd_diag_data_token = 0x0D;         /* data response: write error */
        return RES_ERROR;
    }
    sd_diag_data_token = 0x05;             /* data accepted */
    memcpy(&FAKE_SD_STORAGE[(size_t)sector * LFS_SD_SECTOR_SIZE], buff,
           (size_t)count * LFS_SD_SECTOR_SIZE);
    return RES_OK;
}

#endif /* FAKE_SD_CARD_H */
