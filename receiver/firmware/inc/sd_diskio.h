/**
 ******************************************************************************
 * @file    sd_diskio.h
 * @brief   Header for sd_diskio.c module
 ******************************************************************************
 */

#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern const Diskio_drvTypeDef  SD_Driver;

DSTATUS SD_initialize (BYTE pdrv);
DSTATUS SD_status (BYTE pdrv);
DRESULT SD_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if FF_FS_READONLY == 0
DRESULT SD_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
DRESULT SD_ioctl (BYTE pdrv, BYTE cmd, void *buff);
void SD_SetFastSpeed (void);

extern uint8_t sd_diag_cmd_resp;   /* R1 from last CMD24 */
extern uint8_t sd_diag_data_token; /* data response token from last block write */
extern uint8_t sd_diag_wait_fail;  /* 1 = pre-write busy wait timed out */
extern uint8_t SD_Type;            /* card type: 4=SDv2, 12=SDHC/block-addr */

#ifdef __cplusplus
}
#endif

#endif /* __SD_DISKIO_H */
