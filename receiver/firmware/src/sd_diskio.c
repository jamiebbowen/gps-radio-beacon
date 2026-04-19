/**
 ******************************************************************************
 * @file    sd_diskio.c
 * @brief   SD card disk I/O driver for FATFS
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sd_diskio.h"
#include "ff_gen_drv.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* Diagnostic: last write attempt details, readable from sd_card.c */
uint8_t sd_diag_cmd_resp   = 0;  /* R1 from CMD24 */
uint8_t sd_diag_data_token = 0;  /* data response token after block write */
uint8_t sd_diag_wait_fail  = 0;  /* 1 = pre-write SD_WaitResponse timed out */

/* Private defines -----------------------------------------------------------*/
#define SD_TIMEOUT             1000
#define SD_DUMMY_BYTE          0xFF
#define SD_NO_RESPONSE_EXPECTED 0x80

/* SD card commands */
#define SD_CMD_GO_IDLE_STATE          0   /* CMD0 */
#define SD_CMD_SEND_OP_COND           1   /* CMD1 */
#define SD_CMD_SEND_IF_COND           8   /* CMD8 */
#define SD_CMD_SEND_CSD               9   /* CMD9 */
#define SD_CMD_SEND_CID               10  /* CMD10 */
#define SD_CMD_STOP_TRANSMISSION      12  /* CMD12 */
#define SD_CMD_SEND_STATUS            13  /* CMD13 */
#define SD_CMD_SET_BLOCKLEN           16  /* CMD16 */
#define SD_CMD_READ_SINGLE_BLOCK      17  /* CMD17 */
#define SD_CMD_READ_MULT_BLOCK        18  /* CMD18 */
#define SD_CMD_SET_BLOCK_COUNT        23  /* CMD23 */
#define SD_CMD_WRITE_SINGLE_BLOCK     24  /* CMD24 */
#define SD_CMD_WRITE_MULT_BLOCK       25  /* CMD25 */
#define SD_CMD_PROG_CSD               27  /* CMD27 */
#define SD_CMD_SET_WRITE_PROT         28  /* CMD28 */
#define SD_CMD_CLR_WRITE_PROT         29  /* CMD29 */
#define SD_CMD_SEND_WRITE_PROT        30  /* CMD30 */
#define SD_CMD_SD_ERASE_GRP_START     32  /* CMD32 */
#define SD_CMD_SD_ERASE_GRP_END       33  /* CMD33 */
#define SD_CMD_UNTAG_SECTOR           34  /* CMD34 */
#define SD_CMD_ERASE_GRP_START        35  /* CMD35 */
#define SD_CMD_ERASE_GRP_END          36  /* CMD36 */
#define SD_CMD_UNTAG_ERASE_GROUP      37  /* CMD37 */
#define SD_CMD_ERASE                  38  /* CMD38 */
#define SD_CMD_SD_APP_OP_COND         41  /* CMD41 */
#define SD_CMD_APP_CMD                55  /* CMD55 */
#define SD_CMD_READ_OCR               58  /* CMD58 */

/* SD card responses */
#define SD_RESPONSE_NO_ERROR          0x00
#define SD_RESPONSE_IN_IDLE           0x01
#define SD_RESPONSE_ERASE_RESET       0x02
#define SD_RESPONSE_ILLEGAL_COMMAND   0x04
#define SD_RESPONSE_COM_CRC_ERROR     0x08
#define SD_RESPONSE_ERASE_SEQUENCE_ERROR 0x10
#define SD_RESPONSE_ADDRESS_ERROR     0x20
#define SD_RESPONSE_PARAMETER_ERROR   0x40

/* Data tokens */
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_READ    0xFE
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_READ  0xFE
#define SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE   0xFE
#define SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE 0xFC
#define SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE  0xFD

/* Private variables ---------------------------------------------------------*/
uint8_t SD_Type = 0;  /* Make global for debugging access */
extern SPI_HandleTypeDef hspi1; /* Use external SPI handle from main.c */
/* 1 = use HAL during SD_initialize (reliable for command sequences);
 * 0 = use direct register for runtime ops (avoids HAL BSY underflow hang). */
static uint8_t sd_init_mode = 1;

/* Private function prototypes -----------------------------------------------*/
static void SD_SPI_Init(void);
static uint8_t SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response);
static uint8_t SD_WaitResponse(uint8_t Response);
static uint8_t SD_WaitWriteComplete(void);
static uint8_t SD_ReadData(uint8_t *buff, uint16_t btr);
static uint8_t SD_WriteData(const uint8_t *buff, uint8_t token);
static void SD_CS_LOW(void);
static void SD_CS_HIGH(void);
static uint8_t SD_SPI_TxRx(uint8_t data);

/* Disk functions ------------------------------------------------------------*/

/**
 * @brief  Initializes a Drive
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_initialize(BYTE pdrv)
{
  uint8_t n, ty, cmd, buf[4];
  uint16_t tmr;
  DSTATUS s = STA_NOINIT;

  if (pdrv) return STA_NOINIT; /* Supports only single drive */

  sd_init_mode = 1; /* Use HAL during initialization */

  /* Initialize SPI */
  SD_SPI_Init();

  /* Force reset SD_Type to ensure clean start */
  SD_Type = 0;

  /* Power up delay - SD cards need time to stabilize */
  HAL_Delay(100);

  /* Send 80+ dummy clocks with CS high per SD spec */
  SD_CS_HIGH();
  for (n = 0; n < 10; n++) {
    SD_SPI_TxRx(0xFF);
  }
  HAL_Delay(10);

  /* Enter Idle state with proper sequence */
  ty = 0;
  uint8_t cmd0_response = 0xFF;

  /* Simplified CMD0 sequence - try a few times then give up */
  for (n = 0; n < 5; n++) {
    /* Ensure CS is high before command */
    SD_CS_HIGH();
    HAL_Delay(1);
    
    /* Send some dummy bytes with CS high */
    SD_SPI_TxRx(0xFF);
    SD_SPI_TxRx(0xFF);
    
    /* Send CMD0 - note: SD_SendCmd has its own wait logic */
    cmd0_response = SD_SendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95, SD_RESPONSE_IN_IDLE);
    
    /* Accept both 0x01 (idle) and 0x00 (ready) as valid responses */
    if (cmd0_response == SD_RESPONSE_IN_IDLE || cmd0_response == SD_RESPONSE_NO_ERROR) {
      break; /* Success - card responded */
    }
    
    /* Wait before retry */
    HAL_Delay(20);
  }

  /* Store the actual response for debugging */
  SD_Type = cmd0_response; /* Temporarily store response in SD_Type for debugging */
  
  /* Accept both idle (0x01) and ready (0x00) states - treat them the same */
  /* Some cards may return 0x00 if already initialized, but we still need to */
  /* complete the initialization sequence for proper SPI mode operation */
  if (cmd0_response == SD_RESPONSE_IN_IDLE || cmd0_response == SD_RESPONSE_NO_ERROR) {
    /* SDC Ver2+ */
    uint8_t cmd8_response = SD_SendCmd(SD_CMD_SEND_IF_COND, 0x1AA, 0x87, SD_RESPONSE_IN_IDLE);
    if (cmd8_response == SD_RESPONSE_IN_IDLE || cmd8_response == SD_RESPONSE_NO_ERROR) {
      /* Get the other 4 bytes of R7 resp */
      for (n = 0; n < 4; n++) {
        buf[n] = SD_SPI_TxRx(SD_DUMMY_BYTE);
      }
      /* The card can work at vdd range of 2.7-3.6V */
      if (buf[2] == 0x01 && buf[3] == 0xAA) {
        /* Wait for leaving idle state (ACMD41 with HCS bit) */
        for (tmr = 1000; tmr; tmr--) {
          if (SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_RESPONSE_IN_IDLE) <= 1) {
            if (SD_SendCmd(SD_CMD_SD_APP_OP_COND, 1UL << 30, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
              break; /* Exited */
            }
          }
          HAL_Delay(1);
        }
        /* Check CCS bit in the OCR */
        if (tmr && SD_SendCmd(SD_CMD_READ_OCR, 0, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
          for (n = 0; n < 4; n++) {
            buf[n] = SD_SPI_TxRx(SD_DUMMY_BYTE);
          }
          ty = (buf[0] & 0x40) ? 12 : 4; /* SDv2 (HC or SC) */
        }
      }
    } else { /* SDC Ver1 or MMC */
      cmd = SD_CMD_SD_APP_OP_COND;
      if (SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_RESPONSE_IN_IDLE) <= 1) {
        if (SD_SendCmd(SD_CMD_SD_APP_OP_COND, 0, 0xFF, SD_RESPONSE_IN_IDLE) <= 1) {
          ty = 2; /* SDv1 */
        }
      }
      if (!ty) {
        cmd = SD_CMD_SEND_OP_COND;
        ty = 1; /* MMC */
      }
      /* Wait for leaving idle state */
      for (tmr = 1000; tmr; tmr--) {
        if (ty == 2) {
          if (SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_RESPONSE_IN_IDLE) <= 1) {
            if (SD_SendCmd(cmd, 0, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
              break; /* Exited */
            }
          }
        } else {
          if (SD_SendCmd(cmd, 0, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
            break; /* Exited */
          }
        }
        HAL_Delay(1);
      }
      if (!tmr || SD_SendCmd(SD_CMD_SET_BLOCKLEN, 512, 0xFF, SD_RESPONSE_NO_ERROR) != SD_RESPONSE_NO_ERROR) {
        ty = 0;
      }
    }
  }
  /* Disable CRC checking for data blocks (CMD59 arg=0) */
  if (ty) {
    SD_SendCmd(59, 0, 0xFF, SD_RESPONSE_NO_ERROR);
  }

  SD_Type = ty;
  s = ty ? 0 : STA_NOINIT;
  SD_CS_HIGH();

  sd_init_mode = 0; /* Switch to direct register for runtime operations */

  return s;
}

/**
 * @brief  Gets Disk Status
 * @param  pdrv: Physical drive number (0..)
 * @retval DSTATUS: Operation status
 */
DSTATUS SD_status(BYTE pdrv)
{
  if (pdrv) return STA_NOINIT; /* Supports only single drive */
  return SD_Type ? 0 : STA_NOINIT;
}

/**
 * @brief  Reads Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  buff: Data buffer to store read data
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to read (1..128)
 * @retval DRESULT: Operation result
 */
DRESULT SD_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
  if (pdrv || !count) return RES_PARERR;
  if (!SD_Type) return RES_NOTRDY;

  /* Convert to byte address if needed */
  if (!(SD_Type & 8)) sector *= 512;

  SD_CS_LOW();

  if (count == 1) { /* Single block read */
    if ((SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, sector, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) && SD_ReadData(buff, 512)) {
      count = 0;
    }
  } else { /* Multiple block read */
    if (SD_SendCmd(SD_CMD_READ_MULT_BLOCK, sector, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
      do {
        if (!SD_ReadData(buff, 512)) break;
        buff += 512;
      } while (--count);
      SD_SendCmd(SD_CMD_STOP_TRANSMISSION, 0, 0xFF, SD_NO_RESPONSE_EXPECTED);
    }
  }

  SD_CS_HIGH();

  return count ? RES_ERROR : RES_OK;
}

/**
 * @brief  Writes Sector(s)
 * @param  pdrv: Physical drive number (0..)
 * @param  buff: Data to be written
 * @param  sector: Sector address (LBA)
 * @param  count: Number of sectors to write (1..128)
 * @retval DRESULT: Operation result
 */
DRESULT SD_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
  if (pdrv || !count) return RES_PARERR;
  if (!SD_Type) return RES_NOTRDY;

  /* Convert to byte address if needed */
  if (!(SD_Type & 8)) sector *= 512;

  SD_CS_LOW();

  if (count == 1) { /* Single block write */
    sd_diag_cmd_resp = SD_SendCmd(SD_CMD_WRITE_SINGLE_BLOCK, sector, 0xFF, SD_RESPONSE_NO_ERROR);
    if ((sd_diag_cmd_resp == SD_RESPONSE_NO_ERROR) && SD_WriteData(buff, SD_TOKEN_START_DATA_SINGLE_BLOCK_WRITE)) {
      count = 0;
    }
  } else { /* Multiple block write */
    if (SD_Type & 6) SD_SendCmd(SD_CMD_SET_BLOCK_COUNT, count, 0xFF, SD_RESPONSE_NO_ERROR);
    if (SD_SendCmd(SD_CMD_WRITE_MULT_BLOCK, sector, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) {
      do {
        if (!SD_WriteData(buff, SD_TOKEN_START_DATA_MULTIPLE_BLOCK_WRITE)) break;
        buff += 512;
      } while (--count);
      if (!SD_WriteData(0, SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE)) count = 1;
    }
  }

  SD_CS_HIGH();

  return count ? RES_ERROR : RES_OK;
}

/**
 * @brief  I/O control operation
 * @param  pdrv: Physical drive number (0..)
 * @param  cmd: Control code
 * @param  buff: Buffer to send/receive control data
 * @retval DRESULT: Operation result
 */
DRESULT SD_ioctl(BYTE pdrv, BYTE cmd, void* buff)
{
  DRESULT res = RES_ERROR;
  uint8_t n, csd[16], *ptr = buff;
  DWORD csize;

  if (pdrv) return RES_PARERR;
  if (!SD_Type) return RES_NOTRDY;

  SD_CS_LOW();

  switch (cmd) {
    case CTRL_SYNC: /* Make sure that no pending write process */
      if (SD_WaitResponse(SD_DUMMY_BYTE) == SD_RESPONSE_NO_ERROR) {
        res = RES_OK;
      }
      break;

    case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
      if ((SD_SendCmd(SD_CMD_SEND_CSD, 0, 0xFF, SD_RESPONSE_NO_ERROR) == SD_RESPONSE_NO_ERROR) && SD_ReadData(csd, 16)) {
        if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 - C_SIZE is 22 bits: csd[7][5:0], csd[8], csd[9] */
          csize = ((DWORD)(csd[7] & 0x3F) << 16) | ((DWORD)csd[8] << 8) | csd[9];
          *(DWORD*)buff = (csize + 1) << 10;
        } else { /* SDC ver 1.XX or MMC */
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
          *(DWORD*)buff = (DWORD)csize << (n - 9);
        }
        res = RES_OK;
      }
      break;

    case GET_SECTOR_SIZE: /* Get R/W sector size (WORD) */
      *(WORD*)buff = 512;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE: /* Get erase block size in unit of sector (DWORD) */
      *(DWORD*)buff = 128;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
  }

  SD_CS_HIGH();

  return res;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize SD card SPI interface
 * @note   Hardware SPI is initialized in main.c, this is just a placeholder
 */
static void SD_SPI_Init(void)
{
  /* Hardware SPI1 is already initialized by MX_SPI1_Init() in main.c */
  /* CS pin is already configured in HAL_SPI_MspInit() in main.c */
  /* Nothing to do here - kept for compatibility */
}

/**
 * @brief  Switch SPI1 to high-speed mode after SD card init
 * @note   Call once after f_mount succeeds. Init must use <=400kHz;
 *         after init the card supports up to 25MHz. We use ~2.6MHz
 *         (APB2 21MHz / prescaler 8) as a conservative data-phase speed.
 *         This reduces per-sector write time from ~50ms to ~1.6ms,
 *         dramatically shrinking the corruption window on power loss.
 */
void SD_SetFastSpeed(void)
{
  HAL_SPI_DeInit(&hspi1);
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 21MHz/8 = ~2.6MHz */
  HAL_SPI_Init(&hspi1);
}

/**
 * @brief  Send a command packet to SD card
 */
static uint8_t SD_SendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response)
{
  uint8_t n, res;

  if (Cmd & 0x80) { /* ACMD<n> is the command sequence of CMD55-CMD<n> */
    Cmd &= 0x7F;
    res = SD_SendCmd(SD_CMD_APP_CMD, 0, 0xFF, SD_RESPONSE_IN_IDLE);
    if (res > 1) return res;
  }

  /* Select the card and wait for ready except to stop multiple block read */
  if (Cmd != SD_CMD_STOP_TRANSMISSION) {
    SD_CS_HIGH();
    SD_CS_LOW();
    /* Wait for card not-busy: idle card outputs 0xFF on MISO */
    if (SD_WaitResponse(SD_DUMMY_BYTE) != SD_RESPONSE_NO_ERROR) {
      return 0xFF;
    }
  }

  /* Send command packet */
  SD_SPI_TxRx(0x40 | Cmd);             /* Start + Command index */
  SD_SPI_TxRx((uint8_t)(Arg >> 24));   /* Argument[31..24] */
  SD_SPI_TxRx((uint8_t)(Arg >> 16));   /* Argument[23..16] */
  SD_SPI_TxRx((uint8_t)(Arg >> 8));    /* Argument[15..8] */
  SD_SPI_TxRx((uint8_t)Arg);           /* Argument[7..0] */
  SD_SPI_TxRx(Crc);                    /* CRC */

  /* Receive command response */
  if (Cmd == SD_CMD_STOP_TRANSMISSION) {
    SD_SPI_TxRx(SD_DUMMY_BYTE); /* Skip a stuff byte when stop reading */
  }

  /* Wait for a valid response in timeout of 10 attempts */
  n = 10;
  do {
    res = SD_SPI_TxRx(SD_DUMMY_BYTE);
  } while ((res & 0x80) && --n);

  return res; /* Return with the response value */
}

/**
 * @brief  Wait for write to complete (2s timeout - flash erase can be slow)
 */
static uint8_t SD_WaitWriteComplete(void)
{
  uint8_t b;

  /* Phase 1: wait up to 10ms for card to assert busy (MISO=0x00).
   * If we never see 0x00 the card finished instantly - return success. */
  uint32_t busy_start = HAL_GetTick() + 10;
  do {
    b = SD_SPI_TxRx(SD_DUMMY_BYTE);
    if (b == 0x00) goto wait_done; /* busy started */
  } while ((int32_t)(HAL_GetTick() - busy_start) < 0);
  return SD_RESPONSE_NO_ERROR; /* no busy pulse seen - write done */

wait_done:
  /* Phase 2: wait up to 2s for busy to end (any non-0x00 = done) */
  {
    uint32_t deadline = HAL_GetTick() + 2000;
    do {
      if (SD_SPI_TxRx(SD_DUMMY_BYTE) != 0x00) {
        return SD_RESPONSE_NO_ERROR;
      }
    } while ((int32_t)(HAL_GetTick() - deadline) < 0);
  }
  return SD_RESPONSE_PARAMETER_ERROR;
}

/**
 * @brief  Wait for card response
 */
static uint8_t SD_WaitResponse(uint8_t Response)
{
  /* Use time-based timeout so this works correctly at any SPI clock speed.
   * Count-based (0xFFF) was ~400ms at 82kHz but only ~12ms at 2.6MHz -
   * far too short for an SD flash write which can take up to 250ms. */
  uint32_t deadline = HAL_GetTick() + 500; /* 500ms absolute deadline */

  do {
    uint8_t b = SD_SPI_TxRx(SD_DUMMY_BYTE);
    if (b == Response) return SD_RESPONSE_NO_ERROR;
    /* When waiting for card-ready (0xFF), treat any non-0x00 byte as ready.
     * Per SD spec, busy = MISO held 0x00; this card idles at 0xCA not 0xFF. */
    if (Response == SD_DUMMY_BYTE && b != 0x00) return SD_RESPONSE_NO_ERROR;
  } while ((int32_t)(HAL_GetTick() - deadline) < 0);

  return SD_RESPONSE_PARAMETER_ERROR;
}

/**
 * @brief  Read data from SD card
 */
static uint8_t SD_ReadData(uint8_t *buff, uint16_t btr)
{
  uint8_t token;
  uint16_t tmr;

  /* Wait for data packet in timeout of 100ms */
  tmr = 1000;
  do {
    token = SD_SPI_TxRx(SD_DUMMY_BYTE);
  } while ((token == SD_DUMMY_BYTE) && --tmr);

  if (token != SD_TOKEN_START_DATA_SINGLE_BLOCK_READ) {
    return 0; /* If not valid data token, return with error */
  }

  /* Receive the data block into buffer */
  do {
    *buff++ = SD_SPI_TxRx(SD_DUMMY_BYTE);
  } while (--btr);

  /* Discard CRC */
  SD_SPI_TxRx(SD_DUMMY_BYTE);
  SD_SPI_TxRx(SD_DUMMY_BYTE);

  return 1; /* Return with success */
}

/**
 * @brief  CRC-16 CCITT (polynomial 0x1021) used for SD card data block CRC
 */
static uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

/**
 * @brief  Write data to SD card
 */
static uint8_t SD_WriteData(const uint8_t *buff, uint8_t token)
{
  uint8_t resp;
  uint16_t bc = 512;

  /* Wait for card not-busy before sending data token (MISO=0xFF = ready) */
  sd_diag_wait_fail = 0;
  if (SD_WaitResponse(SD_DUMMY_BYTE) != SD_RESPONSE_NO_ERROR) {
    sd_diag_wait_fail = 1;
    return 0;
  }

  /* Send token */
  SD_SPI_TxRx(token);
  if (token != SD_TOKEN_STOP_DATA_MULTIPLE_BLOCK_WRITE) { /* If not StopTran token */
    /* Send the data block to the MMC */
    const uint8_t *data_start = buff; /* Save for CRC calculation */
    do {
      SD_SPI_TxRx(*buff++);
    } while (--bc);

    /* Send real CRC-16 CCITT - works whether CRC mode is on or off */
    uint16_t crc = CRC16_CCITT(data_start, 512);
    SD_SPI_TxRx((uint8_t)(crc >> 8));
    SD_SPI_TxRx((uint8_t)(crc & 0xFF));

    /* Read data response token - wait for first non-0xFF byte */
    resp = 0xFF;
    for (uint8_t i = 0; i < 16 && resp == 0xFF; i++) {
      resp = SD_SPI_TxRx(SD_DUMMY_BYTE);
    }
    sd_diag_data_token = resp; /* capture for diagnostics */
    /* Accept 0x05 (spec) or 0xCA (non-standard accepted token this card emits) */
    if ((resp & 0x1F) != 0x05 && resp != 0xCA) {
      return 0;
    }

    /* Wait for write to complete: card holds MISO low while busy */
    if (SD_WaitWriteComplete() != SD_RESPONSE_NO_ERROR) {
      return 0;
    }
  }

  return 1;
}

/**
 * @brief  Set CS pin low
 */
static void SD_CS_LOW(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET);
  /* Small delay for CS setup time */
  for(volatile int i = 0; i < 100; i++);
}

/**
 * @brief  Set CS pin high
 */
static void SD_CS_HIGH(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);
  /* Small delay for CS hold time */
  for(volatile int i = 0; i < 100; i++);
}

/**
 * @brief  Hardware SPI transmit/receive byte
 * @note   During SD_initialize (sd_init_mode=1) uses HAL - reliable for the
 *         command/response sequences needed for card detection.
 *         At runtime (sd_init_mode=0) uses direct register access - avoids
 *         the HAL BSY-flag underflow bug that causes f_sync to hang forever
 *         when called thousands of times in a tight busy-wait loop.
 */
static uint8_t SD_SPI_TxRx(uint8_t data)
{
  if (sd_init_mode) {
    uint8_t rxData = 0xFF;
    if (HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, 100) != HAL_OK) {
      HAL_SPI_Abort(&hspi1);
      HAL_SPI_Init(&hspi1);
    }
    return rxData;
  } else {
    SPI_TypeDef *spi = hspi1.Instance;
    uint32_t deadline = HAL_GetTick() + 100;
    while (!(spi->SR & SPI_SR_TXE)) {
      if ((int32_t)(HAL_GetTick() - deadline) >= 0) return 0xFF;
    }
    *(__IO uint8_t *)&spi->DR = data;
    while (!(spi->SR & SPI_SR_RXNE)) {
      if ((int32_t)(HAL_GetTick() - deadline) >= 0) return 0xFF;
    }
    return (uint8_t)spi->DR;
  }
}


/* Disk driver structure */
const Diskio_drvTypeDef SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if FF_FS_READONLY == 0
  SD_write,
#endif
#if FF_USE_IOCTL == 1
  SD_ioctl,
#endif
};
