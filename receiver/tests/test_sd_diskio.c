/**
 * @file test_sd_diskio.c
 * @brief Host-side unit tests for the SD card SPI driver (sd_diskio.c),
 *        talking to a scripted virtual SD card on a fake SPI1.
 *
 * The card model below implements just enough of the SD SPI protocol to
 * exercise the real driver unmodified:
 *   - CMD0/CMD8/ACMD41/CMD58/CMD59 init handshake (SDv2-HC and v1 paths)
 *   - CMD17 single-block reads with 0xFE token + data + CRC
 *   - CMD24 single-block writes with data-response + busy window
 *   - CMD9 CSD reads behind GET_SECTOR_COUNT
 *   - programmable stalls, reject tokens, and infinite-busy for the
 *     timeout/error paths
 *
 * Byte-pump note: the runtime path uses direct-register access while the
 * init path uses HAL_SPI_TransmitReceive. The command layer is identical
 * either way, so protocol tests re-arm sd_init_mode=1 after init (the
 * register pump itself gets its own flag/timeout semantics tests here).
 * The speed-ladder tests additionally verify the B4 fix: every runtime
 * transaction re-asserts the negotiated prescaler, since the display and
 * eInk share SPI1 and freely re-init its clock.
 *
 * Build & run:  make -C receiver/tests
 */

#include "test_harness.h"
#include <string.h>
#include "sd_diskio.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/* Virtual SPI1 registers + card model                                 */
/* ------------------------------------------------------------------ */

SPI_HandleTypeDef hspi1;
static SPI_TypeDef spi1_regs;

static struct {
    uint8_t  present;          /* 0 = socket empty: MISO always 0xFF    */
    uint8_t  cs;               /* 1 = selected (CS low)                 */
    uint8_t  cmd[6];
    uint8_t  cmd_len;
    uint8_t  acmd41_warmup;    /* ACMD41s that still answer "idle"      */
    uint8_t  cmd8_succeeds;    /* 0 = v1/MMC card (CMD8 illegal)        */
    uint8_t  data_resp;        /* data-response token on writes         */
    uint8_t  busy_forever;     /* write busy never lifts                */
    uint8_t  stuck_busy;       /* permanently busy, survives CS toggle  */
    uint8_t  prewrite_busy;    /* busy after CMD24/25 resp, before token */
    uint8_t  never_idle;       /* ACMD41/CMD1 always answer "idle"      */
    uint32_t busy_writes;      /* busy bytes armed on write completion  */
    uint32_t busy_left;        /* busy bytes remaining                  */
    uint32_t arg_lba;          /* block address of in-flight write      */
    uint8_t  initialized;      /* card left idle state (ACMD41 done)    */
    uint8_t  mmc;              /* ACMD41 illegal, CMD1 works            */
    uint8_t  v1_csd;           /* CMD9 returns the v1 CSD layout        */
    uint8_t  idle_byte;        /* in-CS idle byte (quirky cards: 0xCA)  */
    uint8_t  read_stall;       /* accept CMD17, never emit data token   */
    uint8_t  vary_reads;       /* each read differs -> verify fails     */
    uint8_t  write_reject;     /* bad data-response token               */
    uint8_t  stream_left;      /* blocks left in a CMD18 stream         */
    uint32_t stream_lba;       /* next sector the stream serves         */
    uint8_t  multi_write;      /* CMD25 stream open until 0xFD token    */
    uint8_t  blocks[8][512];   /* backing store, sectors 0-7            */
    uint32_t reads, writes;
    uint16_t rx_crc;           /* CRC16 the driver sent on last write   */
    uint8_t  rx_crc_ok;
    uint8_t  out[600];         /* queued response/data stream           */
    uint16_t out_len, out_pos;
    uint8_t  expect_data;      /* waiting for the write-data token      */
    uint8_t  rxbuf[514];
    uint16_t rx_need;
    uint32_t br_seen;          /* bitmask of CR1 BR values seen on wire */
} card;

/* Independent CRC-16 CCITT (poly 0x1021) - the card checks the driver's
 * CRC against its OWN math, so a wrong implementation can't self-agree. */
static uint16_t card_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void card_push(const uint8_t *b, uint16_t n)
{
    memcpy(card.out + card.out_pos + card.out_len, b, n);
    card.out_len += n;
}
static void card_push1(uint8_t b)
{
    card.out[card.out_pos + card.out_len++] = b;
}

/* CSD v2 layout for a 2 GB card (C_SIZE = 2047 -> (2047+1)<<10 sectors) */
static const uint8_t CSD_V2[16] = {
    0x40, 0x0E, 0x00, 0x32, 0x5B, 0x59, 0x00, 0x00,
    0x07, 0xFF, 0x80, 0x0A, 0x40, 0x00, 0x00, 0x00
};

/* CSD v1 layout: READ_BL_LEN=9 (csd[5]&15), C_SIZE=2046 spread over
 * csd[8]>>6|csd[7]<<2|(csd[6]&3)<<10, C_SIZE_MULT bit0 via csd[9]&3 ->
 * n = 9+0+2+2 = 13, csize = 2047, sectors = 2047 << 4 = 32752. */
static const uint8_t CSD_V1[16] = {
    0x00, 0x26, 0x00, 0x32, 0x1F, 0x29, 0x01, 0xFF,
    0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void card_handle_command(void)
{
    uint8_t cmd  = card.cmd[0] & 0x3F;
    uint32_t arg = ((uint32_t)card.cmd[1] << 24) | ((uint32_t)card.cmd[2] << 16)
                 | ((uint32_t)card.cmd[3] << 8) | card.cmd[4];
    /* byte-addressed cards (v1/MMC) get sector*512 as the argument */
    uint32_t lba = (SD_Type & 8) ? arg : (arg >> 9);
    static uint8_t app_cmd = 0;      /* CMD55 prefix tracking */

    switch (cmd) {
    case 0:  card_push1(0x01); card.initialized = 0; break;   /* GO_IDLE    */
    case 1:                                                    /* OP_COND   */
        if (card.mmc && !card.never_idle) {
            card_push1(0x00); card.initialized = 1;
        }
        else if (card.mmc) card_push1(0x01);
        else               card_push1(0x05);
        break;
    case 8:                                                    /* IF_COND   */
        if (card.cmd8_succeeds) {
            const uint8_t r7[5] = { 0x01, 0x00, 0x00, 0x01, 0xAA };
            card_push(r7, 5);
        } else {
            card_push1(0x05);                                 /* illegal   */
        }
        break;
    case 9:                                                    /* SEND_CSD  */
        card_push1(0x00); card_push1(0xFF); card_push1(0xFE);
        card_push(card.v1_csd ? CSD_V1 : CSD_V2, 16);
        card_push1(0x55); card_push1(0x55);
        break;
    case 12:                                                   /* STOP_TRAN */
        /* abort any open read stream; R1b: idle byte after the driver's
         * stuff byte, response, then a short busy tail */
        card.out_len = card.out_pos = 0;
        card.stream_left = 0;
        card_push1(0xFF); card_push1(0x00);
        card.busy_left = 2;
        break;
    case 18:                                                   /* READ_MULT */
        card_push1(0x00);
        card.stream_left = 8;           /* streams until CMD12 stops it */
        card.stream_lba = lba;
        break;
    case 16: case 59: case 55: case 23:                        /* misc ok   */
        card_push1(card.initialized ? 0x00 : 0x01);
        if (cmd == 55) app_cmd = 1;
        break;
    case 41:                                                   /* APP_OP    */
        if (card.mmc || !app_cmd) {
            card_push1(0x05);
        } else if (card.never_idle) {
            card_push1(0x01);
        } else if (card.acmd41_warmup) {
            card.acmd41_warmup--; card_push1(0x01);
        } else {
            card_push1(0x00); card.initialized = 1;
        }
        break;
    case 58: {                                                 /* READ_OCR  */
        const uint8_t ocr[5] = { 0x00, 0xC0, 0x00, 0x00, 0x00 };
        card_push(ocr, 5);                                     /* CCS set   */
        break;
    }
    case 17: {                                                 /* READ_1    */
        card_push1(0x00);
        if (card.read_stall) break;          /* R1 ok, then nothing ever   */
        card_push1(0xFF); card_push1(0xFE);
        uint8_t blk[512];
        memcpy(blk, card.blocks[lba & 7], 512);
        if (card.vary_reads) blk[0] ^= (uint8_t)++card.reads;   /* unstable */
        else card.reads++;
        card_push(blk, 512);
        uint16_t crc = card_crc16(blk, 512);
        card_push1((uint8_t)(crc >> 8)); card_push1((uint8_t)crc);
        break;
    }
    case 24:                                                   /* WRITE_1   */
    case 25:                                                   /* WRITE_MULT*/
        card_push1(0x00);
        card.expect_data = 1;
        card.rx_need = 0;
        card.arg_lba = lba;
        card.multi_write = (cmd == 25);
        if (card.prewrite_busy) card.busy_left = 0xFFFFFFFFu;
        break;
    default:
        card_push1(0x05);                                      /* illegal   */
    }
    if (cmd != 55) app_cmd = 0;
}

static uint8_t card_xfer(uint8_t mosi)
{
    card.br_seen |= 1u << ((spi1_regs.CR1 & SPI_CR1_BR_Msk) >> SPI_CR1_BR_Pos);
    if (!card.cs || !card.present) return 0xFF;
    if (card.stuck_busy) return 0x00;          /* wedged card           */

    /* in-flight write payload bytes are data, never command starts */
    if (card.expect_data && card.rx_need) {
        if (card.rx_need > 2) card.rxbuf[514 - card.rx_need] = mosi;
        else                  card.rx_crc = (uint16_t)((card.rx_crc << 8) | mosi);
        if (--card.rx_need == 0) {
            if (!card.multi_write) card.expect_data = 0;
            card.writes++;
            card.rx_crc_ok =
                (card.rx_crc == card_crc16(card.rxbuf, 512));
            memcpy(card.blocks[card.arg_lba & 7], card.rxbuf, 512);
            card.arg_lba++;
            /* data-response token, THEN the programmed busy window */
            card_push1(card.write_reject ? 0x00 : card.data_resp);
            card.busy_left = card.busy_forever ? 0xFFFFFFFFu
                                             : card.busy_writes;
        }
        return 0xFF;
    }

    /* command frame capture: a 6-byte frame runs to completion once
     * started (arg bytes may themselves match the 0b01xxxxxx start
     * pattern - only an IDLE stream treats them as new commands) */
    if (card.cmd_len) {
        card.cmd[card.cmd_len++] = mosi;
        if (card.cmd_len == 6) { card.cmd_len = 0; card_handle_command(); }
        return card.idle_byte;
    }
    if ((mosi & 0xC0) == 0x40) { card.cmd_len = 1; card.cmd[0] = mosi; return card.idle_byte; }

    /* multi-block read stream: synthesize the next block on demand -
     * the driver stops the flow with CMD12, not a count. Frame capture
     * above always wins so CMD12 reaches the card mid-stream. */
    if (!card.out_len && card.stream_left && mosi == 0xFF) {
        card_push1(0xFF); card_push1(0xFE);
        uint8_t blk[512];
        memcpy(blk, card.blocks[card.stream_lba & 7], 512);
        if (card.vary_reads) blk[0] ^= card.stream_left;
        card_push(blk, 512);
        uint16_t crc = card_crc16(blk, 512);
        card_push1((uint8_t)(crc >> 8)); card_push1((uint8_t)crc);
        card.stream_lba++;
        card.stream_left--;
        card.reads++;
    }

    /* queued response bytes always come first (R1/R7/CSD/data payloads,
     * and the write data-response token pushed when rx completes) */
    if (card.out_len) {
        card.out_len--;
        uint8_t b = card.out[card.out_pos++];
        if (!card.out_len) card.out_pos = 0;   /* drained: reuse buffer */
        return b;
    }

    if (card.expect_data) {
        /* between blocks the card still reports programming-busy */
        if (card.busy_left) { card.busy_left--; return 0x00; }
        if (mosi == 0xFE || mosi == 0xFC) {    /* single/multi token   */
            card.rx_need = 514;
            card.rx_crc = 0;
            return 0xFF;
        }
        if (mosi == 0xFD) {                    /* StopTran: end stream */
            card.expect_data = 0;
            card.multi_write = 0;
            card.busy_left = card.busy_writes; /* flush window         */
            return 0xFF;
        }
        return 0xFF;                           /* swallow until token */
    }

    if (card.busy_left) {
        card.busy_left--;
        return 0x00;                           /* busy: MISO held low  */
    }

    return card.idle_byte;
}

/* ------------------------------------------------------------------ */
/* HAL fakes                                                           */
/* ------------------------------------------------------------------ */

static uint32_t spi_init_calls = 0, spi_abort_calls = 0;
static HAL_StatusTypeDef txrx_result = HAL_OK;

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx,
                                          uint8_t *rx, uint16_t size,
                                          uint32_t timeout)
{
    (void)h; (void)timeout;
    if (txrx_result != HAL_OK) return txrx_result;
    for (uint16_t i = 0; i < size; i++) rx[i] = card_xfer(tx[i]);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{ (void)h; spi_init_calls++; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h)
{ (void)h; spi_abort_calls++; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *d,
                                   uint16_t s, uint32_t t)
{ (void)h; (void)d; (void)s; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *d,
                                  uint16_t s, uint32_t t)
{ (void)h; (void)d; (void)s; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *h, uint8_t *d,
                                      uint16_t s)
{ (void)h; (void)d; (void)s; return HAL_OK; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    if (port == SD_CS_GPIO_PORT && pin == SD_CS_PIN) {
        card.cs = (st == GPIO_PIN_RESET);
        if (st == GPIO_PIN_SET) {          /* CS release aborts the txn */
            card.cmd_len = 0;
            card.out_len = card.out_pos = 0;
            card.expect_data = 0;
            card.rx_need = 0;
            card.busy_left = 0;
            card.stream_left = 0;
            card.multi_write = 0;
        }
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return GPIO_PIN_RESET; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{ (void)port; (void)init; }
void HAL_Delay(uint32_t ms) { Test_SetTick(HAL_GetTick() + ms); }

/* ------------------------------------------------------------------ */

#include "../firmware/src/sd_diskio.c"

static void reset_all(void)
{
    memset(&card, 0, sizeof(card));
    card.present = 1;
    card.cmd8_succeeds = 1;
    card.data_resp = 0xE5;               /* standard "data accepted" */
    card.idle_byte = 0xFF;               /* MISO idles high            */
    card.busy_writes = 4;                /* short busy pulse per write */
    for (int b = 0; b < 8; b++)
        for (int i = 0; i < 512; i++)
            card.blocks[b][i] = (uint8_t)(b * 64 + i / 8);
    txrx_result = HAL_OK;
    spi_init_calls = spi_abort_calls = 0;
    hspi1.Instance = &spi1_regs;
    memset(&spi1_regs, 0, sizeof(spi1_regs));
    spi1_regs.SR = SPI_SR_TXE | SPI_SR_RXNE;
    SD_Type = 0;
    sd_init_mode = 1;
    sd_spi_div = 0;
    sd_diag_cmd_resp = sd_diag_data_token = sd_diag_wait_fail = 0;
    Test_SetTick(0);
    Test_SetTickAutoAdvance(0);
}

/* init a present v2 card and return to HAL-mode pumping for the protocol */
static DSTATUS init_card(void)
{
    DSTATUS s = SD_initialize(0);
    sd_init_mode = 1;   /* keep the card model serving runtime ops too */
    return s;
}

static uint32_t cr1_br(void)
{
    return (spi1_regs.CR1 & SPI_CR1_BR_Msk) >> SPI_CR1_BR_Pos;
}

/* ------------------------------------------------------------------ */
/* Initialization                                                      */
/* ------------------------------------------------------------------ */

TEST(init_v2hc_card_succeeds)
{
    reset_all();
    CHECK(init_card() == 0);             /* DSTATUS 0 = initialized     */
    CHECK(SD_Type == 12);                /* SDv2, block-addressed       */
    CHECK(SD_status(0) == 0);
}

TEST(init_no_card_fails_cleanly)
{
    reset_all();
    card.present = 0;
    CHECK(SD_initialize(0) == STA_NOINIT);
    CHECK(SD_Type == 0);
    /* a dead socket must not poison follow-up calls */
    uint8_t buf[512];
    CHECK(SD_read(0, buf, 0, 1) == RES_NOTRDY);
}

TEST(init_v1_card_takes_the_legacy_path)
{
    reset_all();
    card.cmd8_succeeds = 0;              /* v1/MMC: CMD8 illegal        */
    CHECK(init_card() == 0);
    CHECK(SD_Type == 2);                 /* SDv1, byte-addressed        */
}

TEST(init_mmc_card_uses_cmd1)
{
    reset_all();
    card.cmd8_succeeds = 0;              /* not an SDv2                 */
    card.mmc = 1;                        /* ACMD41 illegal, CMD1 works  */
    CHECK(init_card() == 0);
    CHECK(SD_Type == 1);                 /* MMC, byte-addressed         */

    /* byte-addressed card: sector 2 arrives on the wire as arg=1024 */
    uint8_t buf[512];
    CHECK(SD_read(0, buf, 2, 1) == RES_OK);
    CHECK(memcmp(buf, card.blocks[2], 512) == 0);
}

TEST(init_acmd41_warmup_retries_then_succeeds)
{
    /* Real cards take a few ACMD41 polls to leave idle - the init loop
     * must tolerate 0x01 responses and keep polling within its budget. */
    reset_all();
    card.acmd41_warmup = 4;
    CHECK(init_card() == 0);
    CHECK(SD_Type == 12);
}

TEST(init_hal_error_retries_and_recovers)
{
    reset_all();
    txrx_result = HAL_ERROR;             /* wedge the SPI peripheral    */
    CHECK(SD_initialize(0) == STA_NOINIT);
    CHECK(spi_abort_calls > 0);          /* abort+reinit per failed byte */
    CHECK(spi_init_calls > 0);
}

TEST(init_v1_card_never_leaves_idle_times_out)
{
    /* v1 card (CMD8 illegal) that answers ACMD41 "still idle" forever:
     * the 1000-poll wait loop must burn its budget and give up - not
     * declare a card that never finished initializing. */
    reset_all();
    card.cmd8_succeeds = 0;
    card.never_idle = 1;
    Test_SetTickAutoAdvance(1);
    CHECK(SD_initialize(0) == STA_NOINIT);
    Test_SetTickAutoAdvance(0);
    CHECK(SD_Type == 0);                 /* loop expired: ty cleared    */
}

/* ------------------------------------------------------------------ */
/* Block reads / writes                                                */
/* ------------------------------------------------------------------ */

TEST(read_single_block_roundtrip)
{
    reset_all();
    init_card();
    uint8_t buf[512];
    memset(buf, 0, sizeof(buf));
    CHECK(SD_read(0, buf, 0, 1) == RES_OK);
    CHECK(memcmp(buf, card.blocks[0], 512) == 0);
    CHECK(card.reads == 1);
}

TEST(read_multi_block_streams_until_cmd12)
{
    reset_all();
    init_card();
    uint8_t buf[1024];
    memset(buf, 0, sizeof(buf));
    CHECK(SD_read(0, buf, 1, 2) == RES_OK);
    CHECK(memcmp(buf, card.blocks[1], 512) == 0);
    CHECK(memcmp(buf + 512, card.blocks[2], 512) == 0);
    /* two blocks streamed, then CMD12 cut the flow: no third block's
     * first bytes may have leaked into the driver's buffer */
    CHECK(card.reads == 2);
    CHECK(card.stream_left == 0);
}

TEST(read_timeout_when_card_stalls)
{
    reset_all();
    init_card();
    card.read_stall = 1;
    uint8_t buf[512];
    Test_SetTickAutoAdvance(1);          /* 100 ms deadline can elapse  */
    CHECK(SD_read(0, buf, 0, 1) == RES_ERROR);
}

TEST(write_single_block_roundtrip_and_crc)
{
    reset_all();
    init_card();
    uint8_t buf[512];
    for (int i = 0; i < 512; i++) buf[i] = (uint8_t)(i * 7 + 3);
    CHECK(SD_write(0, buf, 3, 1) == RES_OK);
    CHECK(card.writes == 1);
    /* the CRC16 the driver sent matches the card's own computation -
     * a wrong implementation can't self-agree here */
    CHECK(card.rx_crc_ok == 1);
    CHECK(sd_diag_data_token == 0xE5);
    /* and the data actually landed in the backing store */
    uint8_t rd[512];
    CHECK(SD_read(0, rd, 3, 1) == RES_OK);
    CHECK(memcmp(rd, buf, 512) == 0);
}

TEST(write_multi_block_roundtrip)
{
    reset_all();
    init_card();
    uint8_t buf[1024];
    for (int i = 0; i < 1024; i++) buf[i] = (uint8_t)(i >> 2);
    CHECK(SD_write(0, buf, 2, 2) == RES_OK);
    CHECK(card.writes == 2);
    CHECK(card.rx_crc_ok == 1);
    CHECK(memcmp(card.blocks[2], buf, 512) == 0);
    CHECK(memcmp(card.blocks[3], buf + 512, 512) == 0);
}

TEST(write_no_busy_pulse_succeeds)
{
    /* Some cards complete so fast the busy pulse never shows - the wait
     * must take its "no busy seen" early-success path, not time out. */
    reset_all();
    card.busy_writes = 0;
    init_card();
    uint8_t buf[512];
    memset(buf, 0x5A, sizeof(buf));
    Test_SetTickAutoAdvance(1);          /* phase-1 10 ms busy-watch    */
    CHECK(SD_write(0, buf, 4, 1) == RES_OK);
    CHECK(card.writes == 1);
    CHECK(card.rx_crc_ok == 1);
}

TEST(card_idle_at_0xca_still_counted_ready)
{
    /* Field quirk: this card idles MISO at 0xCA, not 0xFF. The ready
     * check treats "anything but held-low busy" as ready - cover it. */
    reset_all();
    init_card();
    card.idle_byte = 0xCA;
    uint8_t buf[512];
    memset(buf, 0x11, sizeof(buf));
    CHECK(SD_write(0, buf, 5, 1) == RES_OK);
    CHECK(card.writes == 1);
}

TEST(command_prewait_times_out_when_card_stays_busy)
{
    /* Card wedged busy before the command: SendCmd's pre-wait must hit
     * its deadline and the read fails - not hang forever. */
    reset_all();
    init_card();
    card.stuck_busy = 1;
    uint8_t buf[512];
    Test_SetTickAutoAdvance(1);
    CHECK(SD_read(0, buf, 0, 1) == RES_ERROR);
    CHECK(card.reads == 0);              /* pre-wait aborted the command */
}

TEST(write_prewrite_busy_wait_times_out)
{
    /* Card takes the write command, then holds MISO low instead of going
     * ready: the pre-data wait must hit its 500 ms deadline and flag the
     * diagnostic, not send the data token into a deaf card. */
    reset_all();
    init_card();
    card.prewrite_busy = 1;
    uint8_t buf[512];
    memset(buf, 0xAA, sizeof(buf));
    Test_SetTickAutoAdvance(1);
    CHECK(SD_write(0, buf, 0, 1) == RES_ERROR);
    Test_SetTickAutoAdvance(0);
    CHECK(sd_diag_wait_fail == 1);
    CHECK(card.writes == 0);             /* payload never left the host */
}

TEST(write_rejected_token_fails_with_diagnostics)
{
    reset_all();
    init_card();
    card.write_reject = 1;
    uint8_t buf[512];
    memset(buf, 0xAA, sizeof(buf));
    Test_SetTickAutoAdvance(1);
    CHECK(SD_write(0, buf, 0, 1) == RES_ERROR);
    CHECK(sd_diag_cmd_resp == 0x00);     /* CMD24 itself was accepted   */
    CHECK(sd_diag_data_token == 0x00);   /* reject token captured       */
}

TEST(write_busy_timeout_fails)
{
    reset_all();
    init_card();
    card.busy_forever = 1;
    uint8_t buf[512];
    memset(buf, 1, sizeof(buf));
    Test_SetTickAutoAdvance(1);          /* let the 2 s deadline elapse */
    CHECK(SD_write(0, buf, 0, 1) == RES_ERROR);
}

TEST(ioctl_reports_sector_count)
{
    reset_all();
    init_card();
    DWORD sectors = 0;
    CHECK(SD_ioctl(0, GET_SECTOR_COUNT, &sectors) == RES_OK);
    /* CSD v2, C_SIZE=2047: (2047+1) << 10 = 2097152 */
    CHECK(sectors == 2097152u);
    WORD ss = 0;
    CHECK(SD_ioctl(0, GET_SECTOR_SIZE, &ss) == RES_OK);
    CHECK(ss == 512);
}

TEST(ioctl_sector_count_v1_csd_formula)
{
    /* v1/MMC cards carry the smaller CSD: different sector-count math */
    reset_all();
    card.cmd8_succeeds = 0;
    card.v1_csd = 1;
    CHECK(init_card() == 0);
    CHECK(SD_Type == 2);
    DWORD sectors = 0;
    CHECK(SD_ioctl(0, GET_SECTOR_COUNT, &sectors) == RES_OK);
    /* CSD_V1: n=13, csize=2047 -> 2047 << 4 = 32752 */
    CHECK(sectors == 32752u);
}

TEST(ioctl_ctrl_sync_block_size_and_parerr)
{
    reset_all();
    init_card();
    CHECK(SD_ioctl(0, CTRL_SYNC, 0) == RES_OK);
    DWORD bs = 0;
    CHECK(SD_ioctl(0, GET_BLOCK_SIZE, &bs) == RES_OK);
    CHECK(bs == 128u);
    CHECK(SD_ioctl(0, 0x7F, 0) == RES_PARERR);       /* unknown ioctl   */
    CHECK(SD_ioctl(1, CTRL_SYNC, 0) == RES_PARERR);  /* bad drive       */
}

/* ------------------------------------------------------------------ */
/* Speed ladder + the B4 shared-bus fix                                */
/* ------------------------------------------------------------------ */

TEST(fast_speed_ladder_negotiates_fastest_rung)
{
    reset_all();
    init_card();
    SD_SetFastSpeed();
    CHECK(SD_FastSpeedEnabled() == 2);   /* /2 passed first             */
    CHECK(cr1_br() == 0);                /* CR1 shows /2                */
    CHECK(hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_2);
}

TEST(runtime_transaction_reasserts_negotiated_prescaler)
{
    /* THE B4 regression: the display re-inits SPI1 at /16 every frame
     * and the eInk at /8 - before the fix, the next SD op silently ran
     * at whatever the last display left behind, making the negotiated
     * ladder a fiction. Every SD_read/SD_write entry must re-assert it. */
    reset_all();
    init_card();
    SD_SetFastSpeed();                   /* negotiated /2               */
    CHECK(cr1_br() == 0);

    /* "another device" re-inits the bus slow */
    spi1_regs.CR1 = (spi1_regs.CR1 & ~SPI_CR1_BR_Msk) | (0x3 << SPI_CR1_BR_Pos);
    CHECK(cr1_br() == 3);

    uint8_t buf[512];
    CHECK(SD_read(0, buf, 0, 1) == RES_OK);
    CHECK(cr1_br() == 0);                /* SD put its speed back       */

    spi1_regs.CR1 = (spi1_regs.CR1 & ~SPI_CR1_BR_Msk) | (0x2 << SPI_CR1_BR_Pos);
    memset(buf, 0xAA, sizeof(buf));
    CHECK(SD_write(0, buf, 1, 1) == RES_OK);
    CHECK(cr1_br() == 0);
}

TEST(step_down_then_verified_step_up_runs_at_trial_speed)
{
    reset_all();
    init_card();
    SD_SetFastSpeed();
    CHECK(SD_FastSpeedEnabled() == 2);

    CHECK(SD_StepSpeedDown() == 4);
    CHECK(cr1_br() == 1);                /* /4 on the wire              */

    /* the step-up verify must run AT the trial speed: with sd_spi_div
     * still pointing at the old rung, the per-transaction re-assert
     * would have reverted the trial and "verified" the wrong speed */
    card.br_seen = 0;
    CHECK(SD_StepSpeedUpTry() == 2);
    /* every byte the card saw during verification ran at /2 (br=0) */
    CHECK((card.br_seen & ~0x1u) == 0);
    CHECK(cr1_br() == 0);
}

TEST(step_up_rejected_returns_to_previous_rung)
{
    reset_all();
    init_card();
    SD_SetFastSpeed();
    SD_StepSpeedDown();                  /* /4                          */
    card.vary_reads = 1;                 /* reads differ -> verify fails */
    CHECK(SD_StepSpeedUpTry() == 4);     /* rejected, stays /4          */
    CHECK(cr1_br() == 1);
}

TEST(step_down_at_slowest_is_a_hold)
{
    reset_all();
    init_card();
    SD_SetFastSpeed();
    sd_spi_div = 16;                     /* force to the bottom rung    */
    CHECK(SD_StepSpeedDown() == 16);     /* no further rung: hold       */
}

TEST(apply_speed_programs_every_rung)
{
    /* sd_apply_speed's switch must translate each divider correctly */
    reset_all();
    init_card();
    uint8_t buf[512];
    sd_spi_div = 4;  CHECK(SD_read(0, buf, 0, 1) == RES_OK); CHECK(cr1_br() == 1);
    sd_spi_div = 8;  CHECK(SD_read(0, buf, 0, 1) == RES_OK); CHECK(cr1_br() == 2);
    sd_spi_div = 16; CHECK(SD_read(0, buf, 0, 1) == RES_OK); CHECK(cr1_br() == 3);
    sd_spi_div = 99; CHECK(SD_read(0, buf, 0, 1) == RES_OK); CHECK(cr1_br() == 3); /* invalid: untouched */
}

TEST(set_fast_speed_all_rungs_fail_falls_back_to_16)
{
    /* No rung verifies (reads differ): the ladder must land on /16 */
    reset_all();
    init_card();
    card.vary_reads = 1;
    SD_SetFastSpeed();
    CHECK(SD_FastSpeedEnabled() == 16);
    CHECK(cr1_br() == 3);
    CHECK(hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
}

/* ------------------------------------------------------------------ */
/* Direct-register byte pump semantics                                 */
/* ------------------------------------------------------------------ */

TEST(acmd_bit_prefix_sends_cmd55_then_command)
{
    /* Template vestige: a command OR'd with 0x80 gets the CMD55 prefix
     * auto-sent. No production caller uses it (app commands go out as
     * explicit CMD55+CMD pairs), but pin the behavior so the path stays
     * correct if it ever is used. */
    reset_all();
    init_card();
    CHECK(SD_SendCmd((uint8_t)(41 | 0x80), 0, 0xFF, SD_RESPONSE_IN_IDLE) == 0);
}

TEST(direct_pump_flags_and_timeout)
{
    reset_all();
    sd_init_mode = 0;                    /* the runtime path            */

    /* TXE+RXNE both set: write DR, read DR (loopback semantics) */
    spi1_regs.SR = SPI_SR_TXE | SPI_SR_RXNE;
    spi1_regs.DR = 0xAB;
    CHECK(SD_SPI_TxRx(0x55) == 0x55);    /* reads back what was written */

    /* RXNE never sets: the 100 ms deadline returns 0xFF, no hang */
    spi1_regs.SR = SPI_SR_TXE;
    spi1_regs.DR = 0x00;
    Test_SetTickAutoAdvance(1);
    CHECK(SD_SPI_TxRx(0x42) == 0xFF);

    /* TXE never sets: same deadline guard on the write side */
    spi1_regs.SR = 0;
    CHECK(SD_SPI_TxRx(0x42) == 0xFF);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_init_v2hc_card_succeeds();
    run_init_no_card_fails_cleanly();
    run_init_v1_card_takes_the_legacy_path();
    run_init_mmc_card_uses_cmd1();
    run_init_acmd41_warmup_retries_then_succeeds();
    run_init_v1_card_never_leaves_idle_times_out();
    run_init_hal_error_retries_and_recovers();
    run_read_single_block_roundtrip();
    run_read_multi_block_streams_until_cmd12();
    run_read_timeout_when_card_stalls();
    run_write_single_block_roundtrip_and_crc();
    run_write_multi_block_roundtrip();
    run_write_no_busy_pulse_succeeds();
    run_card_idle_at_0xca_still_counted_ready();
    run_command_prewait_times_out_when_card_stays_busy();
    run_write_prewrite_busy_wait_times_out();
    run_write_rejected_token_fails_with_diagnostics();
    run_write_busy_timeout_fails();
    run_ioctl_reports_sector_count();
    run_ioctl_sector_count_v1_csd_formula();
    run_ioctl_ctrl_sync_block_size_and_parerr();
    run_fast_speed_ladder_negotiates_fastest_rung();
    run_runtime_transaction_reasserts_negotiated_prescaler();
    run_step_down_then_verified_step_up_runs_at_trial_speed();
    run_step_up_rejected_returns_to_previous_rung();
    run_step_down_at_slowest_is_a_hold();
    run_apply_speed_programs_every_rung();
    run_set_fast_speed_all_rungs_fail_falls_back_to_16();
    run_direct_pump_flags_and_timeout();
    run_acmd_bit_prefix_sends_cmd55_then_command();
    return TEST_SUMMARY();
}
