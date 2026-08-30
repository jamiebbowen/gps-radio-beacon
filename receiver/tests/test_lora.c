/**
 * @file test_lora.c
 * @brief Host-side unit tests for the SX1268 LoRa driver (lora.c).
 *
 * A behavioral SX1268 chip fake sits behind the SPI/GPIO HAL: it tracks
 * the chip-select line, decodes command transactions, serves register
 * reads (IRQ status, RX buffer, packet status, RSSI) and applies command
 * side effects (mode changes, IRQ clears). Fault knobs cover a stuck
 * BUSY pin, dead SPI, commands failing at any point in the init
 * sequence, and DMA failures/stalls.
 *
 * The tick auto-advances 1 ms per completed SPI transaction and 2 ms per
 * BUSY poll so every timeout loop in the driver terminates naturally.
 *
 * Build & run:  make -C receiver/tests
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "lora.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Clock                                                               */
/* ------------------------------------------------------------------ */

static uint32_t now_ms = 1000;

static void tick_advance(uint32_t ms)
{
    now_ms += ms;
    Test_SetTick(now_ms);
}

/* ------------------------------------------------------------------ */
/* SX1268 chip fake                                                    */
/* ------------------------------------------------------------------ */

static uint8_t  chip_mode        = 2;      /* 2=STDBY_RC, 5=RX, 6=TX */
static uint16_t chip_irq         = 0;      /* latched IRQ register   */
static uint8_t  chip_payload[256];
static uint8_t  chip_payload_len = 0;
static uint8_t  chip_rssi_raw    = 190;    /* -> -95 dBm             */
static uint8_t  chip_snr_raw     = 0x14;   /* +20/4 = +5 dB          */
static uint8_t  chip_rssi_inst   = 200;    /* -> -100 dBm ambient    */
static uint32_t chip_freq_reg    = 0;      /* last programmed PLL word */
static uint8_t  chip_cad_starts  = 0;

/* Fault knobs */
static uint8_t  fake_busy_pin    = 0;      /* BUSY stuck high         */
static uint8_t  fake_status_byte = 0x42;   /* raw GetStatus response  */
static int      fail_after_txns  = -1;     /* -1=off: BUSY jams after N transactions */
static uint8_t  dma_fail         = 0;      /* Receive_DMA returns HAL_ERROR */
static uint8_t  dma_stall        = 0;      /* DMA never completes     */
static uint8_t  txdone_on_tx     = 1;      /* SET_TX latches TX_DONE  */
static uint8_t  irq_during_read  = 0;      /* fire DIO1 mid-read (race test) */

/* Transaction accumulation (CS low .. CS high) */
static uint8_t  txn[300];
static uint16_t txn_len   = 0;
static uint8_t  cs_low    = 0;
static int      txn_count = 0;

/* EXTI pending simulation */
static uint32_t exti_pending = 0;
uint32_t Test_EXTI_GetIt(uint32_t pin)  { return exti_pending & pin; }
void     Test_EXTI_ClearIt(uint32_t pin) { exti_pending &= ~pin; }

static void chip_reset_state(void)
{
    chip_mode = 2;
    chip_irq = 0;
    chip_payload_len = 0;
    fake_busy_pin = 0;
    fake_status_byte = 0x42;
    fail_after_txns = -1;
    dma_fail = 0;
    dma_stall = 0;
    txdone_on_tx = 1;
    txn_len = 0;
    cs_low = 0;
    txn_count = 0;
    exti_pending = 0;
}

/* Apply command side effects when CS rises */
static void chip_handle_txn(void)
{
    if (txn_len == 0) return;
    switch (txn[0]) {
        case SX1268_CMD_SET_STANDBY:  chip_mode = 2; break;
        case SX1268_CMD_SET_RX:       chip_mode = 5; break;
        case SX1268_CMD_SET_TX:
            chip_mode = 6;
            if (txdone_on_tx) chip_irq |= SX1268_IRQ_TX_DONE;
            break;
        case SX1268_CMD_SET_CAD:      chip_cad_starts++; break;
        case SX1268_CMD_CLR_IRQSTATUS:
            if (txn_len >= 3) {
                chip_irq &= (uint16_t)~(((uint16_t)txn[1] << 8) | txn[2]);
            }
            break;
        case SX1268_CMD_SET_RFFREQUENCY:
            if (txn_len >= 5) {
                chip_freq_reg = ((uint32_t)txn[1] << 24) | ((uint32_t)txn[2] << 16)
                              | ((uint32_t)txn[3] << 8) | txn[4];
            }
            break;
        default: break;
    }
    txn_count++;
    tick_advance(1);   /* time passes on the bus */
    if (fail_after_txns >= 0 && txn_count >= fail_after_txns) {
        fake_busy_pin = 1;   /* chip wedges: BUSY never drops again */
    }
}

/* ------------------------------------------------------------------ */
/* HAL fakes                                                           */
/* ------------------------------------------------------------------ */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == LORA_BUSY_PORT && pin == LORA_BUSY_PIN) {
        if (fake_busy_pin) tick_advance(2);   /* let WaitOnBusy time out */
        return fake_busy_pin ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    return GPIO_PIN_RESET;
}

static GPIO_PinState rxen_state = GPIO_PIN_RESET;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    if (port == LORA_CS_PORT && pin == LORA_CS_PIN) {
        if (st == GPIO_PIN_RESET && !cs_low) {         /* select   */
            cs_low = 1;
            txn_len = 0;
        } else if (st == GPIO_PIN_SET && cs_low) {     /* deselect */
            cs_low = 0;
            chip_handle_txn();
        }
    } else if (port == LORA_RXEN_PORT && pin == LORA_RXEN_PIN) {
        rxen_state = st;
    }
    /* RESET pin toggles are accepted silently */
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    for (uint16_t i = 0; i < size && txn_len < sizeof(txn); i++) {
        txn[txn_len++] = data[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data,
                                  uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    memset(data, 0, size);
    if (txn_len == 0) return HAL_OK;

    switch (txn[0]) {
        case SX1268_CMD_GET_STATUS:
            data[0] = fake_status_byte;
            break;
        case SX1268_CMD_GET_IRQSTATUS:
            if (size >= 2) {
                data[0] = (uint8_t)(chip_irq >> 8);
                data[1] = (uint8_t)(chip_irq & 0xFF);
            }
            break;
        case SX1268_CMD_GET_RXBUFFERSTATUS:
            if (size >= 2) {
                data[0] = chip_payload_len;
                data[1] = 0;   /* rx start pointer */
            }
            if (irq_during_read) {
                /* A new packet's DIO1 edge lands mid-read: the ISR bumps
                 * the sequence number after LoRa_ReadPacket's snapshot. */
                extern void EXTI1_IRQHandler(void);
                exti_pending |= LORA_DIO1_PIN;
                EXTI1_IRQHandler();
            }
            break;
        case SX1268_CMD_GET_PACKETSTATUS:
            if (size >= 3) {
                data[0] = chip_rssi_raw;
                data[1] = chip_snr_raw;
                data[2] = 0;
            }
            break;
        case SX1268_CMD_GET_RSSIINST:
            data[0] = chip_rssi_inst;
            break;
        case SX1268_CMD_READ_BUFFER: {
            uint8_t off = (txn_len >= 2) ? txn[1] : 0;
            for (uint16_t i = 0; i < size; i++) {
                data[i] = chip_payload[(off + i) % sizeof(chip_payload)];
            }
            break;
        }
        default:
            break;
    }
    return HAL_OK;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);   /* defined in lora.c */
static SPI_HandleTypeDef *dma_hspi_seen = NULL;

HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *h, uint8_t *data,
                                      uint16_t size)
{
    dma_hspi_seen = h;
    if (dma_fail) return HAL_ERROR;
    if (dma_stall) {
        /* Never complete; the spin timeout is driven by the harness tick
         * auto-advance (see test) - nothing to do here. */
        return HAL_OK;
    }
    /* Complete "instantly": serve data then fire the completion callback */
    HAL_SPI_Receive(h, data, size, 0);
    HAL_SPI_RxCpltCallback(h);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t p, uint32_t s)
{
    (void)irq; (void)p; (void)s;
}
static uint8_t nvic_exti1_enabled = 0;
void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
    if (irq == EXTI1_IRQn) nvic_exti1_enabled = 1;
}
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; }
void HAL_Delay(uint32_t ms) { tick_advance(ms); }

/* Include the module under test (static access: lora_initialized etc.) */
#include "../firmware/src/lora.c"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static SPI_HandleTypeDef hspi;
static DMA_HandleTypeDef hdma_rx;

static void fresh_init(void)
{
    chip_reset_state();
    memset(&hspi, 0, sizeof(hspi));
    hspi.hdmarx = &hdma_rx;
    if (LoRa_Init(&hspi) != LORA_OK) {
        CHECK(0);   /* init must succeed for dependent tests */
    }
}

/** Deliver a packet: load the chip buffer, latch RX_DONE, fire DIO1 */
static void deliver_packet(const uint8_t *data, uint8_t len)
{
    memcpy(chip_payload, data, len);
    chip_payload_len = len;
    chip_irq |= SX1268_IRQ_RX_DONE;
    exti_pending |= LORA_DIO1_PIN;
    EXTI1_IRQHandler();
}

/* ------------------------------------------------------------------ */
/* Init tests                                                          */
/* ------------------------------------------------------------------ */

TEST(test_init_success)
{
    fresh_init();

    CHECK(LoRa_GetChannel() == 0);
    CHECK(rxen_state == GPIO_PIN_SET);         /* RF switch in RX */
    CHECK(chip_mode == 5);                     /* chip left in RX */
    CHECK(nvic_exti1_enabled == 1);            /* DIO1 armed last */
    CHECK(LoRa_PacketAvailable() == 0);
    CHECK(LoRa_GetIRQCount() == 0);

    /* CH0 = 433.0 MHz -> PLL word 433e6 / 32e6 * 2^25 */
    uint32_t expect = (uint32_t)((double)433000000.0 / 32000000.0 * 33554432.0);
    CHECK(chip_freq_reg == expect);
}

TEST(test_init_failure_paths)
{
    /* NULL SPI handle */
    CHECK(LoRa_Init(NULL) == LORA_ERROR);

    /* BUSY stuck high from power-on: 0xB1 */
    chip_reset_state();
    fake_busy_pin = 1;
    CHECK(LoRa_Init(&hspi) == 0xB1);

    /* SPI dead (status reads 0x00): 0xC0 */
    chip_reset_state();
    fake_status_byte = 0x00;
    CHECK(LoRa_Init(&hspi) == 0xC0);
    chip_reset_state();
    fake_status_byte = 0xFF;
    CHECK(LoRa_Init(&hspi) == 0xC0);

    /* Chip wedging (BUSY jams) after each successive command exercises
     * every per-command error return in the init sequence. Collect the
     * distinct codes and verify all of them surfaced. */
    uint8_t seen[32];
    int seen_n = 0;
    for (int n = 1; n <= 18; n++) {
        chip_reset_state();
        fail_after_txns = n;
        uint8_t r = LoRa_Init(&hspi);
        if (r == LORA_OK) break;
        int already = 0;
        for (int i = 0; i < seen_n; i++) if (seen[i] == r) already = 1;
        if (!already && seen_n < 32) seen[seen_n++] = r;
    }
    static const uint8_t expected[] = {
        0xB3,       /* TCXO busy-timeout */
        0xB2, 0xB4, 0xB5, 0xC1, 0xC2, 0xC3, 0xC4, 0xC6, 0xC7, 0xC8,
        0xB9, 0xBA  /* the two SetRx steps */
    };
    for (size_t i = 0; i < sizeof(expected); i++) {
        int found = 0;
        for (int j = 0; j < seen_n; j++) if (seen[j] == expected[i]) found = 1;
        CHECK(found);
    }
}

/* ------------------------------------------------------------------ */
/* Packet path                                                         */
/* ------------------------------------------------------------------ */

TEST(test_receive_small_packet)
{
    fresh_init();

    const uint8_t pkt[5] = {0x05, 1, 2, 3, 42};   /* < DMA threshold */
    deliver_packet(pkt, sizeof(pkt));

    CHECK(LoRa_PacketAvailable() == 1);
    CHECK(LoRa_GetIRQCount() == 1);

    LoRa_Packet_t p;
    memset(&p, 0, sizeof(p));
    CHECK(LoRa_ReadPacket(&p) == LORA_OK);
    CHECK(p.length == 5);
    CHECK(memcmp(p.data, pkt, 5) == 0);
    CHECK(p.crc_error == false);
    CHECK(p.rssi == -95);                      /* raw 190 / -2 */
    CHECK(p.snr == 5);                         /* raw 0x14 / 4 */
    CHECK(LoRa_GetRSSI() == -95);
    CHECK(LoRa_GetSNR() == 5);
    CHECK(LoRa_PacketAvailable() == 0);        /* flag consumed */
}

TEST(test_receive_negative_snr)
{
    fresh_init();
    chip_snr_raw = 0xF8;                       /* -8/4 = -2 dB */
    const uint8_t pkt[3] = {1, 2, 3};
    deliver_packet(pkt, sizeof(pkt));

    LoRa_Packet_t p;
    CHECK(LoRa_ReadPacket(&p) == LORA_OK);
    CHECK(p.snr == -2);                        /* signed, not +62 */
}

TEST(test_receive_dma_packet_and_dma_faults)
{
    fresh_init();

    uint8_t big[32];
    for (int i = 0; i < 32; i++) big[i] = (uint8_t)i;
    deliver_packet(big, sizeof(big));

    LoRa_Packet_t p;
    CHECK(LoRa_ReadPacket(&p) == LORA_OK);     /* DMA path */
    CHECK(p.length == 32);
    CHECK(memcmp(p.data, big, 32) == 0);
    CHECK(dma_hspi_seen == &hspi);

    /* DMA start failure -> error path with RX re-entry */
    dma_fail = 1;
    deliver_packet(big, sizeof(big));
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    dma_fail = 0;

    /* DMA never completes -> abort + error. The driver's spin-wait makes
     * no HAL calls, so the tick needs the systick-style auto-advance for
     * its 50 ms timeout to fire. */
    dma_stall = 1;
    deliver_packet(big, sizeof(big));
    Test_SetTickAutoAdvance(25);
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    Test_SetTickAutoAdvance(0);
    dma_stall = 0;

    /* Foreign-SPI DMA completion must be ignored */
    SPI_HandleTypeDef other;
    lora_spi_dma_busy = 1;
    HAL_SPI_RxCpltCallback(&other);
    CHECK(lora_spi_dma_busy == 1);
    lora_spi_dma_busy = 0;
}

TEST(test_crc_error_and_no_data)
{
    fresh_init();

    /* CRC error IRQ */
    chip_irq |= SX1268_IRQ_CRC_ERROR;
    exti_pending |= LORA_DIO1_PIN;
    EXTI1_IRQHandler();
    LoRa_Packet_t p;
    memset(&p, 0, sizeof(p));
    CHECK(LoRa_ReadPacket(&p) == LORA_CRC_ERROR);
    CHECK(p.crc_error == true);
    CHECK(LoRa_PacketAvailable() == 0);
    CHECK(chip_irq == 0);                      /* IRQs cleared */

    /* Spurious flag with no IRQ latched: NO_DATA, flag consumed */
    lora_packet_ready = 1;
    CHECK(LoRa_ReadPacket(&p) == LORA_NO_DATA);
    CHECK(LoRa_PacketAvailable() == 0);

    /* RX_DONE with an empty buffer: error path forces RX re-entry */
    chip_payload_len = 0;
    chip_irq |= SX1268_IRQ_RX_DONE;
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    CHECK(chip_mode == 5);                     /* back in RX */

    CHECK(LoRa_ReadPacket(NULL) == LORA_ERROR);
}

TEST(test_irq_race_keeps_flag)
{
    /* If a new DIO1 edge fires during the read, the packet_ready flag
     * must survive so the next loop iteration reads the second packet. */
    fresh_init();
    const uint8_t pkt[3] = {9, 9, 9};
    deliver_packet(pkt, sizeof(pkt));

    LoRa_Packet_t p;
    irq_during_read = 1;                       /* DIO1 fires mid-read */
    CHECK(LoRa_ReadPacket(&p) == LORA_OK);
    irq_during_read = 0;
    CHECK(LoRa_PacketAvailable() == 1);        /* NOT consumed */
    lora_packet_ready = 0;
}

TEST(test_exti_handler_gating)
{
    fresh_init();
    /* No pending bit: handler must do nothing */
    exti_pending = 0;
    uint32_t before = LoRa_GetIRQCount();
    EXTI1_IRQHandler();
    CHECK(LoRa_GetIRQCount() == before);
    CHECK(LoRa_PacketAvailable() == 0);
}

/* ------------------------------------------------------------------ */
/* Channel switching                                                   */
/* ------------------------------------------------------------------ */

TEST(test_set_channel)
{
    fresh_init();

    CHECK(LoRa_SetChannel(LORA_CHANNEL_COUNT) == LORA_ERROR);  /* range */
    CHECK(LoRa_SetChannel(0) == LORA_OK);                      /* same: no-op */

    CHECK(LoRa_SetChannel(3) == LORA_OK);
    CHECK(LoRa_GetChannel() == 3);
    /* Driver arithmetic: float MHz * 1e6 (float!), then double scaling */
    uint32_t hz = (uint32_t)(LORA_CHANNEL_FREQ_MHZ(3) * 1000000.0f);
    uint32_t expect = (uint32_t)((double)hz / 32000000.0 * 33554432.0);
    CHECK(chip_freq_reg == expect);
    CHECK(chip_mode == 5);                     /* re-entered RX */

    /* Pending packet from the old channel must be dropped */
    lora_packet_ready = 1;
    CHECK(LoRa_SetChannel(4) == LORA_OK);
    CHECK(LoRa_PacketAvailable() == 0);

    /* Chip wedges during the retune: error surfaces */
    fail_after_txns = txn_count + 1;
    CHECK(LoRa_SetChannel(5) == LORA_ERROR);
    fake_busy_pin = 0;
    fail_after_txns = -1;
}

/* ------------------------------------------------------------------ */
/* CAD                                                                 */
/* ------------------------------------------------------------------ */

TEST(test_cad_flow)
{
    fresh_init();

    CHECK(LoRa_StartCad() == LORA_OK);
    CHECK(chip_cad_starts == 1);

    /* CAD still running: no CAD_DONE latched */
    CHECK(LoRa_CadResult() == LORA_CAD_PENDING);

    /* Done, nothing heard */
    chip_irq |= SX1268_IRQ_CAD_DONE;
    CHECK(LoRa_CadResult() == LORA_CAD_NONE);
    CHECK((chip_irq & SX1268_IRQ_CAD_DONE) == 0);   /* flags cleared */

    /* Done + preamble detected */
    chip_irq |= SX1268_IRQ_CAD_DONE | SX1268_IRQ_CAD_DETECTED;
    CHECK(LoRa_CadResult() == LORA_CAD_DETECTED);

    /* Chip wedged: StartCad fails cleanly */
    fake_busy_pin = 1;
    CHECK(LoRa_StartCad() == LORA_ERROR);
    fake_busy_pin = 0;
}

/* ------------------------------------------------------------------ */
/* Status / misc accessors                                             */
/* ------------------------------------------------------------------ */

TEST(test_status_and_accessors)
{
    fresh_init();

    uint8_t status = 0;
    CHECK(LoRa_GetDeviceStatus(&status) == LORA_OK);
    CHECK(status == fake_status_byte);
    CHECK(LoRa_GetDeviceStatus(NULL) == LORA_ERROR);

    fake_busy_pin = 1;
    CHECK(LoRa_GetDeviceStatus(&status) == LORA_BUSY);
    fake_busy_pin = 0;

    CHECK(LoRa_GetStatus() == fake_status_byte);

    int16_t rssi = 0;
    CHECK(LoRa_GetRssiInst(&rssi) == LORA_OK);
    CHECK(rssi == -100);                       /* raw 200 / -2 */
    CHECK(LoRa_GetRssiInst(NULL) == LORA_ERROR);

    uint16_t irq = 0xFFFF;
    chip_irq = 0x0203;
    CHECK(LoRa_GetIRQStatus(&irq) == LORA_OK);
    CHECK(irq == 0x0203);
    CHECK(LoRa_ClearIRQStatus(0x0203) == LORA_OK);
    CHECK(chip_irq == 0);

    CHECK(LoRa_SetReceiveMode() == LORA_OK);
    CHECK(chip_mode == 5);
    CHECK(LoRa_SetStandbyMode() == LORA_OK);
    CHECK(chip_mode == 2);
    (void)LoRa_SetReceiveMode();
}

TEST(test_uninitialized_guards)
{
    uint8_t saved = lora_initialized;
    lora_initialized = 0;

    LoRa_Packet_t p;
    int16_t rssi;
    CHECK(LoRa_SetReceiveMode() == LORA_ERROR);
    CHECK(LoRa_SetStandbyMode() == LORA_ERROR);
    CHECK(LoRa_PacketAvailable() == 0);
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    CHECK(LoRa_SetChannel(1) == LORA_ERROR);
    CHECK(LoRa_StartCad() == LORA_ERROR);
    CHECK(LoRa_CadResult() == LORA_CAD_FAIL);
    CHECK(LoRa_GetRssiInst(&rssi) == LORA_ERROR);
    CHECK(LoRa_Transmit((const uint8_t *)"x", 1) == LORA_ERROR);

    lora_initialized = saved;
}

/* ------------------------------------------------------------------ */
/* Transmit (kept-safe path)                                           */
/* ------------------------------------------------------------------ */

TEST(test_transmit)
{
    fresh_init();

    CHECK(LoRa_Transmit(NULL, 5) == LORA_ERROR);
    CHECK(LoRa_Transmit((const uint8_t *)"x", 0) == LORA_ERROR);

    /* Success: TX_DONE latches on SET_TX */
    const uint8_t msg[4] = {1, 2, 3, 4};
    CHECK(LoRa_Transmit(msg, sizeof(msg)) == LORA_OK);
    CHECK(chip_mode == 5);                     /* restored to RX */
    CHECK(rxen_state == GPIO_PIN_SET);         /* RF switch back to RX */

    /* TX_DONE never arrives: 1 s timeout, still restored to RX */
    txdone_on_tx = 0;
    CHECK(LoRa_Transmit(msg, sizeof(msg)) == LORA_TIMEOUT);
    CHECK(chip_mode == 5);
    txdone_on_tx = 1;
}

TEST(test_command_failure_propagation)
{
    fresh_init();
    LoRa_Packet_t p;
    memset(&p, 0, sizeof(p));
    const uint8_t pkt[5] = {0x05, 1, 2, 3, 42};

    /* Fully wedged chip: every WaitOnBusy times out */
    fake_busy_pin = 1;
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);       /* GetIRQStatus fails */
    CHECK(LoRa_CadResult() == LORA_CAD_FAIL);
    uint16_t irq = 0;
    CHECK(LoRa_GetIRQStatus(&irq) == LORA_ERROR);   /* ReadCommand busy ret */
    CHECK(LoRa_Transmit(pkt, sizeof(pkt)) == LORA_ERROR); /* WriteBuffer busy */
    fake_busy_pin = 0;

    /* ReadPacket: IRQ read succeeds (RX_DONE), buffer-status read fails */
    deliver_packet(pkt, sizeof(pkt));
    fail_after_txns = txn_count + 1;                /* jam after GET_IRQSTATUS */
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    fail_after_txns = -1; fake_busy_pin = 0;

    /* ReadPacket: payload read fails (ReadBuffer busy) -> error-recovery
     * tail re-enters RX */
    deliver_packet(pkt, sizeof(pkt));
    fail_after_txns = txn_count + 2;                /* jam after RXBUFFERSTATUS */
    CHECK(LoRa_ReadPacket(&p) == LORA_ERROR);
    fail_after_txns = -1; fake_busy_pin = 0;

    /* SetChannel: standby OK, frequency command fails -> best-effort
     * retune back to the old channel, then LORA_ERROR */
    fresh_init();
    CHECK(LoRa_SetChannel(2) == LORA_OK);           /* known-good baseline */
    fail_after_txns = txn_count + 2;                /* jam after standby txn */
    CHECK(LoRa_SetChannel(3) == LORA_ERROR);
    CHECK(LoRa_GetChannel() == 2);                  /* stayed on old channel */
    fail_after_txns = -1; fake_busy_pin = 0;

    /* SetChannel: retune OK, RX re-entry fails */
    fail_after_txns = txn_count + 3;                /* jam after standby+freq */
    CHECK(LoRa_SetChannel(4) == LORA_ERROR);
    fail_after_txns = -1; fake_busy_pin = 0;

    /* StartCad: CADPARAMS command fails (standby succeeded) */
    fail_after_txns = txn_count + 2;
    CHECK(LoRa_StartCad() == LORA_ERROR);
    fail_after_txns = -1; fake_busy_pin = 0;

    /* StartCad: CADPARAMS OK, stale-flag clear fails */
    fail_after_txns = txn_count + 3;
    CHECK(LoRa_StartCad() == LORA_ERROR);
    fail_after_txns = -1; fake_busy_pin = 0;
}

/* ------------------------------------------------------------------ */

int main(void)
{
    Test_SetTick(now_ms);

    run_test_init_success();
    run_test_init_failure_paths();
    run_test_receive_small_packet();
    run_test_receive_negative_snr();
    run_test_receive_dma_packet_and_dma_faults();
    run_test_crc_error_and_no_data();
    run_test_irq_race_keeps_flag();
    run_test_exti_handler_gating();
    run_test_set_channel();
    run_test_cad_flow();
    run_test_status_and_accessors();
    run_test_uninitialized_guards();
    run_test_transmit();
    run_test_command_failure_propagation();

    return TEST_SUMMARY();
}
