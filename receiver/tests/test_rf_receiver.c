/**
 * @file test_rf_receiver.c
 * @brief Host-side unit tests for the receiver RF glue logic.
 *
 * rf_receiver.c is compiled unmodified against a fake LoRa driver (defined
 * below) and the HAL stubs, with the real rf_parser.c linked in. Covers:
 *   - channel cycling across the FULL 8-channel plan (regression: a stale
 *     build once shipped '% 4' and the RX only cycled CH0-3)
 *   - channel switching side effects (pending data + heartbeat reset)
 *   - heartbeat one-shot and non-consuming getters + age tracking
 *   - boot channel scan: dwell hop, lock on any packet, manual stop
 *   - ambient noise-floor estimator: percentile robustness against
 *     mid-packet contamination, alert engage/clear hysteresis
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "rf_receiver.h"
#include "rf_parser.h"
#include "packet_format.h"
#include "lora.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* Fake LoRa driver (rf_receiver.c links against these, not lora.c)    */
/* ------------------------------------------------------------------ */

static uint8_t       fake_channel   = 0;
static uint8_t       fake_mode      = 5;     /* 5 = RX (ChipMode bits [6:4]) */
static int16_t       fake_rssi_inst = -120;  /* Ambient level "measured"   */
static LoRa_Packet_t fake_pkt;
static uint8_t       fake_pkt_pending = 0;

/* Failure knobs: init/error paths under test */
static uint8_t       fake_lora_init_result   = LORA_OK;
static uint8_t       fake_device_status_fail = 0;
static uint16_t      fake_irq_value          = 0;

static uint32_t      fake_set_rx_calls       = 0;
static uint32_t      fake_init_calls         = 0;
uint8_t LoRa_Init(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    fake_init_calls++;
    return fake_lora_init_result;
}
uint8_t LoRa_SetReceiveMode(void) { fake_set_rx_calls++; return LORA_OK; }
uint8_t LoRa_GetDeviceStatus(uint8_t *status)
{
    if (fake_device_status_fail) { *status = 0; return LORA_ERROR; }
    *status = (uint8_t)((fake_mode << 4) | 0x02);  /* ChipMode at [6:4] */
    return LORA_OK;
}
uint8_t LoRa_GetIRQStatus(uint16_t *irq) { *irq = fake_irq_value; return LORA_OK; }
uint8_t LoRa_GetRssiInst(int16_t *rssi) { *rssi = fake_rssi_inst; return LORA_OK; }
uint8_t LoRa_PacketAvailable(void) { return fake_pkt_pending; }
uint8_t LoRa_ReadPacket(LoRa_Packet_t *pkt)
{
    if (!fake_pkt_pending) return LORA_ERROR;
    *pkt = fake_pkt;
    fake_pkt_pending = 0;
    return LORA_OK;
}
uint8_t LoRa_SetChannel(uint8_t ch)
{
    if (ch >= LORA_CHANNEL_COUNT) return LORA_ERROR;
    fake_channel = ch;
    return LORA_OK;
}
uint8_t  LoRa_GetChannel(void) { return fake_channel; }
uint32_t LoRa_GetIRQCount(void) { return 0; }

/* CAD fake: unavailable by default, so the scan degrades to the dwell phase
 * on its first update (mirrors a chip that rejects SetCad) and the dwell
 * tests exercise the exact fallback path real hardware would take. */
static uint8_t  fake_cad_available = 0;
static uint8_t  fake_cad_result    = LORA_CAD_NONE;
static uint32_t fake_cad_starts    = 0;
uint8_t LoRa_StartCad(void)
{
    if (!fake_cad_available) return LORA_ERROR;
    fake_cad_starts++;
    return LORA_OK;
}
uint8_t LoRa_CadResult(void) { return fake_cad_result; }

/* Include the module under test directly (not linked - see tests/Makefile)
 * so tests can reach static state (rf_spi_test_result, last_rssi, ...) to
 * exercise one-shot diagnostics and error paths without hardware. */
#include "../firmware/src/rf_receiver.c"

/* HAL fakes (declared in the stub headers) */
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin)
{
    (void)p; (void)pin; return GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{
    (void)p; (void)pin; (void)s;
}
/* MCU peripheral init knobs. fake_dma_fail_at: 0=never, 1=first HAL_DMA_Init
 * call fails (SPI2 TX stream), 2=second fails (RX stream). */
static uint8_t fake_spi_init_fail = 0;
static uint8_t fake_dma_fail_at   = 0;
static uint8_t fake_dma_calls     = 0;
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    (void)h;
    return fake_spi_init_fail ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h)
{
    (void)h;
    fake_dma_calls++;
    return (fake_dma_fail_at && fake_dma_calls == fake_dma_fail_at)
               ? HAL_ERROR : HAL_OK;
}
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t p, uint32_t s)
{
    (void)irq; (void)p; (void)s;
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }
void HAL_Delay(uint32_t ms) { (void)ms; }

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint32_t now_ms = 1000;

/* Advance fake time and poll DataAvailable once per step, mimicking the
 * main loop. step_ms should be <= 250 to keep the diag path (which refreshes
 * the cached device mode) running at its real cadence. */
static void run_for(uint32_t duration_ms, uint32_t step_ms)
{
    uint32_t end = now_ms + duration_ms;
    while (now_ms < end) {
        now_ms += step_ms;
        Test_SetTick(now_ms);
        (void)RF_Receiver_DataAvailable();
    }
}

static void inject_heartbeat(uint8_t rocket_id, uint8_t channel,
                             uint8_t sats, uint16_t uptime_s)
{
    HeartbeatPacket_t hb = {
        .packet_type = PACKET_TYPE_HEARTBEAT,
        .rocket_id   = rocket_id,
        .channel     = channel,
        .satellites  = sats,
        .fix_quality = 0,
        .uptime_s    = uptime_s,
        .gps_health  = HB_GPS_HEALTH(HB_GPS_ACQUIRING, 0),
    };
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    memcpy(fake_pkt.data, &hb, sizeof(hb));
    fake_pkt.length = HEARTBEAT_PACKET_SIZE;
    fake_pkt.rssi = -87;
    fake_pkt.snr = 6;
    fake_pkt_pending = 1;
}

/* Legacy 7-byte heartbeat (pre-gps_health TX firmware) */
static void inject_heartbeat_v1(uint8_t rocket_id, uint8_t channel)
{
    inject_heartbeat(rocket_id, channel, 2, 99);
    /* Poison the byte beyond the V1 length: the receiver must never read
     * it (zero-fill), or a stale radio buffer would decode as a verdict. */
    fake_pkt.data[HEARTBEAT_PACKET_SIZE_V1] = 0xAB;
    fake_pkt.length = HEARTBEAT_PACKET_SIZE_V1;
}

static void put_i32_le(uint8_t *buf, int32_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void inject_gps_packet(double lat_deg, double lon_deg)
{
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    fake_pkt.data[0] = PACKET_TYPE_GPS;
    put_i32_le(&fake_pkt.data[1], (int32_t)(lat_deg * 10000000.0));
    put_i32_le(&fake_pkt.data[5], (int32_t)(lon_deg * 10000000.0));
    fake_pkt.data[9] = 100; fake_pkt.data[10] = 0;  /* alt 356 m -> 0x0164 */
    fake_pkt.data[11] = 9;                           /* sats */
    fake_pkt.data[12] = 0x03;                        /* fix flags */
    fake_pkt.length = 13;
    fake_pkt.rssi = -95;
    fake_pkt.snr = 4;
    fake_pkt_pending = 1;
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_channel_cycling_covers_all_channels)
{
    CHECK(RF_Receiver_GetChannel() == 0);

    /* Regression: a stale object once baked '% 4' into NextChannel and the
     * receiver could only reach CH0-3. Walk the full wheel. */
    for (uint8_t i = 1; i <= LORA_CHANNEL_COUNT; i++) {
        uint8_t ch = RF_Receiver_NextChannel();
        CHECK(ch == (i % LORA_CHANNEL_COUNT));
    }
    CHECK(RF_Receiver_GetChannel() == 0);  /* wrapped all the way around */

    CHECK(RF_Receiver_SetChannel(LORA_CHANNEL_COUNT - 1) == RF_OK);
    CHECK(RF_Receiver_GetChannel() == LORA_CHANNEL_COUNT - 1);
    CHECK(RF_Receiver_SetChannel(LORA_CHANNEL_COUNT) == RF_ERROR);
    CHECK(RF_Receiver_GetChannel() == LORA_CHANNEL_COUNT - 1);
    CHECK(RF_Receiver_SetChannel(0) == RF_OK);
}

TEST(test_callsign_captured_in_binary_mode)
{
    /* The callsign packet has no type byte and variable length, so the
     * binary dispatch used to drop it silently. It must reach the ASCII
     * parser (which classifies comma-free packets as callsigns) without
     * being mistaken for position data. */
    const char *cs = "KE0MZS-2 CH3";
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    memcpy(fake_pkt.data, cs, strlen(cs));
    fake_pkt.length = (uint16_t)strlen(cs);
    fake_pkt.rssi = -80;
    fake_pkt.snr = 5;
    fake_pkt_pending = 1;
    run_for(250, 250);

    /* Not position data: same policy as heartbeats */
    CHECK(RF_Receiver_DataAvailable() == 0);
    CHECK(RF_Receiver_GetLastPacketTime() == 0);

    /* But the parser retained it */
    GPS_Data scratch;
    char heard[RF_PARSER_MAX_CALLSIGN_LEN] = "";
    CHECK(RF_Receiver_GetParsedData(&scratch, heard, sizeof(heard), NULL) == 1);
    CHECK(strcmp(heard, cs) == 0);

    /* Binary garbage is NOT a callsign */
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    fake_pkt.data[0] = 0x42;
    fake_pkt.data[1] = 0x00;
    fake_pkt.data[2] = 0xFF;
    fake_pkt.data[3] = 0x10;
    fake_pkt.data[4] = 0x99;
    fake_pkt.length = 5;
    fake_pkt_pending = 1;
    run_for(250, 250);
    heard[0] = '\0';
    CHECK(RF_Receiver_GetParsedData(&scratch, heard, sizeof(heard), NULL) == 1);
    CHECK(strcmp(heard, cs) == 0);              /* unchanged */
}

TEST(test_heartbeat_lifecycle)
{
    HeartbeatPacket_t hb;
    uint32_t age_ms = 0;

    /* Nothing heard yet */
    CHECK(RF_Receiver_GetHeartbeat(&hb) == 0);
    CHECK(RF_Receiver_GetLastHeartbeat(&hb, &age_ms) == 0);

    inject_heartbeat(3, 5, 7, 42);
    run_for(250, 250);

    /* A heartbeat is NOT position data */
    CHECK(RF_Receiver_DataAvailable() == 0);
    CHECK(RF_Receiver_GetLastPacketTime() == 0);

    /* One-shot getter: first call yes, second no */
    CHECK(RF_Receiver_GetHeartbeat(&hb) == 1);
    CHECK(hb.rocket_id == 3 && hb.channel == 5 && hb.satellites == 7);
    CHECK(hb.uptime_s == 42);
    CHECK(HB_GPS_STATE(hb.gps_health) == HB_GPS_ACQUIRING);
    CHECK(HB_GPS_RESETS(hb.gps_health) == 0);
    CHECK(RF_Receiver_GetHeartbeat(&hb) == 0);

    /* Non-consuming getter keeps returning it, with a growing age */
    CHECK(RF_Receiver_GetLastHeartbeat(&hb, &age_ms) == 1);
    CHECK(hb.rocket_id == 3);
    run_for(3000, 250);
    CHECK(RF_Receiver_GetLastHeartbeat(NULL, &age_ms) == 1);
    CHECK(age_ms >= 3000 && age_ms <= 3500);

    /* Heartbeat RSSI/SNR land in the signal-quality getters */
    int16_t rssi; int8_t snr;
    RF_Receiver_GetSignalQuality(&rssi, &snr);
    CHECK(rssi == -87 && snr == 6);

    /* Channel switch discards the old rocket's heartbeat */
    CHECK(RF_Receiver_NextChannel() == 1);
    CHECK(RF_Receiver_GetLastHeartbeat(&hb, &age_ms) == 0);
    CHECK(RF_Receiver_SetChannel(0) == RF_OK);

    /* Legacy 7-byte heartbeat (old TX firmware): still accepted, and the
     * absent gps_health byte decodes as UNKNOWN - never as leftover bytes
     * from whatever the radio buffer held before. */
    inject_heartbeat_v1(4, 6);
    run_for(250, 250);
    CHECK(RF_Receiver_GetHeartbeat(&hb) == 1);
    CHECK(hb.rocket_id == 4 && hb.channel == 6);
    CHECK(hb.uptime_s == 99);
    CHECK(hb.gps_health == HB_GPS_UNKNOWN);
    CHECK(RF_Receiver_SetChannel(0) == RF_OK);
}

TEST(test_position_packet_flow)
{
    inject_gps_packet(40.0, -105.0);
    run_for(250, 250);

    CHECK(RF_Receiver_DataAvailable() == 1);
    CHECK(RF_Receiver_GetLastPacketTime() == now_ms);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(RF_Receiver_GetGPSData(&gps) == RF_OK);
    CHECK_NEAR(gps.latitude, 40.0, 1e-5);
    CHECK_NEAR(gps.longitude, -105.0, 1e-5);
    CHECK(gps.satellites == 9);

    /* Consuming the packet clears the ready flag */
    CHECK(RF_Receiver_DataAvailable() == 0);

    /* Freshness: stale after the timeout, not before */
    CHECK(RF_Receiver_IsDataStale(now_ms + 1000) == 0);
    CHECK(RF_Receiver_IsDataStale(now_ms + 600000) == 1);
}

TEST(test_channel_scan_locks_on_packet)
{
    CHECK(RF_Receiver_GetChannel() == 0);

    RF_Receiver_StartScan();
    CHECK(RF_Receiver_IsScanning() == 1);

    /* Nothing on the air: one full dwell must hop exactly one channel */
    uint32_t hops = 0;
    uint32_t start_ms = now_ms;
    while (now_ms - start_ms < 7000) {
        now_ms += 250;
        Test_SetTick(now_ms);
        (void)RF_Receiver_DataAvailable();
        if (RF_Receiver_ScanUpdate()) break;
    }
    hops = RF_Receiver_GetChannel();
    CHECK(hops == 1);
    CHECK(RF_Receiver_IsScanning() == 1);

    /* Any CRC-valid packet (even a heartbeat) locks the scan */
    inject_heartbeat(2, 1, 3, 10);
    run_for(250, 250);
    CHECK(RF_Receiver_ScanUpdate() == 1);
    CHECK(RF_Receiver_IsScanning() == 0);
    CHECK(RF_Receiver_GetChannel() == 1);

    /* A packet sitting unread in the radio at dwell expiry must block the
     * hop (hopping would discard it and the scan would cycle past a live
     * beacon), then lock once the main loop reads it. */
    RF_Receiver_StartScan();
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* CAD refused -> dwell phase */
    inject_heartbeat(2, 1, 4, 20);          /* pending, NOT yet read */
    now_ms += 7000;                          /* dwell long expired */
    Test_SetTick(now_ms);
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* no lock yet... */
    CHECK(RF_Receiver_GetChannel() == 1);    /* ...but no hop either */
    run_for(250, 250);                       /* main loop reads the packet */
    CHECK(RF_Receiver_ScanUpdate() == 1);    /* now it locks */
    CHECK(RF_Receiver_GetChannel() == 1);

    /* Manual stop */
    RF_Receiver_StartScan();
    RF_Receiver_StopScan();
    CHECK(RF_Receiver_IsScanning() == 0);
    CHECK(RF_Receiver_ScanUpdate() == 0);

    CHECK(RF_Receiver_SetChannel(0) == RF_OK);
}

TEST(test_channel_scan_cad_fast_phase)
{
    CHECK(RF_Receiver_GetChannel() == 0);
    fake_cad_available = 1;
    fake_cad_result    = LORA_CAD_NONE;
    
    /* Quiet band: every CAD comes back clean and the scan laps the whole
     * wheel in update pairs (start CAD, read result+hop) - no dwell time. */
    RF_Receiver_StartScan();
    fake_cad_starts = 0;
    for (uint8_t i = 0; i < 2 * LORA_CHANNEL_COUNT; i++) {
        now_ms += 5;                         /* ~real polling cadence */
        Test_SetTick(now_ms);
        CHECK(RF_Receiver_ScanUpdate() == 0);
    }
    CHECK(fake_cad_starts == LORA_CHANNEL_COUNT);
    CHECK(RF_Receiver_GetChannel() == 0);    /* full lap, wrapped around */
    CHECK(RF_Receiver_IsScanning() == 1);
    
    /* Preamble detected: the scan parks in RX-wait, the packet arrives,
     * and the very next update locks on the channel that sniffed it. */
    fake_cad_result = LORA_CAD_DETECTED;
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* IDLE: starts a CAD */
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* RUNNING: sees DETECTED */
    CHECK(RF_Receiver_GetChannel() == 0);    /* no hop while receiving */
    inject_heartbeat(1, 0, 5, 30);
    run_for(250, 250);                       /* main loop reads the packet */
    CHECK(RF_Receiver_ScanUpdate() == 1);    /* locked */
    CHECK(RF_Receiver_IsScanning() == 0);
    CHECK(RF_Receiver_GetChannel() == 0);
    
    /* False detection (noise burst, nothing decodable): after the RX-wait
     * grace period the scan gives up on the channel and resumes sniffing. */
    RF_Receiver_StartScan();
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* start CAD */
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* DETECTED -> RX wait */
    now_ms += 900;                           /* > RX-wait grace (800 ms) */
    Test_SetTick(now_ms);
    CHECK(RF_Receiver_ScanUpdate() == 0);
    CHECK(RF_Receiver_GetChannel() == 1);    /* hopped on, still scanning */
    CHECK(RF_Receiver_IsScanning() == 1);
    
    /* Fast-phase timeout: with CAD never detecting, the scan must fall
     * back to the dwell phase (the 52 s guarantee) instead of trusting
     * CAD forever - a weak/odd signal may be receivable yet CAD-invisible. */
    fake_cad_result = LORA_CAD_NONE;
    now_ms += 31000;                         /* > RF_SCAN_CAD_PHASE_MS */
    Test_SetTick(now_ms);
    fake_cad_starts = 0;
    CHECK(RF_Receiver_ScanUpdate() == 0);    /* falls back to dwell */
    run_for(7000, 250);                      /* one dwell, no CAD activity */
    (void)RF_Receiver_ScanUpdate();
    CHECK(fake_cad_starts == 0);             /* CAD phase really over */
    CHECK(RF_Receiver_IsScanning() == 1);    /* dwell scan carries on */
    
    /* A packet still locks the dwell fallback */
    inject_heartbeat(2, 2, 6, 40);
    run_for(250, 250);
    CHECK(RF_Receiver_ScanUpdate() == 1);
    
    /* Restore defaults for any later tests */
    fake_cad_available = 0;
    CHECK(RF_Receiver_SetChannel(0) == RF_OK);
}

TEST(test_noise_floor_estimation_and_alert)
{
    int16_t nf = 0;

    /* Channel switch in the previous test reset the window */
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 0);

    /* Quiet channel: floor converges on the ambient level, no alert */
    fake_rssi_inst = -120;
    run_for(10000, 250);   /* 10 samples at 1 Hz */
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 1);
    CHECK(nf == -120);
    CHECK(RF_Receiver_NoiseAlert() == 0);

    /* A few strong mid-packet samples must NOT drag the floor up:
     * the 25th-percentile estimator ignores the upper tail. */
    fake_rssi_inst = -60;
    run_for(4000, 250);    /* 4 contaminated samples in a 16-window */
    fake_rssi_inst = -120;
    run_for(2000, 250);
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 1);
    CHECK(nf == -120);
    CHECK(RF_Receiver_NoiseAlert() == 0);

    /* Genuine interference lifts the whole window -> alert engages */
    fake_rssi_inst = -90;
    run_for(17000, 250);   /* refill the entire window */
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 1);
    CHECK(nf == -90);
    CHECK(RF_Receiver_NoiseAlert() == 1);

    /* Hysteresis: floor between clear (-105) and alert (-100) keeps the
     * alert latched... */
    fake_rssi_inst = -103;
    run_for(17000, 250);
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 1);
    CHECK(nf == -103);
    CHECK(RF_Receiver_NoiseAlert() == 1);

    /* ...and only a floor at/below the clear threshold releases it */
    fake_rssi_inst = -110;
    run_for(17000, 250);
    CHECK(RF_Receiver_NoiseAlert() == 0);

    /* Sampling is gated on the radio being in RX mode */
    CHECK(RF_Receiver_SetChannel(1) == RF_OK);   /* resets the window */
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 0);
    fake_mode = 2;                               /* STDBY */
    run_for(20000, 250);
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 0);  /* no samples collected */
    fake_mode = 5;                               /* back to RX */
    run_for(10000, 250);
    CHECK(RF_Receiver_GetNoiseFloor(&nf) == 1);
}

/* ------------------------------------------------------------------ */

TEST(test_init_failure_paths)
{
    /* Regression: these paths were while(1) hangs - a blank screen
     * indistinguishable from a dead battery. Each failure must surface
     * as a distinct error code so the boot screen can name the culprit
     * and main's 10 s retry loop gets a chance to recover. */
    fake_spi_init_fail = 1;
    CHECK(RF_Receiver_Init() == RF_ERR_SPI_INIT);
    fake_spi_init_fail = 0;

    fake_dma_calls = 0; fake_dma_fail_at = 1;
    CHECK(RF_Receiver_Init() == RF_ERR_DMA_TX_INIT);

    fake_dma_calls = 0; fake_dma_fail_at = 2;
    CHECK(RF_Receiver_Init() == RF_ERR_DMA_RX_INIT);
    fake_dma_fail_at = 0;

    /* LoRa chip errors pass through untranslated (0xB* identifies the
     * exact bring-up step on the boot screen) */
    fake_lora_init_result = 0xB1;
    CHECK(RF_Receiver_Init() == 0xB1);
    fake_lora_init_result = LORA_OK;

    /* Device status unreadable after an otherwise good init: init still
     * succeeds (the radio may just be busy) but the SPI self-test result
     * must read "fail" on the diagnostics screen, not "pass". */
    fake_device_status_fail = 1;
    CHECK(RF_Receiver_Init() == RF_OK);
    uint8_t spi_test = 0xFF;
    RF_Receiver_GetIRQDiagnostics(NULL, NULL, NULL, &spi_test, NULL);
    CHECK(spi_test == 0);
    fake_device_status_fail = 0;
}

TEST(test_spi_selftest_oneshot_in_poll)
{
    /* If init never ran the self-test (result still 0xFF), the first
     * DataAvailable() poll runs it once. Exercise both verdicts. */
    rf_spi_test_result = 0xFF;
    fake_device_status_fail = 1;
    now_ms += 251; Test_SetTick(now_ms);
    (void)RF_Receiver_DataAvailable();
    uint8_t spi_test = 0xFF;
    RF_Receiver_GetIRQDiagnostics(NULL, NULL, NULL, &spi_test, NULL);
    CHECK(spi_test == 0);
    fake_device_status_fail = 0;

    rf_spi_test_result = 0xFF;
    now_ms += 251; Test_SetTick(now_ms);
    (void)RF_Receiver_DataAvailable();
    RF_Receiver_GetIRQDiagnostics(NULL, NULL, NULL, &spi_test, NULL);
    CHECK(spi_test == 1);
}

TEST(test_nonzero_irq_status_latched)
{
    fake_irq_value = 0x0002;   /* RX_DONE */
    now_ms += 251; Test_SetTick(now_ms);
    (void)RF_Receiver_DataAvailable();
    fake_irq_value = 0;

    uint16_t last_irq = 0;
    uint32_t checks = 0;
    RF_Receiver_GetIRQDiagnostics(&checks, &last_irq, NULL, NULL, NULL);
    CHECK(last_irq == 0x0002);
    CHECK(checks > 0);
}

static void put_i16_le(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

TEST(test_fused_packet_flow)
{
    /* EKF-fused packet: parsed, flagged ready, and delivered like GPS */
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    fake_pkt.data[0] = PACKET_TYPE_FUSED;
    put_i32_le(&fake_pkt.data[1], (int32_t)(39.89 * 10000000.0));
    put_i32_le(&fake_pkt.data[5], (int32_t)(-105.11 * 10000000.0));
    put_i32_le(&fake_pkt.data[9], 123400);          /* alt cm */
    put_i16_le(&fake_pkt.data[13], 500);            /* vN cm/s */
    put_i16_le(&fake_pkt.data[15], -200);           /* vE cm/s */
    put_i16_le(&fake_pkt.data[17], 100);            /* vD cm/s */
    fake_pkt.data[19] = 3;                          /* age ds */
    fake_pkt.data[20] = 0x01;                       /* flags */
    fake_pkt.length = FUSED_PACKET_SIZE;
    fake_pkt.rssi = -90; fake_pkt.snr = 5;
    fake_pkt_pending = 1;

    now_ms += 10; Test_SetTick(now_ms);
    CHECK(RF_Receiver_DataAvailable() == 1);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(RF_Receiver_GetGPSData(&gps) == RF_OK);
    CHECK(gps.is_fused == 1);
}

TEST(test_get_gps_data_without_packet)
{
    RF_Parser_Reset();
    GPS_Data gps;
    CHECK(RF_Receiver_GetGPSData(&gps) == RF_ERROR);
}

TEST(test_data_staleness_tracking)
{
    /* Channel switch discards the previous rocket's freshness */
    CHECK(RF_Receiver_SetChannel(0) == RF_OK);
    CHECK(RF_Receiver_GetLastPacketTime() == 0);
    CHECK(RF_Receiver_IsDataStale(now_ms) == 1);    /* nothing yet */

    inject_gps_packet(39.89, -105.11);
    now_ms += 10; Test_SetTick(now_ms);
    CHECK(RF_Receiver_DataAvailable() == 1);
    GPS_Data gps;
    (void)RF_Receiver_GetGPSData(&gps);

    CHECK(RF_Receiver_IsDataStale(now_ms) == 0);
    CHECK(RF_Receiver_IsDataStale(now_ms + RF_DATA_STALE_TIMEOUT_MS + 1) == 1);
    CHECK(RF_Receiver_GetLastPacketTime() != 0);
}

TEST(test_signal_quality_thresholds)
{
    last_rssi = -125; last_snr = 5;      /* below RSSI floor */
    CHECK(RF_Receiver_IsSignalQualityGood() == 0);

    last_rssi = -90; last_snr = -12;     /* below SNR floor */
    CHECK(RF_Receiver_IsSignalQualityGood() == 0);

    last_rssi = -90; last_snr = 5;
    CHECK(RF_Receiver_IsSignalQualityGood() == 1);

    int16_t rssi = 0; int8_t snr = 0;
    RF_Receiver_GetSignalQuality(&rssi, &snr);
    CHECK(rssi == -90 && snr == 5);
}

TEST(test_cad_result_failure_falls_back_to_dwell)
{
    /* SPI fault while reading the CAD result: CAD can't be trusted */
    fake_cad_available = 1;
    fake_cad_result = LORA_CAD_FAIL;
    RF_Receiver_StartScan();

    now_ms += 10; Test_SetTick(now_ms);
    CHECK(RF_Receiver_ScanUpdate() == 0);   /* IDLE -> CAD started */
    now_ms += 10; Test_SetTick(now_ms);
    CHECK(RF_Receiver_ScanUpdate() == 0);   /* result FAIL -> fallback */

    CHECK(RF_Receiver_IsScanning() == 1);
    CHECK(scan_cad_phase == 0);             /* dwell phase now */
    RF_Receiver_StopScan();
    fake_cad_available = 0;
}

TEST(test_cad_timeout_strikes_then_fallback)
{
    /* CAD never completes (e.g. aborted by a mode change): retry
     * RF_SCAN_CAD_MAX_STRIKES times, then degrade to the dwell scan. */
    fake_cad_available = 1;
    fake_cad_result = LORA_CAD_PENDING;
    RF_Receiver_StartScan();

    for (uint8_t strike = 1; strike <= 3; strike++) {
        now_ms += 10; Test_SetTick(now_ms);
        CHECK(RF_Receiver_ScanUpdate() == 0);          /* start CAD */
        CHECK(scan_cad_state == RF_CAD_RUNNING);
        now_ms += 200; Test_SetTick(now_ms);           /* > 150 ms deadline */
        CHECK(RF_Receiver_ScanUpdate() == 0);          /* timed out */
        if (strike < 3) {
            CHECK(scan_cad_state == RF_CAD_IDLE);      /* retry */
            CHECK(scan_cad_phase == 1);
        }
    }
    CHECK(scan_cad_phase == 0);   /* third strike: dwell fallback */
    CHECK(RF_Receiver_IsScanning() == 1);
    RF_Receiver_StopScan();
    fake_cad_available = 0;
    fake_cad_result = LORA_CAD_NONE;
}

TEST(test_diagnostics_getters)
{
    uint32_t bytes = 0; uint8_t matches = 0; uint8_t last4[4] = {0};
    RF_Receiver_GetDiagnostics(&bytes, &matches, last4);
    CHECK(bytes > 0);                       /* packets flowed in earlier tests */
    RF_Receiver_GetDiagnostics(NULL, NULL, NULL);  /* must not crash */

    uint16_t csum = 0xFFFF;
    RF_Receiver_GetExtendedDiagnostics(&csum);
    CHECK(csum == 0);                       /* dead counter with LoRa CRC */
    RF_Receiver_GetExtendedDiagnostics(NULL);

    char ascii[64];
    RF_Receiver_GetAsciiBuffer(ascii, sizeof(ascii));
    CHECK(ascii[sizeof(ascii) - 1] == '\0');
    RF_Receiver_GetAsciiBuffer(NULL, 0);           /* must not crash */
    (void)RF_Receiver_GetLastPacketASCII(ascii, sizeof(ascii));

    GPS_Data gps; char cs[16]; uint8_t ck;
    (void)RF_Receiver_GetParsedData(&gps, cs, sizeof(cs), &ck);

    CHECK(RF_Receiver_GetLoRaPacketCount() > 0);

    /* Deprecated UART-era shims: fixed answers, no side effects */
    RF_Receiver_SetBaudFudgeFactor(1234);
    CHECK(RF_Receiver_GetBaudFudgeFactor() == RF_BAUD_FUDGE_FACTOR_DEFAULT);
    RF_Receiver_OutputCalibrationSignal(10);

    uint32_t irqs = 1; uint32_t pkts = 0; uint32_t dups = 1;
    RF_Receiver_GetPacketLossDiagnostics(&irqs, &pkts, &dups);
    CHECK(dups == 0);
    CHECK(pkts == RF_Receiver_GetLoRaPacketCount());

    /* DMA IRQ trampolines: just route to the HAL */
    DMA1_Stream3_IRQHandler();
    DMA1_Stream4_IRQHandler();
}

TEST(test_radio_wedge_rx_recovery)
{
    /* A healthy link first: RX confirmed, no recovery churn */
    scan_active = 0;
    rf_wedges_recovered = 0;
    fake_mode = 5;
    uint32_t base_rx = fake_set_rx_calls;
    uint32_t base_init = fake_init_calls;
    run_for(600, 200);
    CHECK(rf_wedges_recovered == 0);

    /* Chip falls out of RX mid-session (EMI, glitched SPI command, 3V3
     * sag): the 4 Hz diag notices within 250 ms, and a sustained 3 s
     * not-RX streak re-enters RX. */
    fake_mode = 2;                               /* STDBY_RC */
    run_for(3400, 200);
    CHECK(fake_set_rx_calls > base_rx);          /* RX re-entered */
    CHECK(rf_wedges_recovered == 1);

    /* Still wedged: three RX retries, then a full chip re-init */
    run_for(10000, 200);
    CHECK(fake_init_calls > base_init);          /* LoRa_Init ran */
    CHECK(rf_wedges_recovered >= 4);

    /* RX restored: the streak resets and recovery goes quiet */
    fake_mode = 5;
    uint32_t wedges_now = rf_wedges_recovered;
    run_for(1000, 200);
    CHECK(rf_wedges_recovered == wedges_now);
    CHECK(RF_Receiver_GetWedgesRecovered() == rf_wedges_recovered);
}

TEST(test_auto_rescan_after_prolonged_silence)
{
    fake_mode = 5;  /* keep the wedge-recovery ladder quiet */

    /* Never heard anything: prolonged silence must NOT kick off a scan
     * (a manual channel pick with a quiet band is a deliberate choice). */
    RF_Receiver_StopScan();
    last_any_packet_ms = 0;
    run_for(400000, 250);
    (void)RF_Receiver_ScanUpdate();
    CHECK(RF_Receiver_IsScanning() == 0);

    /* Beacon heard, then brief silence (battery-save cadence is 60 s):
     * no re-scan. GetGPSData consumes the packet so rf_packet_ready clears
     * and later injections aren't gated. */
    GPS_Data gps;
    inject_gps_packet(39.89, -105.11);
    run_for(250, 250);
    CHECK(RF_Receiver_GetGPSData(&gps) == RF_OK);
    run_for(60000, 250);
    (void)RF_Receiver_ScanUpdate();
    CHECK(RF_Receiver_IsScanning() == 0);

    /* Silence past 5 minutes after contact: the beacon likely rebooted
     * onto its boot channel - re-enter the channel scan. */
    run_for(241000, 250);
    (void)RF_Receiver_ScanUpdate();
    CHECK(RF_Receiver_IsScanning() == 1);

    /* A new packet locks the scan again */
    inject_gps_packet(39.89, -105.11);
    run_for(250, 250);
    CHECK(RF_Receiver_ScanUpdate() == 1);
    CHECK(RF_Receiver_IsScanning() == 0);
}

int main(void)
{
    Test_SetTick(now_ms);

    /* Failure paths first: they re-init repeatedly and must end with a
     * clean, successful init for the rest of the suite. */
    run_test_init_failure_paths();
    CHECK(RF_Receiver_Init() == RF_OK);

    run_test_radio_wedge_rx_recovery();
    run_test_spi_selftest_oneshot_in_poll();
    run_test_nonzero_irq_status_latched();
    run_test_channel_cycling_covers_all_channels();
    run_test_callsign_captured_in_binary_mode();
    run_test_heartbeat_lifecycle();
    run_test_position_packet_flow();
    run_test_fused_packet_flow();
    run_test_get_gps_data_without_packet();
    run_test_data_staleness_tracking();
    run_test_signal_quality_thresholds();
    run_test_channel_scan_locks_on_packet();
    run_test_channel_scan_cad_fast_phase();
    run_test_cad_result_failure_falls_back_to_dwell();
    run_test_cad_timeout_strikes_then_fallback();
    run_test_noise_floor_estimation_and_alert();
    run_test_diagnostics_getters();
    run_test_auto_rescan_after_prolonged_silence();

    return TEST_SUMMARY();
}
