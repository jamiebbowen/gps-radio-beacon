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
static uint8_t       fake_mode      = 5;     /* 5 = RX (status bits [7:5]) */
static int16_t       fake_rssi_inst = -120;  /* Ambient level "measured"   */
static LoRa_Packet_t fake_pkt;
static uint8_t       fake_pkt_pending = 0;

uint8_t LoRa_Init(SPI_HandleTypeDef *hspi) { (void)hspi; return LORA_OK; }
uint8_t LoRa_SetReceiveMode(void) { return LORA_OK; }
uint8_t LoRa_GetDeviceStatus(uint8_t *status)
{
    *status = (uint8_t)((fake_mode << 5) | 0x02);
    return LORA_OK;
}
uint8_t LoRa_GetIRQStatus(uint16_t *irq) { *irq = 0; return LORA_OK; }
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
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h) { (void)h; return HAL_OK; }
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
    };
    memset(&fake_pkt, 0, sizeof(fake_pkt));
    memcpy(fake_pkt.data, &hb, sizeof(hb));
    fake_pkt.length = HEARTBEAT_PACKET_SIZE;
    fake_pkt.rssi = -87;
    fake_pkt.snr = 6;
    fake_pkt_pending = 1;
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

int main(void)
{
    Test_SetTick(now_ms);
    CHECK(RF_Receiver_Init() == RF_OK);

    run_test_channel_cycling_covers_all_channels();
    run_test_heartbeat_lifecycle();
    run_test_position_packet_flow();
    run_test_channel_scan_locks_on_packet();
    run_test_noise_floor_estimation_and_alert();

    return TEST_SUMMARY();
}
