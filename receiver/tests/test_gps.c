/**
 * @file test_gps.c
 * @brief Host-side unit tests for the local (receiver-side) GPS driver.
 *
 * Fakes the UART HAL and feeds NMEA bytes through the real ISR callback
 * path (HAL_UART_RxCpltCallback), exercising sentence assembly, framing
 * error rejection, buffer-overflow recovery, the IRQ re-arm logic, and
 * the debug-rotation display in GPS_Update. Parsing is done by the real
 * gps_parser.c (linked in).
 *
 * Build & run:  make -C receiver/tests          (see tests/Makefile)
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gps.h"
#include "test_harness.h"

/* Include the module under test directly (not linked - see tests/Makefile) */
#include "../firmware/src/gps.c"

/* gps.c references this global (owned by main.c on target) */
GPS_Data local_gps_data;

/* ------------------------------------------------------------------ */
/* Fake UART HAL                                                       */
/* ------------------------------------------------------------------ */

USART_TypeDef Test_USART2;   /* referenced by the USART2 stub macro */

static uint8_t  fake_uart_init_fail    = 0;
static uint8_t  fake_receive_it_fail   = 0;
static uint32_t receive_it_calls       = 0;
static uint32_t irq_handler_calls      = 0;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t p, uint32_t s)
{
    (void)irq; (void)p; (void)s;
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq)  { (void)irq; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; }

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
    if (fake_uart_init_fail) return HAL_ERROR;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                      uint8_t *data, uint16_t size)
{
    (void)data; (void)size;
    receive_it_calls++;
    if (fake_receive_it_fail) return HAL_ERROR;
    huart->RxState = 0x22; /* HAL_UART_STATE_BUSY_RX */
    return HAL_OK;
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    (void)huart;
    irq_handler_calls++;
}

/* TX capture for the boot-time UBX-CFG-MSG module config */
static uint8_t tx_cap[512];
static size_t  tx_cap_len = 0;
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data, uint16_t size,
                                    uint32_t timeout)
{
    (void)huart; (void)timeout;
    for (uint16_t i = 0; i < size && tx_cap_len < sizeof(tx_cap); i++) {
        tx_cap[tx_cap_len++] = data[i];
    }
    return HAL_OK;
}
static int tx_cap_contains_msg(uint8_t msg_id, uint8_t rate)
{
    /* frame: B5 62 06 01 08 00 F0 <id> 00 <rate> ... */
    for (size_t i = 0; i + 10 <= tx_cap_len; i++) {
        if (tx_cap[i]   == 0xB5 && tx_cap[i+1] == 0x62 &&
            tx_cap[i+2] == 0x06 && tx_cap[i+3] == 0x01 &&
            tx_cap[i+6] == 0xF0 && tx_cap[i+7] == msg_id &&
            tx_cap[i+9] == rate) return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Byte-feeding helpers (simulate the RX-complete interrupt)           */
/* ------------------------------------------------------------------ */

static void feed_byte(uint8_t b)
{
    gps_rx_data = b;                        /* what the DMA/IRQ would store */
    huart_gps.RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(&huart_gps);
}

static void feed_string(const char *s)
{
    while (*s) feed_byte((uint8_t)*s++);
}

/** Append correct "*HH\r\n" NMEA checksum+terminator to a "$..." body. */
static void feed_sentence(const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body + 1; *p; p++) cs ^= (uint8_t)*p;
    char full[128];
    snprintf(full, sizeof(full), "%s*%02X\r\n", body, cs);
    feed_string(full);
}

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_init_success_and_failure_paths)
{
    /* UART init failure: report GPS_ERROR (main degrades gracefully),
     * never reset the receiver over an optional peripheral */
    fake_uart_init_fail = 1;
    CHECK(GPS_Init() == GPS_ERROR);
    fake_uart_init_fail = 0;

    /* Receive_IT failure: also GPS_ERROR, counted as a failed re-arm */
    uint32_t ra0 = uart_rearm_failures;
    fake_receive_it_fail = 1;
    CHECK(GPS_Init() == GPS_ERROR);
    CHECK(uart_rearm_failures == ra0 + 1);
    fake_receive_it_fail = 0;

    /* Clean init */
    CHECK(GPS_Init() == GPS_OK);
    CHECK(uart_rearm_failures == ra0 + 1);
}

TEST(test_rejected_fix_does_not_latch_isfixed)
{
    /* A teleporting fix is rejected by the parser; it must not leave
     * GPS_IsFixed() reporting a fix main.c would then trust. */
    CHECK(GPS_Init() == GPS_OK);
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_Update(&gps) == GPS_OK);
    CHECK(GPS_IsFixed() == 1);

    feed_sentence("$GPGGA,123520,5807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    (void)GPS_Update(&gps);
    CHECK(GPS_IsFixed() == 0);                 /* rejected: no fix */
    CHECK(gps.latitude > 48.0f && gps.latitude < 48.2f);   /* kept */

    /* Back at the anchor: the fix returns */
    feed_sentence("$GPGGA,123521,4807.040,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(GPS_Update(&gps) == GPS_OK);
    CHECK(GPS_IsFixed() == 1);
}

TEST(test_complete_sentence_parsed)
{
    CHECK(GPS_Init() == GPS_OK);

    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_pend_head != gps_pend_tail);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(GPS_Update(&gps) == GPS_OK);
    CHECK(gps.fix == 1);
    CHECK(gps.satellites == 8);
    CHECK(gps.latitude > 48.0f && gps.latitude < 48.2f);

    /* Sentence consumed: next update has nothing to parse */
    CHECK(GPS_Update(&gps) == GPS_ERROR);
}

TEST(test_unparseable_sentence_reports_error)
{
    /* Complete framing but bad checksum: assembled, then parser rejects */
    feed_string("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,,*00\r\n");
    CHECK(gps_pend_head != gps_pend_tail);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(GPS_Update(&gps) == GPS_ERROR);
}

TEST(test_framing_error_byte_dropped)
{
    CHECK(GPS_Init() == GPS_OK);
    uint32_t bytes_before = uart_byte_counter;

    /* Byte arrives with a UART error flag: must be rejected and the
     * receive re-armed, not fed into the sentence buffer */
    gps_rx_data = '$';
    huart_gps.ErrorCode = 0x04;  /* framing error */
    huart_gps.RxState = HAL_UART_STATE_READY;
    HAL_UART_RxCpltCallback(&huart_gps);

    CHECK(uart_byte_counter == bytes_before);  /* not counted */
    CHECK(gps_isr_index == 0);                 /* not buffered */
    huart_gps.ErrorCode = HAL_UART_ERROR_NONE;

    /* Re-arm failure inside the error path is counted */
    huart_gps.ErrorCode = 0x04;
    fake_receive_it_fail = 1;
    uint32_t ra0 = uart_rearm_failures;
    HAL_UART_RxCpltCallback(&huart_gps);
    CHECK(uart_rearm_failures == ra0 + 1);
    fake_receive_it_fail = 0;
    huart_gps.ErrorCode = HAL_UART_ERROR_NONE;
}

TEST(test_noise_before_start_char_ignored)
{
    CHECK(GPS_Init() == GPS_OK);

    /* Garbage with no leading '$' must never assemble a sentence */
    feed_string("garbage noise\r\n");
    CHECK(gps_pend_head == gps_pend_tail);

    /* A '$' mid-noise starts a fresh sentence cleanly */
    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_pend_head != gps_pend_tail);
    gps_pend_tail = gps_pend_head;             /* drain without parsing */
    gps_isr_index = 0;
}

TEST(test_new_dollar_restarts_sentence)
{
    /* A '$' arriving mid-sentence (lost terminator) restarts assembly
     * instead of corrupting the buffer */
    feed_string("$GPGGA,123");
    CHECK(gps_isr_index == 10);
    feed_sentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    CHECK(gps_pend_head != gps_pend_tail);
    CHECK(strncmp((char *)gps_pending[gps_pend_tail], "$GPRMC", 6) == 0);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(GPS_Update(&gps) == GPS_OK);         /* RMC parsed fine */
}

TEST(test_buffer_overflow_resets_index)
{
    CHECK(GPS_Init() == GPS_OK);

    /* A run-on "sentence" longer than the buffer must reset, then a
     * normal sentence must still get through */
    feed_byte('$');
    for (int i = 0; i < GPS_BUFFER_SIZE + 10; i++) feed_byte('X');
    CHECK(gps_pend_head == gps_pend_tail);
    CHECK(gps_isr_index == 0);                 /* overflow reset */

    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_pend_head != gps_pend_tail);
    gps_pend_tail = gps_pend_head;
    gps_isr_index = 0;
}

TEST(test_lf_without_cr_not_a_sentence)
{
    /* "\n" terminator without the preceding "\r": incomplete framing */
    feed_string("$GPGGA,123\n");
    CHECK(gps_pend_head == gps_pend_tail);
    gps_isr_index = 0;
}

TEST(test_debug_rotation_covers_all_modes)
{
    CHECK(GPS_Init() == GPS_OK);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));

    /* 6 debug modes x 5 calls each: walk the whole rotation (including
     * the raw-hex offset cycling and the empty-buffer branch) */
    for (int i = 0; i < 65; i++) {
        (void)GPS_Update(&gps);
    }
    CHECK(1);                                  /* no crash = pass */

    /* NMEA-buffer debug mode with data present (index > 0) */
    feed_string("$GPG");                       /* partial sentence */
    for (int i = 0; i < 30; i++) {
        (void)GPS_Update(&gps);
    }
    gps_isr_index = 0;
}

TEST(test_rearm_when_uart_ready)
{
    CHECK(GPS_Init() == GPS_OK);

    /* RxState READY: GPS_Update must re-arm reception */
    huart_gps.RxState = HAL_UART_STATE_READY;
    uint32_t calls_before = receive_it_calls;
    GPS_Data gps;
    (void)GPS_Update(&gps);
    CHECK(receive_it_calls == calls_before + 1);

    /* Re-arm failure is counted */
    huart_gps.RxState = HAL_UART_STATE_READY;
    fake_receive_it_fail = 1;
    uint32_t ra0 = uart_rearm_failures;
    (void)GPS_Update(&gps);
    CHECK(uart_rearm_failures == ra0 + 1);
    fake_receive_it_fail = 0;

    /* Busy RX: no re-arm */
    huart_gps.RxState = 0x22;
    calls_before = receive_it_calls;
    (void)GPS_Update(&gps);
    CHECK(receive_it_calls == calls_before);
}

TEST(test_error_callback_rearms_after_ore_teardown)
{
    CHECK(GPS_Init() == GPS_OK);

    /* The HAL IRQHandler's ORE path calls UART_EndRxTransfer AFTER
     * RxCpltCallback: the re-arm done there is torn back down and
     * RxState lands on READY. HAL_UART_ErrorCallback must re-arm inside
     * the same interrupt context so bytes aren't dropped until the next
     * GPS_Update poll. */
    huart_gps.RxState = HAL_UART_STATE_READY;
    huart_gps.ErrorCode = 0x08;              /* HAL_UART_ERROR_ORE */
    uint32_t calls_before = receive_it_calls;
    uint32_t errs_before = uart_error_count;
    HAL_UART_ErrorCallback(&huart_gps);
    CHECK(receive_it_calls == calls_before + 1);
    CHECK(uart_error_count == errs_before + 1);
    CHECK(huart_gps.RxState == 0x22);        /* stub marks BUSY_RX */
    CHECK(huart_gps.ErrorCode == HAL_UART_ERROR_NONE);

    /* A failed re-arm in the error callback is counted too */
    huart_gps.RxState = HAL_UART_STATE_READY;
    huart_gps.ErrorCode = 0x08;
    fake_receive_it_fail = 1;
    uint32_t ra0 = uart_rearm_failures;
    HAL_UART_ErrorCallback(&huart_gps);
    CHECK(uart_rearm_failures == ra0 + 1);
    fake_receive_it_fail = 0;

    /* Non-ORE path: the armed transfer survives (RxState stays BUSY_RX),
     * so the callback must NOT disturb it */
    huart_gps.RxState = 0x22;
    calls_before = receive_it_calls;
    HAL_UART_ErrorCallback(&huart_gps);
    CHECK(receive_it_calls == calls_before);

    /* A different UART instance must be ignored entirely */
    UART_HandleTypeDef other;
    memset(&other, 0, sizeof(other));
    USART_TypeDef other_usart;
    other.Instance = &other_usart;
    other.RxState = HAL_UART_STATE_READY;
    calls_before = receive_it_calls;
    HAL_UART_ErrorCallback(&other);
    CHECK(receive_it_calls == calls_before);
}

TEST(test_isr_plumbing)
{
    uint32_t before = irq_handler_calls;
    USART2_IRQHandler();
    CHECK(irq_handler_calls == before + 1);

    /* Callback for a different UART instance: ignored */
    UART_HandleTypeDef other;
    memset(&other, 0, sizeof(other));
    USART_TypeDef other_usart;
    other.Instance = &other_usart;
    uint32_t bytes_before = uart_byte_counter;
    HAL_UART_RxCpltCallback(&other);
    CHECK(uart_byte_counter == bytes_before);

    /* Fix latch tracks the parser: still 1 from the valid RMC fed earlier */
    GPS_Data fix_scratch = {0};
    CHECK(GPS_IsFixed() == 1);

    /* A void (V) RMC clears it even though the sentence parses as no-fix */
    feed_sentence("$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    (void)GPS_Update(&fix_scratch);
    CHECK(GPS_IsFixed() == 0);

    /* And a valid fix re-latches it */
    feed_sentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    (void)GPS_Update(&fix_scratch);
    CHECK(GPS_IsFixed() == 1);

    /* Receive_IT failure INSIDE the byte callback is counted (the GPS_Update
     * path is covered by test_rearm_when_uart_ready) */
    uint32_t ra0 = uart_rearm_failures;
    fake_receive_it_fail = 1;
    feed_byte('$');
    fake_receive_it_fail = 0;
    CHECK(uart_rearm_failures == ra0 + 1);
}

TEST(test_init_sends_ubx_msg_config)
{
    /* Boot-time message shaping: six CFG-MSG set-rate frames (legacy UBX,
     * valid on NEO-6M-class modules) - GSV/GLL/VTG/GSA off, RMC/GGA on. */
    tx_cap_len = 0;
    CHECK(GPS_Init() == GPS_OK);
    CHECK(tx_cap_contains_msg(0x03, 0));       /* GSV off  */
    CHECK(tx_cap_contains_msg(0x01, 0));       /* GLL off  */
    CHECK(tx_cap_contains_msg(0x05, 0));       /* VTG off  */
    CHECK(tx_cap_contains_msg(0x02, 0));       /* GSA off  */
    CHECK(tx_cap_contains_msg(0x04, 1));       /* RMC on   */
    CHECK(tx_cap_contains_msg(0x00, 1));       /* GGA on   */
}

TEST(test_fix_latch_expires_when_stream_dies)
{
    /* Field-failure shape: antenna unplugged / module wedge mid-session.
     * Without an age gate the latch returned 1 forever and main.c kept
     * re-copying frozen coordinates as a live "L:Fix" until reboot. */
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    Test_SetTick(100000);
    feed_sentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    (void)GPS_Update(&gps);
    CHECK(GPS_IsFixed() == 1);

    /* Stream goes silent: beyond the stale window the fix expires */
    Test_SetTick(100000 + GPS_FIX_STALE_MS + 1);
    CHECK(GPS_IsFixed() == 0);

    /* Sentences resume: the latch re-arm itself */
    feed_sentence("$GPRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    (void)GPS_Update(&gps);
    CHECK(GPS_IsFixed() == 1);
    Test_SetTick(0);        /* leave the shared tick where others expect it */
}

TEST(test_pending_fifo_survives_stall_and_drops_oldest_none)
{
    /* Two sentences completing inside one main-loop stall (display frame
     * write): BOTH must survive in FIFO order. */
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    Test_SetTick(0);
    uint32_t drops_before = gps_sentences_dropped;
    feed_sentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,09,0.9,545.4,M,46.9,M,,");
    (void)GPS_Update(&gps);                    /* drains both in one call */
    CHECK(gps_sentences_dropped == drops_before);
    CHECK(gps.satellites == 9);                /* GGA (second) won */
    CHECK(gps.fix == 1);
    CHECK(gps_pend_head == gps_pend_tail);     /* queue empty */

    /* Three completions in one stall: queue depth is 2, the newest is
     * dropped and counted (position freshness beats completeness). */
    feed_sentence("$GPRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    feed_sentence("$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    feed_sentence("$GPRMC,123521,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    CHECK(gps_sentences_dropped == drops_before + 1);
    memset(&gps, 0, sizeof(gps));
    (void)GPS_Update(&gps);
    CHECK(gps.second == 20);                   /* third RMC (…521) was dropped */
    CHECK(gps.satellites == 8);                /* but the GGA survived */
}

TEST(test_rx_stats_snapshot)
{
    /* Aggregate the counters the diagnostics screen reports: wire bytes,
     * completed sentences, dropped-completions and UART errors. */
    uart_byte_counter = 0;
    nmea_sentence_counter = 0;
    gps_sentences_dropped = 0;
    uint32_t err_before = uart_error_count;

    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    feed_string("garbage noise\r\n");          /* completes, unparseable */
    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    (void)GPS_Update(&gps);

    uint32_t bytes = 0, sentences = 0, dropped = 0, rearm = 99;
    uint16_t uart_errs = 0;
    GPS_GetRxStats(&bytes, &sentences, &dropped, &uart_errs, &rearm);
    CHECK(rearm == uart_rearm_failures);
    CHECK(bytes == uart_byte_counter && bytes > 0);
    CHECK(sentences == nmea_sentence_counter);
    CHECK(dropped == gps_sentences_dropped);
    CHECK(uart_errs == uart_error_count);
    CHECK(uart_errs >= err_before);

    /* NULL-safe */
    GPS_GetRxStats(NULL, NULL, NULL, NULL, NULL);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_success_and_failure_paths();
    run_test_init_sends_ubx_msg_config();
    run_test_complete_sentence_parsed();
    run_test_rejected_fix_does_not_latch_isfixed();
    run_test_pending_fifo_survives_stall_and_drops_oldest_none();
    run_test_unparseable_sentence_reports_error();
    run_test_framing_error_byte_dropped();
    run_test_noise_before_start_char_ignored();
    run_test_new_dollar_restarts_sentence();
    run_test_buffer_overflow_resets_index();
    run_test_lf_without_cr_not_a_sentence();
    run_test_debug_rotation_covers_all_modes();
    run_test_rearm_when_uart_ready();
    run_test_error_callback_rearms_after_ore_teardown();
    run_test_isr_plumbing();
    run_test_fix_latch_expires_when_stream_dies();
    run_test_rx_stats_snapshot();

    return TEST_SUMMARY();
}
