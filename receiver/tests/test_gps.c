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

    /* Receive_IT failure: also GPS_ERROR, with the 0xE1.. debug pattern */
    fake_receive_it_fail = 1;
    CHECK(GPS_Init() == GPS_ERROR);
    uint8_t raw[4];
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xE1 && raw[3] == 0xE4);
    fake_receive_it_fail = 0;

    /* Clean init */
    CHECK(GPS_Init() == GPS_OK);
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xA1);                  /* init marker */
    CHECK(strcmp(local_gps_data.debug_lat, "NO_DATA") == 0);

    GPS_GetRawBytes(NULL);                  /* must not crash */
}

TEST(test_complete_sentence_parsed)
{
    CHECK(GPS_Init() == GPS_OK);

    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_nmea_ready == 1);

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
    CHECK(gps_nmea_ready == 1);

    GPS_Data gps;
    memset(&gps, 0, sizeof(gps));
    CHECK(GPS_Update(&gps) == GPS_ERROR);

    uint8_t raw[4];
    GPS_GetRawBytes(raw);
    CHECK(raw[2] == 0xB5 && raw[3] == 0xB6);   /* parse-failed markers */
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
    CHECK(gps_nmea_index == 0);                /* not buffered */
    huart_gps.ErrorCode = HAL_UART_ERROR_NONE;

    /* Re-arm failure inside the error path sets the 0xF2 marker */
    huart_gps.ErrorCode = 0x04;
    fake_receive_it_fail = 1;
    HAL_UART_RxCpltCallback(&huart_gps);
    uint8_t raw[4];
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xF2);
    fake_receive_it_fail = 0;
    huart_gps.ErrorCode = HAL_UART_ERROR_NONE;
}

TEST(test_noise_before_start_char_ignored)
{
    CHECK(GPS_Init() == GPS_OK);

    /* Garbage with no leading '$' must never assemble a sentence */
    feed_string("garbage noise\r\n");
    CHECK(gps_nmea_ready == 0);

    /* A '$' mid-noise starts a fresh sentence cleanly */
    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_nmea_ready == 1);
    gps_nmea_ready = 0;
    gps_nmea_index = 0;
}

TEST(test_new_dollar_restarts_sentence)
{
    /* A '$' arriving mid-sentence (lost terminator) restarts assembly
     * instead of corrupting the buffer */
    feed_string("$GPGGA,123");
    CHECK(gps_nmea_index == 10);
    feed_sentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
    CHECK(gps_nmea_ready == 1);
    CHECK(strncmp((char *)gps_nmea_buffer, "$GPRMC", 6) == 0);

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
    CHECK(gps_nmea_ready == 0);
    CHECK(gps_nmea_index == 0);                /* overflow reset */

    feed_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(gps_nmea_ready == 1);
    gps_nmea_ready = 0;
    gps_nmea_index = 0;
}

TEST(test_lf_without_cr_not_a_sentence)
{
    /* "\n" terminator without the preceding "\r": incomplete framing */
    feed_string("$GPGGA,123\n");
    CHECK(gps_nmea_ready == 0);
    gps_nmea_index = 0;
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
    gps_nmea_index = 0;
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

    /* Re-arm failure path sets the error pattern */
    huart_gps.RxState = HAL_UART_STATE_READY;
    fake_receive_it_fail = 1;
    (void)GPS_Update(&gps);
    uint8_t raw[4];
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xE1);
    fake_receive_it_fail = 0;

    /* Busy RX: no re-arm */
    huart_gps.RxState = 0x22;
    calls_before = receive_it_calls;
    (void)GPS_Update(&gps);
    CHECK(receive_it_calls == calls_before);
}

TEST(test_isr_plumbing)
{
    uint32_t before = irq_handler_calls;
    USART2_IRQHandler();
    CHECK(irq_handler_calls == before + 1);

    uint8_t raw[4];
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xAA);                     /* ISR-entry marker */

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

    /* Receive_IT failure INSIDE the byte callback marks last_bytes[0]=0xF2
     * (the 0xE1 path in GPS_Update is covered by test_rearm_when_uart_ready) */
    fake_receive_it_fail = 1;
    feed_byte('$');
    fake_receive_it_fail = 0;
    GPS_GetRawBytes(raw);
    CHECK(raw[0] == 0xF2);
}

TEST(test_first_and_raw_capture_buffers_fill)
{
    /* Feed enough bytes to fill the first-10 and raw-capture buffers */
    CHECK(GPS_Init() == GPS_OK);
    uart_byte_counter = 0;
    first_10_bytes_filled = 0;
    raw_capture_filled = 0;
    raw_capture_index = 0;

    for (int i = 0; i < RAW_CAPTURE_SIZE + 5; i++) {
        feed_byte((uint8_t)('0' + (i % 10)));
    }
    CHECK(first_10_bytes_filled == 1);
    CHECK(raw_capture_filled == 1);
    CHECK(first_10_bytes[0] == '0');
    CHECK(first_10_bytes[9] == '9');
    gps_nmea_index = 0;
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_test_init_success_and_failure_paths();
    run_test_complete_sentence_parsed();
    run_test_unparseable_sentence_reports_error();
    run_test_framing_error_byte_dropped();
    run_test_noise_before_start_char_ignored();
    run_test_new_dollar_restarts_sentence();
    run_test_buffer_overflow_resets_index();
    run_test_lf_without_cr_not_a_sentence();
    run_test_debug_rotation_covers_all_modes();
    run_test_rearm_when_uart_ready();
    run_test_isr_plumbing();
    run_test_first_and_raw_capture_buffers_fill();

    return TEST_SUMMARY();
}
