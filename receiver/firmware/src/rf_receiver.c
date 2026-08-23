/**
 * @file rf_receiver.c
 * @brief RF receiver driver implementation for E22-400M33S LoRa module
 */

#include "rf_receiver.h"
#include "rf_parser.h"  /* For RF packet parsing functions */
#include "lora.h"       /* LoRa driver */
#include "packet_format.h"
#include <string.h>  /* For memcpy and strstr functions */
#include <stdlib.h>  /* For atof and atoi functions */
#include <ctype.h>   /* For isdigit function */
#include <stdio.h>   /* For snprintf function */

/* Private defines */
#define RF_ASCII_BUFFER_SIZE       128

/* Channel-scan dwell time. Must exceed the transmitter's slowest pad-state
 * packet interval (PRE_LAUNCH_INTERVAL_SEC = 5 s) so a beacon with GPS lock
 * cannot slip between hops. A beacon with NO fix only sends its callsign
 * every 5 min, so the scan may cycle for a while - that is expected; it
 * locks on the first CRC-valid packet from any channel. */
#define RF_SCAN_DWELL_MS           6500U

/* CAD fast-scan phase: sniff each channel for a LoRa preamble (~20 ms per
 * channel, <0.3 s per lap over all 8) instead of dwelling 6.5 s. A CAD only
 * fires while a preamble is actually on the air, so laps repeat for
 * RF_SCAN_CAD_PHASE_MS; each transmission is another chance to be caught
 * mid-preamble. If nothing is caught (beacon idle, weak signal, or CAD
 * config rejected by the chip), the scan falls back to the dwell phase,
 * which guarantees a lock within one 52 s lap of a transmitting beacon. */
#define RF_SCAN_CAD_PHASE_MS       30000U  /* fast phase length (~6 heartbeats) */
#define RF_SCAN_CAD_TIMEOUT_MS     150U    /* CAD itself must finish in ~20 ms */
#define RF_SCAN_CAD_RX_WAIT_MS     800U    /* detection -> packet grace period */
#define RF_SCAN_CAD_MAX_STRIKES    3U      /* timed-out CADs before fallback */

/* CAD scan sub-states */
#define RF_CAD_IDLE     0   /* next update starts a CAD */
#define RF_CAD_RUNNING  1   /* CAD in flight, polling for the result */
#define RF_CAD_RX_WAIT  2   /* preamble detected, chip receiving the packet */

/* Private variables */
static SPI_HandleTypeDef hspi_lora;
/* DMA handles are attached to hspi_lora so lora.c can issue HAL_SPI_Receive_DMA
 * for large buffer reads. TX DMA is reserved but currently unused; kept so that
 * future TX paths don't need to re-plumb __HAL_LINKDMA. */
static DMA_HandleTypeDef hdma_spi2_tx;
static DMA_HandleTypeDef hdma_spi2_rx;
static uint32_t rf_bytes_received = 0;
static uint32_t rf_lora_packets_received = 0;  // Total LoRa packets received
static uint32_t rf_irq_checks = 0;  // How many times we've checked IRQ status
static uint16_t rf_last_irq_status = 0;  // Last IRQ status we read
static uint8_t rf_last_device_mode = 0;  // Last device operating mode
static uint8_t rf_spi_test_result = 0xFF;  // SPI communication test (0=fail, 1=pass, 0xFF=not tested)
static uint8_t rf_last_busy_state = 0;  // Last BUSY pin state
static uint8_t rf_header_matches = 0;
static volatile uint8_t rf_packet_ready = 0;
static uint8_t rf_last_bytes[4] = {0};
static char rf_ascii_buffer[RF_ASCII_BUFFER_SIZE];
static LoRa_Packet_t last_packet;
static int16_t last_rssi = 0;
static int8_t last_snr = 0;
static uint32_t last_packet_time = 0;  /* Timestamp of last valid packet */

/* Channel-scan state */
static uint8_t scan_active = 0;
static uint32_t scan_dwell_start = 0;
static uint32_t scan_dwell_pkt_count = 0;
static uint8_t scan_cad_phase = 0;       /* 1 = CAD fast phase, 0 = dwell */
static uint32_t scan_cad_phase_start = 0;
static uint8_t scan_cad_state = RF_CAD_IDLE;
static uint32_t scan_cad_deadline = 0;   /* per-state timeout */
static uint8_t scan_cad_strikes = 0;     /* consecutive timed-out CADs */

/* Last heartbeat received (no-fix keepalive from the beacon) */
static HeartbeatPacket_t last_heartbeat;
static uint8_t heartbeat_pending = 0;
static uint32_t last_heartbeat_time = 0;   /* 0 = never heard one */

/* Ambient noise-floor monitor.
 * GetRssiInst is sampled ~1 Hz while the radio sits in continuous RX. The
 * floor estimate is the 25th percentile of a sliding window: samples taken
 * mid-packet (or during a scan retune) land in the upper percentiles, so
 * they can't inflate the estimate, while a genuinely raised floor lifts
 * the whole window and shows through. */
#define RF_NOISE_WINDOW        16                        /* samples (~16 s) */
#define RF_NOISE_PCTL_IDX      (RF_NOISE_WINDOW / 4)     /* 25th percentile */
#define RF_NOISE_MIN_SAMPLES   (RF_NOISE_WINDOW / 2)     /* before valid */
static int16_t  noise_samples[RF_NOISE_WINDOW];
static uint8_t  noise_sample_idx = 0;
static uint8_t  noise_sample_count = 0;
static uint8_t  noise_alert_active = 0;

/* Private function prototypes */
static uint8_t RF_SPI_Init(void);

/**
 * @brief Initialize RF receiver
 * @retval Status code
 */
uint8_t RF_Receiver_Init(void)
{
  /* Initialize SPI for LoRa communication. A failure here used to hang in
   * while(1) with a blank screen - indistinguishable from a dead battery.
   * Report it instead: main shows the code and retries every 10 s. */
  uint8_t spi_result = RF_SPI_Init();
  if (spi_result != RF_OK) {
    return spi_result;
  }
  
  /* Initialize LoRa module */
  uint8_t lora_result = LoRa_Init(&hspi_lora);
  if (lora_result != LORA_OK) {
    return lora_result;  // Pass through the specific error code
  }
  
  /* Initialize buffers and flags */
  memset(rf_ascii_buffer, 0, RF_ASCII_BUFFER_SIZE);
  rf_packet_ready = 0;
  
  /* Initialize the parser */
  RF_Parser_Init();
  
  /* LoRa_Init already entered RX mode, no need to call again */
  
  /* Run SPI test after init */
  HAL_Delay(100);
  uint8_t test_status = 0;
  if (LoRa_GetDeviceStatus(&test_status) == LORA_OK && test_status != 0x00) {
    rf_spi_test_result = 1;  // Pass
    rf_last_device_mode = (test_status >> 5) & 0x07;  // Extract mode
  } else {
    rf_spi_test_result = 0;  // Fail
  }
  
  /* Check BUSY pin */
  rf_last_busy_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
  
  return RF_OK;
}

/**
 * @brief Ensure receiver enters RX mode (called once after init)
 */
static void RF_EnsureRxMode(void) {
    static uint8_t rx_entered = 0;
    
    /* Enter RX mode once on first call */
    if (!rx_entered) {
        LoRa_SetReceiveMode();
        rx_entered = 1;
    }
}

/**
 * @brief Check if RF data is available
 * @retval 1 if data available, 0 otherwise
 */
uint8_t RF_Receiver_DataAvailable(void)
{
  /* Ensure we stay in RX mode */
  RF_EnsureRxMode();
  
  /* Track IRQ checks for diagnostics */
  rf_irq_checks++;
  
  /* One-time SPI test - try to get device status on first call */
  if (rf_spi_test_result == 0xFF) {
    uint8_t test_status = 0;
    if (LoRa_GetDeviceStatus(&test_status) == LORA_OK && test_status != 0x00) {
      rf_spi_test_result = 1;  // Pass
    } else {
      rf_spi_test_result = 0;  // Fail
    }
  }
  
  /* Diagnostics-only reads, throttled to 4 Hz: each is a blocking SPI
   * transaction with a BUSY wait, and this function runs up to 3x per
   * main-loop iteration. Doing them every call added dead time to the loop
   * (hurting button/display latency) for data only shown on debug screens. */
  static uint32_t last_diag_ms = 0;
  uint32_t now_ms = HAL_GetTick();
  if (now_ms - last_diag_ms >= 250) {
    last_diag_ms = now_ms;
    
    /* Get IRQ status for diagnostics */
    uint16_t irq_status = 0;
    LoRa_GetIRQStatus(&irq_status);
    if (irq_status != 0) {
      rf_last_irq_status = irq_status;  // Store non-zero status
    }
    
    /* Check BUSY pin state */
    rf_last_busy_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
    
    /* Get device status to verify mode */
    uint8_t device_status = 0;
    if (LoRa_GetDeviceStatus(&device_status) == LORA_OK) {
      rf_last_device_mode = (device_status >> 5) & 0x07;  // Extract mode from bits [7:5]
    }
  }
  
  /* Ambient noise-floor sampling, 1 Hz. Only sample while the chip is
   * confirmed in RX mode (mode 5) - GetRssiInst is undefined in standby,
   * which the radio briefly enters during scan/manual retunes. */
  static uint32_t last_noise_ms = 0;
  if (now_ms - last_noise_ms >= 1000) {
    last_noise_ms = now_ms;
    int16_t inst_rssi;
    if (rf_last_device_mode == 5 && LoRa_GetRssiInst(&inst_rssi) == LORA_OK) {
      noise_samples[noise_sample_idx] = inst_rssi;
      noise_sample_idx = (uint8_t)((noise_sample_idx + 1) % RF_NOISE_WINDOW);
      if (noise_sample_count < RF_NOISE_WINDOW) noise_sample_count++;
      
      /* Update the alert with hysteresis so it doesn't flap at the edge */
      int16_t nf;
      if (RF_Receiver_GetNoiseFloor(&nf)) {
        if (!noise_alert_active && nf >= RF_NOISE_ALERT_DBM) {
          noise_alert_active = 1;
        } else if (noise_alert_active && nf <= RF_NOISE_CLEAR_DBM) {
          noise_alert_active = 0;
        }
      }
    }
  }
  
  /* Check for new LoRa packets */
  if (!rf_packet_ready && LoRa_PacketAvailable()) {
    /* Read the packet */
    if (LoRa_ReadPacket(&last_packet) == LORA_OK) {
      rf_lora_packets_received++;  // Count every LoRa packet received
      rf_bytes_received += last_packet.length;
      last_rssi = last_packet.rssi;
      last_snr = last_packet.snr;
      
      /* Update last-bytes diagnostics with the trailing bytes of the packet.
       * Guard against packets shorter than 4 bytes: indexing
       * data[length - 4 + i] with length < 4 read out of bounds. */
      {
        uint16_t tail = (last_packet.length >= 4) ? 4 : last_packet.length;
        uint16_t start = last_packet.length - tail;
        for (uint16_t i = 0; i < tail; i++) {
          rf_last_bytes[i] = last_packet.data[start + i];
        }
      }
      
      /* Parse packet based on format */
#if USE_BINARY_PACKETS
      /* Binary packet format - dispatch by type + length */
      if (last_packet.length == 13 && last_packet.data[0] == PACKET_TYPE_GPS) {
        /* Raw GPS fix */
        if (RF_Parser_ParseBinaryPacket(last_packet.data, last_packet.length) == RF_PARSER_OK) {
          rf_packet_ready = 1;
          rf_header_matches++;
          last_packet_time = HAL_GetTick();
        }
      } else if (last_packet.length == FUSED_PACKET_SIZE
              && last_packet.data[0] == PACKET_TYPE_FUSED) {
        /* EKF-fused position + velocity from TX nav layer */
        if (RF_Parser_ParseFusedPacket(last_packet.data, last_packet.length) == RF_PARSER_OK) {
          rf_packet_ready = 1;
          rf_header_matches++;
          last_packet_time = HAL_GetTick();
        }
      } else if ((last_packet.length == HEARTBEAT_PACKET_SIZE
               || last_packet.length == HEARTBEAT_PACKET_SIZE_V1)
              && last_packet.data[0] == PACKET_TYPE_HEARTBEAT) {
        /* No-fix keepalive. Deliberately does NOT set rf_packet_ready or
         * last_packet_time: a heartbeat proves the link is alive, not that
         * position data is fresh. It still counts toward
         * rf_lora_packets_received, which is what locks the channel scan.
         * V1 (7-byte) heartbeats from older beacons lack the trailing
         * gps_health byte; zero-fill decodes it as HB_GPS_UNKNOWN. */
        memset(&last_heartbeat, 0, sizeof(last_heartbeat));
        memcpy(&last_heartbeat, last_packet.data, last_packet.length);
        heartbeat_pending = 1;
        last_heartbeat_time = HAL_GetTick();
      }
#else
      /* ASCII packet format */
      if (last_packet.length < RF_ASCII_BUFFER_SIZE) {
        memcpy(rf_ascii_buffer, last_packet.data, last_packet.length);
        rf_ascii_buffer[last_packet.length] = '\0';
        
        /* Parse ASCII packet */
        if (RF_Parser_ParseAsciiPacket(rf_ascii_buffer) == RF_PARSER_OK) {
          rf_packet_ready = 1;
          rf_header_matches++;
          last_packet_time = HAL_GetTick();
        }
      }
#endif
    }
  }
  
  return rf_packet_ready;
}

/**
 * @brief Get GPS data from RF packet
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
uint8_t RF_Receiver_GetGPSData(GPS_Data *gps_data)
{
  /* Use the parser to get GPS data */
  if (RF_Parser_GetParsedData(gps_data, NULL, 0, NULL)) {
    /* Mark packet as processed */
    rf_packet_ready = 0;
    return RF_OK;
  }
  
  return RF_ERROR;
}

/**
 * @brief Initialize SPI for LoRa communication
 * @retval RF_OK or an RF_ERR_* code identifying the failed peripheral
 */
static uint8_t RF_SPI_Init(void)
{
  /* Enable DMA1 clock */
  __HAL_RCC_DMA1_CLK_ENABLE();
  
  /* Enable SPI2 clock */
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure GPIO pins for SPI2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* SPI2 GPIO Configuration:
     PB13 --> SPI2_SCK
     PB14 --> SPI2_MISO
     PB15 --> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Configure SPI2 */
  hspi_lora.Instance = SPI2;
  hspi_lora.Init.Mode = SPI_MODE_MASTER;
  hspi_lora.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_lora.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_lora.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_lora.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi_lora.Init.NSS = SPI_NSS_SOFT;
  hspi_lora.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // APB1 = 21MHz, prescaler 16 gives ~1.3MHz (safe for LoRa SPI)
  hspi_lora.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_lora.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_lora.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_lora.Init.CRCPolynomial = 10;
  
  if (HAL_SPI_Init(&hspi_lora) != HAL_OK) {
    return RF_ERR_SPI_INIT;
  }
  
  /* Configure DMA for SPI2 TX (DMA1 Stream 4, Channel 0) */
  hdma_spi2_tx.Instance = DMA1_Stream4;
  hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
  hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi2_tx.Init.Mode = DMA_NORMAL;
  hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  
  if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) {
    return RF_ERR_DMA_TX_INIT;
  }
  __HAL_LINKDMA(&hspi_lora, hdmatx, hdma_spi2_tx);
  
  /* Configure DMA for SPI2 RX (DMA1 Stream 3, Channel 0) */
  hdma_spi2_rx.Instance = DMA1_Stream3;
  hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
  hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi2_rx.Init.Mode = DMA_NORMAL;
  hdma_spi2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  
  if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) {
    return RF_ERR_DMA_RX_INIT;
  }
  __HAL_LINKDMA(&hspi_lora, hdmarx, hdma_spi2_rx);
  
  /* Enable DMA interrupts */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  
  return RF_OK;
}



/**
 * @brief Get diagnostic information about RF receiver
 * @param bytes_received Pointer to store total bytes received
 * @param header_matches Pointer to store header match count
 * @param last_bytes Pointer to store last received bytes (4 bytes)
 * @retval None
 */
void RF_Receiver_GetDiagnostics(uint32_t *bytes_received, uint8_t *header_matches, uint8_t *last_bytes)
{
  if (bytes_received) {
    *bytes_received = rf_bytes_received;
  }
  
  if (header_matches) {
    *header_matches = rf_header_matches;
  }
  
  if (last_bytes) {
    /* Copy last bytes */
    for (uint8_t i = 0; i < 4; i++) {
      last_bytes[i] = rf_last_bytes[i];
    }
  }
}

/**
 * @brief Get extended diagnostic information about RF receiver
 * @param checksum_errors Pointer to store checksum error count
 * @retval None
 */
void RF_Receiver_GetExtendedDiagnostics(uint16_t *checksum_errors)
{
  if (checksum_errors) {
    /* Get checksum errors from the parser */
    RF_Parser_GetChecksumErrors(checksum_errors);
  }
}



/**
 * @brief Get the last valid ASCII packet
 * @param buffer Buffer to copy the ASCII data into
 * @param size Size of the buffer
 */
void RF_Receiver_GetAsciiBuffer(char *buffer, uint16_t size) {
  if (buffer && size > 0) {
    /* Forward the call to the parser module */
    RF_Parser_GetLastValidPacket(buffer, size);
    buffer[size - 1] = '\0'; /* Ensure null termination */
  }
}

/**
 * @brief Get the last valid ASCII packet and return its length
 * @param buffer Buffer to copy the ASCII data into
 * @param max_len Maximum length of the buffer
 * @return Length of the copied data
 */
uint16_t RF_Receiver_GetLastPacketASCII(char *buffer, uint16_t max_len) {
  /* Forward the call to the parser module */
  return RF_Parser_GetLastValidPacket(buffer, max_len);
}

/**
 * @brief Get parsed data from the last valid packet
 * @param gps_data Pointer to GPS_Data structure to fill
 * @param callsign Buffer to store callsign
 * @param callsign_size Size of callsign buffer
 * @param checksum Pointer to store checksum
 * @retval Status (1 if data available, 0 if not)
 */
uint8_t RF_Receiver_GetParsedData(GPS_Data *gps_data, char *callsign, uint16_t callsign_size, uint8_t *checksum)
{
  /* Forward the call to the parser module */
  return RF_Parser_GetParsedData(gps_data, callsign, callsign_size, checksum);
}

/**
 * @brief Get total number of LoRa packets received
 * @retval Number of LoRa packets received since boot
 */
uint32_t RF_Receiver_GetLoRaPacketCount(void) {
  return rf_lora_packets_received;
}

/**
 * @brief Get LoRa IRQ diagnostics
 * @param irq_checks Pointer to store number of IRQ status checks
 * @param last_irq Pointer to store last non-zero IRQ status
 * @param device_mode Pointer to store last device operating mode (0=Unused, 2=STDBY_RC, 3=STDBY_XOSC, 4=FS, 5=RX, 6=TX)
 * @param spi_test Pointer to store SPI test result (0=fail, 1=pass, 0xFF=not tested)
 * @param busy_state Pointer to store BUSY pin state (0=low/ready, 1=high/busy)
 */
void RF_Receiver_GetIRQDiagnostics(uint32_t *irq_checks, uint16_t *last_irq, uint8_t *device_mode, uint8_t *spi_test, uint8_t *busy_state) {
  if (irq_checks) *irq_checks = rf_irq_checks;
  if (last_irq) *last_irq = rf_last_irq_status;
  if (device_mode) *device_mode = rf_last_device_mode;
  if (spi_test) *spi_test = rf_spi_test_result;
  if (busy_state) *busy_state = rf_last_busy_state;
}

/**
 * @brief Set the UART baud rate fudge factor (deprecated for LoRa)
 * @param fudge_factor Fudge factor value (in thousandths, e.g. 1000 = 1.0)
 * @retval None
 */
void RF_Receiver_SetBaudFudgeFactor(uint16_t fudge_factor)
{
  /* Not applicable for LoRa - function kept for compatibility */
  (void)fudge_factor;
}

/**
 * @brief Get the current UART baud rate fudge factor (deprecated for LoRa)
 * @retval Current fudge factor value (in thousandths)
 */
uint16_t RF_Receiver_GetBaudFudgeFactor(void)
{
  /* Not applicable for LoRa - return default value for compatibility */
  return RF_BAUD_FUDGE_FACTOR_DEFAULT;
}

/**
 * @brief Output a calibration signal on the UART TX pin (deprecated for LoRa)
 * @param duration_ms Duration to output the signal in milliseconds
 * @retval None
 */
void RF_Receiver_OutputCalibrationSignal(uint32_t duration_ms)
{
  /* Not applicable for LoRa - function kept for compatibility */
  (void)duration_ms;
}

/**
 * @brief Get signal quality metrics from last packet
 * @param rssi Pointer to store RSSI in dBm
 * @param snr Pointer to store SNR in dB
 * @retval None
 */
void RF_Receiver_GetSignalQuality(int16_t *rssi, int8_t *snr)
{
  if (rssi) *rssi = last_rssi;
  if (snr) *snr = last_snr;
}

/**
 * @brief Check if received data is stale
 * @param current_time_ms Current system time in milliseconds
 * @retval 1 if data is stale, 0 if fresh
 */
uint8_t RF_Receiver_IsDataStale(uint32_t current_time_ms)
{
  /* No packet received yet */
  if (last_packet_time == 0) {
    return 1;
  }
  
  /* Check if data is older than timeout threshold */
  uint32_t age_ms = current_time_ms - last_packet_time;
  return (age_ms > RF_DATA_STALE_TIMEOUT_MS) ? 1 : 0;
}

/**
 * @brief Get timestamp of last valid packet
 * @retval Timestamp in milliseconds
 */
uint32_t RF_Receiver_GetLastPacketTime(void)
{
  return last_packet_time;
}

/**
 * @brief Check if signal quality is good enough for reliable data
 * @retval 1 if signal quality is good, 0 if poor
 */
uint8_t RF_Receiver_IsSignalQualityGood(void)
{
  /* Check RSSI threshold */
  if (last_rssi < RF_MIN_RSSI_DBM) {
    return 0;
  }
  
  /* Check SNR threshold */
  if (last_snr < RF_MIN_SNR_DB) {
    return 0;
  }
  
  return 1;
}

/**
 * @brief Switch to a different rocket channel
 * @param channel Channel index (0..LORA_CHANNEL_COUNT-1)
 * @retval RF_OK on success, RF_ERROR otherwise
 */
uint8_t RF_Receiver_SetChannel(uint8_t channel)
{
  if (LoRa_SetChannel(channel) != LORA_OK) {
    return RF_ERROR;
  }
  /* Anything still pending was received from the previous rocket */
  rf_packet_ready = 0;
  last_packet_time = 0;
  /* Noise samples and the last heartbeat belong to the previous frequency */
  noise_sample_count = 0;
  noise_sample_idx = 0;
  noise_alert_active = 0;
  heartbeat_pending = 0;
  last_heartbeat_time = 0;
  return RF_OK;
}

/**
 * @brief Get the currently tuned rocket channel
 */
uint8_t RF_Receiver_GetChannel(void)
{
  return LoRa_GetChannel();
}

/**
 * @brief Cycle to the next rocket channel (wraps around)
 * @retval The channel that is active after the call (unchanged on failure)
 */
uint8_t RF_Receiver_NextChannel(void)
{
  uint8_t next = (uint8_t)((LoRa_GetChannel() + 1) % LORA_CHANNEL_COUNT);
  (void)RF_Receiver_SetChannel(next);
  return LoRa_GetChannel();
}

/**
 * @brief Get the last received heartbeat (one-shot)
 * @param hb Output buffer for the heartbeat packet
 * @retval 1 if a new heartbeat arrived since the last call, 0 otherwise
 */
uint8_t RF_Receiver_GetHeartbeat(HeartbeatPacket_t *hb)
{
  if (!heartbeat_pending || hb == NULL) {
    return 0;
  }
  memcpy(hb, &last_heartbeat, sizeof(*hb));
  heartbeat_pending = 0;
  return 1;
}

/**
 * @brief Get the last heartbeat without consuming it (for display)
 * @param hb Output buffer for the heartbeat packet (may be NULL)
 * @param age_ms Output: milliseconds since it was received (may be NULL)
 * @retval 1 if a heartbeat has ever been received on this channel, 0 otherwise
 */
uint8_t RF_Receiver_GetLastHeartbeat(HeartbeatPacket_t *hb, uint32_t *age_ms)
{
  if (last_heartbeat_time == 0) {
    return 0;
  }
  if (hb) memcpy(hb, &last_heartbeat, sizeof(*hb));
  if (age_ms) *age_ms = HAL_GetTick() - last_heartbeat_time;
  return 1;
}

/**
 * @brief Get the estimated ambient noise floor on the tuned channel
 * @param nf_dbm Output: noise floor in dBm
 * @retval 1 if the estimate is valid (enough samples collected), 0 otherwise
 */
uint8_t RF_Receiver_GetNoiseFloor(int16_t *nf_dbm)
{
  if (noise_sample_count < RF_NOISE_MIN_SAMPLES || nf_dbm == NULL) {
    return 0;
  }
  
  /* Partial selection sort: only the lowest RF_NOISE_PCTL_IDX+1 elements
   * are needed. Window is 16 entries so this is trivially cheap. */
  int16_t tmp[RF_NOISE_WINDOW];
  memcpy(tmp, noise_samples, noise_sample_count * sizeof(tmp[0]));
  uint8_t pctl = RF_NOISE_PCTL_IDX;
  if (pctl >= noise_sample_count) pctl = (uint8_t)(noise_sample_count - 1);
  for (uint8_t i = 0; i <= pctl; i++) {
    uint8_t min_j = i;
    for (uint8_t j = (uint8_t)(i + 1); j < noise_sample_count; j++) {
      if (tmp[j] < tmp[min_j]) min_j = j;
    }
    int16_t t = tmp[i]; tmp[i] = tmp[min_j]; tmp[min_j] = t;
  }
  *nf_dbm = tmp[pctl];
  return 1;
}

/**
 * @brief Check whether the tuned channel's noise floor is abnormally high
 * @retval 1 while the noise alert is active (with hysteresis), 0 otherwise
 */
uint8_t RF_Receiver_NoiseAlert(void)
{
  return noise_alert_active;
}

/**
 * @brief Start scanning across all rocket channels
 * @note The scan dwells RF_SCAN_DWELL_MS per channel starting from the
 *       current one, hopping until any CRC-valid LoRa packet is received.
 *       Poll RF_Receiver_ScanUpdate() from the main loop to drive it.
 */
void RF_Receiver_StartScan(void)
{
  scan_active = 1;
  scan_dwell_start = HAL_GetTick();
  scan_dwell_pkt_count = rf_lora_packets_received;
  scan_cad_phase = 1;
  scan_cad_phase_start = scan_dwell_start;
  scan_cad_state = RF_CAD_IDLE;
  scan_cad_strikes = 0;
}

/**
 * @brief Stop the channel scan (e.g. when the user picks a channel manually)
 */
void RF_Receiver_StopScan(void)
{
  if (scan_active && scan_cad_phase && scan_cad_state != RF_CAD_RX_WAIT) {
    /* The CAD phase leaves the chip in standby between sniffs; a scan
     * stopped there would leave the receiver deaf on the picked channel.
     * (RX_WAIT means the chip is already receiving - leave it alone.) */
    (void)LoRa_SetReceiveMode();
  }
  scan_active = 0;
}

/**
 * @brief Check whether a channel scan is in progress
 */
uint8_t RF_Receiver_IsScanning(void)
{
  return scan_active;
}

/**
 * @brief Leave the CAD fast phase and hand the scan to the dwell phase
 */
static void RF_Scan_FallbackToDwell(void)
{
  scan_cad_phase = 0;
  /* CAD leaves the chip in standby; the dwell phase listens continuously */
  (void)LoRa_SetReceiveMode();
  scan_dwell_start = HAL_GetTick();
  scan_dwell_pkt_count = rf_lora_packets_received;
}

/**
 * @brief Drive the channel scan; call once per main-loop iteration
 * @retval 1 the moment the scan locks onto a channel (a packet arrived),
 *         0 while still scanning or when no scan is active
 */
uint8_t RF_Receiver_ScanUpdate(void)
{
  if (!scan_active) {
    return 0;
  }
  
  /* Any CRC-valid packet on the current channel ends the scan. Uses the
   * raw LoRa packet counter (not the parser) so binary GPS, fused and
   * callsign packets all count. */
  if (rf_lora_packets_received > scan_dwell_pkt_count) {
    scan_active = 0;
    if (scan_cad_phase) {
      /* CAD_RX reception is single-shot: after RX_DONE the chip idles in
       * standby, so continuous RX must be restored on the locked channel. */
      (void)LoRa_SetReceiveMode();
    }
    return 1;
  }
  
  /* ---- CAD fast phase: preamble-sniff each channel (~0.3 s per lap) ---- */
  if (scan_cad_phase) {
    uint32_t now = HAL_GetTick();
    
    if (now - scan_cad_phase_start >= RF_SCAN_CAD_PHASE_MS) {
      RF_Scan_FallbackToDwell();
      return 0;
    }
    
    switch (scan_cad_state) {
      case RF_CAD_IDLE:
        if (LoRa_StartCad() == LORA_OK) {
          scan_cad_state = RF_CAD_RUNNING;
          scan_cad_deadline = now + RF_SCAN_CAD_TIMEOUT_MS;
        } else {
          /* Chip rejected CAD (SPI trouble, unsupported state): the dwell
           * scan needs nothing special, degrade to it immediately. */
          RF_Scan_FallbackToDwell();
        }
        break;
      
      case RF_CAD_RUNNING: {
        uint8_t res = LoRa_CadResult();
        if (res == LORA_CAD_DETECTED) {
          /* Chip dropped into RX and is capturing the packet; the DIO1 /
           * DataAvailable path reads it and the lock check above fires. */
          scan_cad_state = RF_CAD_RX_WAIT;
          scan_cad_deadline = now + RF_SCAN_CAD_RX_WAIT_MS;
          scan_cad_strikes = 0;
        } else if (res == LORA_CAD_NONE) {
          /* Quiet channel: hop and sniff the next one */
          (void)RF_Receiver_NextChannel();
          scan_dwell_pkt_count = rf_lora_packets_received;
          scan_cad_state = RF_CAD_IDLE;
          scan_cad_strikes = 0;
        } else if (res == LORA_CAD_FAIL) {
          /* SPI fault reading the result: don't trust CAD at all */
          RF_Scan_FallbackToDwell();
        } else if ((int32_t)(now - scan_cad_deadline) >= 0) {
          /* CAD never completed. Usually a transient abort - e.g. the
           * one-time RF_EnsureRxMode() call issues a SetRx that cancels
           * whatever CAD is in flight when the main loop first polls the
           * radio - so retry before concluding CAD doesn't work here. */
          if (++scan_cad_strikes >= RF_SCAN_CAD_MAX_STRIKES) {
            RF_Scan_FallbackToDwell();
          } else {
            scan_cad_state = RF_CAD_IDLE;
          }
        }
        /* LORA_CAD_PENDING within the deadline: keep polling */
        break;
      }
      
      case RF_CAD_RX_WAIT:
      default:
        if ((int32_t)(now - scan_cad_deadline) >= 0) {
          /* False detection (noise burst): nothing decodable arrived */
          (void)RF_Receiver_NextChannel();
          scan_dwell_pkt_count = rf_lora_packets_received;
          scan_cad_state = RF_CAD_IDLE;
        }
        break;
    }
    return 0;
  }
  
  /* ---- Dwell phase: listen RF_SCAN_DWELL_MS per channel ---- */
  /* Dwell expired with nothing heard: hop to the next channel. But never
   * hop over an unread packet: LoRa_SetChannel discards the radio's pending
   * flag, so a heartbeat that landed at the end of the dwell (the only one
   * we'll get - they are 1 per dwell) would be lost and the scan would keep
   * cycling past a live beacon. Let DataAvailable() read it first; the lock
   * check above fires on the next call. */
  if (HAL_GetTick() - scan_dwell_start >= RF_SCAN_DWELL_MS) {
    if (LoRa_PacketAvailable()) {
      return 0;
    }
    (void)RF_Receiver_NextChannel();
    scan_dwell_start = HAL_GetTick();
    scan_dwell_pkt_count = rf_lora_packets_received;
  }
  
  return 0;
}

/**
 * @brief Get packet loss diagnostics
 * @param irq_count Pointer to store total DIO1 interrupts (packets arriving at radio)
 * @param lora_packets Pointer to store LoRa packets read from radio
 * @param duplicates Pointer to store duplicate packets rejected
 * @retval None
 */
void RF_Receiver_GetPacketLossDiagnostics(uint32_t *irq_count, uint32_t *lora_packets, uint32_t *duplicates)
{
  if (irq_count) *irq_count = LoRa_GetIRQCount();
  if (lora_packets) *lora_packets = rf_lora_packets_received;
  if (duplicates) *duplicates = 0;  /* Duplicate detection disabled */
}

/**
 * @brief DMA1 Stream 3 IRQ handler (SPI2 RX)
 */
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

/**
 * @brief DMA1 Stream 4 IRQ handler (SPI2 TX)
 */
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

/* Note: HAL_SPI_RxCpltCallback (for Receive_DMA completion) is defined in
 * lora.c and clears lora_spi_dma_busy. HAL_SPI_TxRxCpltCallback and
 * HAL_SPI_TxCpltCallback are not used because TX and full-duplex DMA paths
 * are not exercised. Add them here dispatching on hspi->Instance if needed. */
