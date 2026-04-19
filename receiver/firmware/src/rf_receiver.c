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

/* Private variables */
static SPI_HandleTypeDef hspi_lora;
static DMA_HandleTypeDef hdma_spi2_tx;
static DMA_HandleTypeDef hdma_spi2_rx;
static volatile uint8_t spi_dma_busy = 0;
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

/* Private function prototypes */
static void RF_SPI_Init(void);

/**
 * @brief Initialize RF receiver
 * @retval Status code
 */
uint8_t RF_Receiver_Init(void)
{
  /* Initialize SPI for LoRa communication */
  RF_SPI_Init();
  
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
  
  /* Check for new LoRa packets */
  if (!rf_packet_ready && LoRa_PacketAvailable()) {
    /* Read the packet */
    if (LoRa_ReadPacket(&last_packet) == LORA_OK) {
      rf_lora_packets_received++;  // Count every LoRa packet received
      rf_bytes_received += last_packet.length;
      last_rssi = last_packet.rssi;
      last_snr = last_packet.snr;
      
      /* Update last bytes for diagnostics */
      for (int i = 0; i < 4 && i < last_packet.length; i++) {
        rf_last_bytes[i] = last_packet.data[last_packet.length - 4 + i];
      }
      
      /* Parse packet based on format */
#if USE_BINARY_PACKETS
      /* Binary packet format */
      if (last_packet.length == 13 && last_packet.data[0] == PACKET_TYPE_GPS) {
        /* Parse binary packet */
        if (RF_Parser_ParseBinaryPacket(last_packet.data, last_packet.length) == RF_PARSER_OK) {
          rf_packet_ready = 1;
          rf_header_matches++;
          last_packet_time = HAL_GetTick();
        }
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
 * @retval None
 */
static void RF_SPI_Init(void)
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
    /* Error handling */
    while(1);
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
    while(1);
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
    while(1);
  }
  __HAL_LINKDMA(&hspi_lora, hdmarx, hdma_spi2_rx);
  
  /* Enable DMA interrupts */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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

/**
 * @brief SPI TX/RX DMA complete callback
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2) {
    spi_dma_busy = 0;
  }
}
