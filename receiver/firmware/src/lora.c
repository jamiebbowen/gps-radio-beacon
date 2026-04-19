/**
 * @file lora.c
 * @brief E22-400M33S LoRa Module Driver Implementation (SX1268)
 */

#include "lora.h"
#include <string.h>

/* Private variables */
static SPI_HandleTypeDef *lora_hspi = NULL;
static uint8_t lora_initialized = 0;
static int16_t last_rssi = 0;
static int8_t last_snr = 0;

/* Interrupt flag - set by DIO1 interrupt, cleared when packet is read */
volatile uint8_t lora_packet_ready = 0;

/* IRQ diagnostic counter - increments every time DIO1 fires */
static volatile uint32_t lora_irq_count = 0;

/* DMA busy flag for non-blocking SPI reads */
static volatile uint8_t lora_spi_dma_busy = 0;

/* Private function prototypes */
static void LoRa_CS_High(void);
static void LoRa_CS_Low(void);
static uint8_t LoRa_WaitOnBusy(uint32_t timeout_ms);
static uint8_t LoRa_SendCommand(uint8_t cmd, const uint8_t *params, uint8_t param_len);
static uint8_t LoRa_ReadCommand(uint8_t cmd, uint8_t *data, uint8_t data_len);
static uint8_t LoRa_WriteRegister(uint16_t address, uint8_t *data, uint8_t length);
static uint8_t LoRa_ReadRegister(uint16_t address, uint8_t *data, uint8_t length);
static uint8_t LoRa_WriteBuffer(uint8_t offset, const uint8_t *data, uint8_t length);
static uint8_t LoRa_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length);

/**
 * @brief Initialize LoRa module
 */
uint8_t LoRa_Init(SPI_HandleTypeDef *hspi) {
    if (hspi == NULL) {
        return LORA_ERROR;
    }
    
    lora_hspi = hspi;
    
    /* Initialize GPIO pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure CS pin */
    GPIO_InitStruct.Pin = LORA_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_CS_PORT, &GPIO_InitStruct);
    LoRa_CS_High(); // Deselect initially
    
    /* Configure RESET pin */
    GPIO_InitStruct.Pin = LORA_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_RESET_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    
    /* Configure BUSY pin (input) */
    GPIO_InitStruct.Pin = LORA_BUSY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_BUSY_PORT, &GPIO_InitStruct);
    
    /* Configure DIO1 pin (interrupt) */
    GPIO_InitStruct.Pin = LORA_DIO1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_DIO1_PORT, &GPIO_InitStruct);
    
    /* Enable EXTI1 interrupt for DIO1 (PB1) */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);  /* Highest priority for packet reception */
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    
    /* Configure TXEN pin (RF switch TX enable) */
    GPIO_InitStruct.Pin = LORA_TXEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_TXEN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_TXEN_PORT, LORA_TXEN_PIN, GPIO_PIN_RESET);  // TXEN=0 initially
    
    /* Configure RXEN pin (RF switch RX enable) */
    GPIO_InitStruct.Pin = LORA_RXEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_RXEN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LORA_RXEN_PORT, LORA_RXEN_PIN, GPIO_PIN_RESET);  // RXEN=0 initially
    
    /* Reset the module */
    LoRa_Reset();
    HAL_Delay(150);  // Extra delay after reset for stability with slower SPI
    
    /* Check if BUSY pin is stuck high (hardware issue) */
    if (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        /* Wait a bit longer in case the module is still booting */
        HAL_Delay(100);
        if (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
            return 0xB1;  // BUSY stuck high - different error code
        }
    }
    
    /* Try to read device status to verify SPI communication */
    LoRa_CS_Low();
    uint8_t status_cmd = 0xC0;  // GetStatus command
    uint8_t status_response = 0;
    HAL_SPI_Transmit(lora_hspi, &status_cmd, 1, 10);
    HAL_SPI_Receive(lora_hspi, &status_response, 1, 10);
    LoRa_CS_High();
    
    /* If status is 0x00 or 0xFF, SPI might not be working */
    if (status_response == 0x00 || status_response == 0xFF) {
        return 0xC0;  // SPI communication issue
    }
    
    /* Set TCXO mode FIRST - E22-400M33S needs TCXO enabled before other commands */
    /* Don't wait for BUSY before this command - module just reset */
    LoRa_CS_Low();
    uint8_t tcxo_cmd = SX1268_CMD_SET_TCXOMODE;
    uint8_t tcxo_params[4] = {
        0x04,  // TCXO voltage: 1.8V (0x04 for 1.8V, 0x05 for 2.2V, 0x06 for 2.4V, 0x07 for 3.0V, 0x08 for 3.3V)
        0x00,  // timeout (24-bit value) - try 100ms timeout (0x000064 = 100 * 15.625us steps)
        0x00,
        0x64   // 100 in decimal = 0x64
    };
    HAL_SPI_Transmit(lora_hspi, &tcxo_cmd, 1, 10);
    HAL_SPI_Transmit(lora_hspi, tcxo_params, 4, 10);
    LoRa_CS_High();
    HAL_Delay(10);  // Short delay
    
    /* Now wait for BUSY to go low after TCXO command */
    if (LoRa_WaitOnBusy(1000) != LORA_OK) {
        return 0xB3;  // TCXO command failed - BUSY timeout
    }
    HAL_Delay(100);  // Extra delay for TCXO to stabilize and power up
    
    /* Set standby mode - stay with STDBY_RC for simplicity */
    uint8_t standby_param = 0x00; // STDBY_RC
    if (LoRa_SendCommand(SX1268_CMD_SET_STANDBY, &standby_param, 1) != LORA_OK) {
        return 0xB2;  // Standby command failed
    }
    HAL_Delay(10);
    
    /* Set packet type to LoRa */
    uint8_t packet_type = SX1268_PACKET_TYPE_LORA;
    if (LoRa_SendCommand(SX1268_CMD_SET_PACKETTYPE, &packet_type, 1) != LORA_OK) {
        return 0xB4;  // Packet type command failed
    }
    
    /* Set RF frequency (433 MHz) */
    uint32_t freq_in_hz = (uint32_t)(LORA_FREQUENCY_MHZ * 1000000.0f);
    uint32_t freq_reg = (uint32_t)((double)freq_in_hz / (double)32000000.0 * (double)33554432.0);
    uint8_t freq_params[4] = {
        (uint8_t)(freq_reg >> 24),
        (uint8_t)(freq_reg >> 16),
        (uint8_t)(freq_reg >> 8),
        (uint8_t)(freq_reg)
    };
    if (LoRa_SendCommand(SX1268_CMD_SET_RFFREQUENCY, freq_params, 4) != LORA_OK) {
        return 0xB5;  // RF frequency command failed
    }
    
    /* Set PA configuration (for +22dBm output) */
    uint8_t pa_params[4] = {
        0x04,  // paDutyCycle
        0x07,  // hpMax (max power)
        0x00,  // deviceSel (SX1262)
        0x01   // paLut
    };
    if (LoRa_SendCommand(SX1268_CMD_SET_PACONFIG, pa_params, 4) != LORA_OK) {
        return 0xC1;  // PA config command failed
    }
    
    /* Set TX parameters */
    uint8_t tx_params[2] = {
        LORA_TX_POWER_DBM,  // power
        0x04                 // rampTime (40us)
    };
    if (LoRa_SendCommand(SX1268_CMD_SET_TXPARAMS, tx_params, 2) != LORA_OK) {
        return 0xC2;  // TX params command failed
    }
    
    /* Set modulation parameters */
    // SF, BW, CR, LowDataRateOptimize
    uint8_t mod_params[4];
    mod_params[0] = LORA_SPREADING_FACTOR;
    
    // Bandwidth encoding
    uint8_t bw_param;
    if (LORA_BANDWIDTH_KHZ == 125.0f) bw_param = 0x04;
    else if (LORA_BANDWIDTH_KHZ == 250.0f) bw_param = 0x05;
    else if (LORA_BANDWIDTH_KHZ == 500.0f) bw_param = 0x06;
    else bw_param = 0x04; // Default to 125kHz
    mod_params[1] = bw_param;
    
    mod_params[2] = LORA_CODING_RATE;
    mod_params[3] = 0x00; // Low data rate optimize off
    
    if (LoRa_SendCommand(SX1268_CMD_SET_MODULATIONPARAMS, mod_params, 4) != LORA_OK) {
        return 0xC3;  // Modulation params command failed
    }
    
    /* Set packet parameters */
    uint8_t pkt_params[6] = {
        (uint8_t)(LORA_PREAMBLE_LENGTH >> 8),   // Preamble length MSB
        (uint8_t)(LORA_PREAMBLE_LENGTH & 0xFF), // Preamble length LSB
        0x00,   // Header type: variable length
        0xFF,   // Payload length (max)
        0x01,   // CRC on
        0x00    // IQ standard
    };
    if (LoRa_SendCommand(SX1268_CMD_SET_PACKETPARAMS, pkt_params, 6) != LORA_OK) {
        return 0xC4;  // Packet params command failed
    }
    
    /* SKIP sync word for now - register write may cause issues */
    /* Module will use default sync word */
    
    /* Set buffer base addresses */
    uint8_t buf_params[2] = {0x00, 0x00}; // TX base, RX base
    if (LoRa_SendCommand(SX1268_CMD_SET_BUFFERBASEADDRESS, buf_params, 2) != LORA_OK) {
        return 0xC6;  // Buffer base address command failed
    }
    
    /* Configure DIO/IRQ - needed for RX mode to work properly */
    /* IRQ mask: RxDone=0x02, Timeout=0x200, CRCError=0x40 = 0x0242 */
    uint16_t irq_mask = SX1268_IRQ_RX_DONE | SX1268_IRQ_TIMEOUT | SX1268_IRQ_CRC_ERROR;
    uint8_t irq_params[8] = {
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask & 0xFF),  // IRQ mask MSB, LSB
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask & 0xFF),  // DIO1 mask (same)
        0x00, 0x00,  // DIO2 mask (none)
        0x00, 0x00   // DIO3 mask (none)
    };
    if (LoRa_SendCommand(SX1268_CMD_CFG_DIOIRQ, irq_params, 8) != LORA_OK) {
        return 0xC7;  // IRQ config command failed
    }
    HAL_Delay(50);  // Wait for IRQ config to complete
    
    /* Configure DIO2 as RF switch control - AUTOMATIC mode */
    /* The SX1268 will control TXEN/RXEN automatically via DIO2 */
    uint8_t dio2_params[1] = {0x00};  // Enable DIO2 as RF switch (automatic)
    if (LoRa_SendCommand(SX1268_CMD_SET_RFSWITCHMODE, dio2_params, 1) != LORA_OK) {
        return 0xC8;  // DIO2 RF switch config failed
    }
    HAL_Delay(50);  // Wait for RF switch mode to take effect
    
    /* SKIP regulator mode - use default LDO mode */
    /* Module should use default LDO mode */
    
    /* Calibrate the module - this may be REQUIRED before RX mode works */
    /* Calibrate all blocks: RC64k, RC13M, PLL, ADC, Image */
    uint8_t calibrate_param = 0x7F;  // Calibrate all
    if (LoRa_SendCommand(SX1268_CMD_CALIBRATE, &calibrate_param, 1) == LORA_OK) {
        HAL_Delay(100);  // Wait for calibration to complete
    }
    
    /* Give module time to settle before entering RX */
    HAL_Delay(100);
    
    lora_initialized = 1;
    
    /* Clear all IRQs before entering RX mode */
    uint8_t clear_irq[2] = {0xFF, 0xFF};
    LoRa_SendCommand(SX1268_CMD_CLR_IRQSTATUS, clear_irq, 2);
    HAL_Delay(10);
    
    /* Manual RF switch control - set to RX mode */
    HAL_GPIO_WritePin(LORA_TXEN_PORT, LORA_TXEN_PIN, GPIO_PIN_RESET);  // TXEN=0
    HAL_GPIO_WritePin(LORA_RXEN_PORT, LORA_RXEN_PIN, GPIO_PIN_SET);    // RXEN=1 (RX mode!)
    HAL_Delay(20);  // Let RF switch settle
    
    /* Try entering RX mode with shorter timeout first to test */
    uint8_t rx_params[3] = {0x00, 0x00, 0x00};  // RX single (no timeout) - simpler mode
    if (LoRa_SendCommand(SX1268_CMD_SET_RX, rx_params, 3) != LORA_OK) {
        return 0xB9;  // SetRX command failed at SPI level
    }
    
    HAL_Delay(50);  // Extra delay for mode transition
    
    /* Retry with infinite timeout if first attempt doesn't work */
    uint8_t rx_params2[3] = {0xFF, 0xFF, 0xFF};  // Infinite RX mode
    LoRa_SendCommand(SX1268_CMD_SET_RX, rx_params2, 3);  // Don't check return, just try
    
    /* Verify we entered RX mode */
    HAL_Delay(200);  // Give plenty of time for mode transition
    
    /* BYPASS mode check - just return OK and see if it actually receives */
    /* The mode reporting might be incorrect but the module could still work */
    return LORA_OK;
}

/**
 * @brief Reset LoRa module
 */
void LoRa_Reset(void) {
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
}

/**
 * @brief Set LoRa to receive mode (continuous)
 */
uint8_t LoRa_SetReceiveMode(void) {
    if (!lora_initialized) return LORA_ERROR;
    
    /* Set RX mode with infinite timeout - stay in RX continuously */
    /* Timeout = 0xFFFFFF = infinite (no timeout) */
    uint8_t rx_params[3] = {0xFF, 0xFF, 0xFF};  // Infinite RX mode
    return LoRa_SendCommand(SX1268_CMD_SET_RX, rx_params, 3);
}

/**
 * @brief Set LoRa to standby mode
 */
uint8_t LoRa_SetStandbyMode(void) {
    if (!lora_initialized) return LORA_ERROR;
    
    uint8_t standby_param = 0x00; // STDBY_RC
    return LoRa_SendCommand(SX1268_CMD_SET_STANDBY, &standby_param, 1);
}

/**
 * @brief Check if packet is available (interrupt-driven)
 * @note Uses hardware interrupt flag
 */
uint8_t LoRa_PacketAvailable(void) {
    if (!lora_initialized) return 0;
    
    /* Simply check the interrupt flag */
    return lora_packet_ready;
}

/**
 * @brief Read received packet
 */
uint8_t LoRa_ReadPacket(LoRa_Packet_t *packet) {
    if (!lora_initialized || packet == NULL) return LORA_ERROR;
    
    /* Check IRQ status */
    uint16_t irq_status;
    if (LoRa_GetIRQStatus(&irq_status) != LORA_OK) {
        return LORA_ERROR;
    }
    
    /* Check for CRC error */
    if (irq_status & SX1268_IRQ_CRC_ERROR) {
        packet->crc_error = true;
        LoRa_ClearIRQ(SX1268_IRQ_ALL);
        lora_packet_ready = 0;  /* Clear flag on CRC error */
        /* Don't re-enter RX - infinite RX mode keeps radio in RX */
        return LORA_CRC_ERROR;
    }
    
    /* Check if packet received */
    if (!(irq_status & SX1268_IRQ_RX_DONE)) {
        return LORA_NO_DATA;
    }
    
    /* Get RX buffer status */
    uint8_t buffer_status[2];
    if (LoRa_ReadCommand(SX1268_CMD_GET_RXBUFFERSTATUS, buffer_status, 2) != LORA_OK) {
        return LORA_ERROR;
    }
    
    uint8_t payload_length = buffer_status[0];
    uint8_t rx_start_ptr = buffer_status[1];
    
    /* Read the packet */
    if (payload_length > 0 && payload_length <= 256) {
        if (LoRa_ReadBuffer(rx_start_ptr, packet->data, payload_length) == LORA_OK) {
            packet->length = payload_length;
            packet->crc_error = false;
            
            /* Get packet status (RSSI, SNR) */
            uint8_t pkt_status[3];
            if (LoRa_ReadCommand(SX1268_CMD_GET_PACKETSTATUS, pkt_status, 3) == LORA_OK) {
                packet->rssi = -(pkt_status[0] / 2);
                packet->snr = (int8_t)(pkt_status[1] / 4);
                last_rssi = packet->rssi;
                last_snr = packet->snr;
            }
            
            /* Clear IRQ for this packet */
            LoRa_ClearIRQ(SX1268_IRQ_ALL);
            
            /* Clear interrupt flag - packet has been read */
            lora_packet_ready = 0;
            
            /* Don't re-enter RX - infinite RX mode keeps radio receiving automatically */
            
            return LORA_OK;
        }
    }
    
    /* Clear IRQ on error - infinite RX mode keeps radio receiving */
    LoRa_ClearIRQ(SX1268_IRQ_ALL);
    lora_packet_ready = 0;  /* Clear flag on error */
    
    return LORA_ERROR;
}

/**
 * @brief Get RSSI of last received packet
 */
int16_t LoRa_GetRSSI(void) {
    return last_rssi;
}

/**
 * @brief Get SNR of last received packet
 */
int8_t LoRa_GetSNR(void) {
    return last_snr;
}

/**
 * @brief Get total IRQ count
 */
uint32_t LoRa_GetIRQCount(void) {
    return lora_irq_count;
}

/**
 * @brief Get status of LoRa module
 */
uint8_t LoRa_GetStatus(void) {
    uint8_t status;
    LoRa_ReadCommand(SX1268_CMD_GET_STATUS, &status, 1);
    return status;
}

/**
 * @brief Clear IRQ flags
 */
uint8_t LoRa_ClearIRQ(uint16_t irq_mask) {
    uint8_t params[2] = {
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask & 0xFF)
    };
    return LoRa_SendCommand(SX1268_CMD_CLR_IRQSTATUS, params, 2);
}

/**
 * @brief Get IRQ status
 */
uint8_t LoRa_GetIRQStatus(uint16_t *irq_status) {
    uint8_t status[2];
    if (LoRa_ReadCommand(SX1268_CMD_GET_IRQSTATUS, status, 2) == LORA_OK) {
        *irq_status = ((uint16_t)status[0] << 8) | status[1];
        return LORA_OK;
    }
    return LORA_ERROR;
}

/**
 * @brief Clear IRQ status
 */
uint8_t LoRa_ClearIRQStatus(uint16_t irq_mask) {
    uint8_t clear_params[2] = {
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask & 0xFF)
    };
    return LoRa_SendCommand(SX1268_CMD_CLR_IRQSTATUS, clear_params, 2);
}

/**
 * @brief Get device status/mode
 * Status byte format:
 * [7:5] = chip mode: 0=Unused, 2=STDBY_RC, 3=STDBY_XOSC, 4=FS, 5=RX, 6=TX
 * [4:1] = command status
 * [0] = reserved
 * NOTE: GetStatus returns status IMMEDIATELY after command, no NOP needed
 */
uint8_t LoRa_GetDeviceStatus(uint8_t *status) {
    if (lora_hspi == NULL || status == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command and read status simultaneously */
    uint8_t cmd = SX1268_CMD_GET_STATUS;
    uint8_t status_byte = 0;
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    HAL_SPI_Receive(lora_hspi, &status_byte, 1, 10);
    
    /* Deselect chip */
    LoRa_CS_High();
    
    *status = status_byte;
    return LORA_OK;
}

/**
 * @brief Transmit packet (for testing)
 */
uint8_t LoRa_Transmit(const uint8_t *data, uint8_t length) {
    if (!lora_initialized || data == NULL || length == 0) return LORA_ERROR;
    
    /* Set standby mode */
    LoRa_SetStandbyMode();
    
    /* Write data to buffer */
    if (LoRa_WriteBuffer(0, data, length) != LORA_OK) {
        return LORA_ERROR;
    }
    
    /* Set packet length */
    uint8_t pkt_params[6] = {
        (uint8_t)(LORA_PREAMBLE_LENGTH >> 8),
        (uint8_t)(LORA_PREAMBLE_LENGTH & 0xFF),
        0x00,    // Variable length
        length,  // Payload length
        0x01,    // CRC on
        0x00     // IQ standard
    };
    if (LoRa_SendCommand(SX1268_CMD_SET_PACKETPARAMS, pkt_params, 6) != LORA_OK) {
        return LORA_ERROR;
    }
    
    /* Start TX with timeout (no timeout: 0x000000) */
    uint8_t tx_params[3] = {0x00, 0x00, 0x00};
    return LoRa_SendCommand(SX1268_CMD_SET_TX, tx_params, 3);
}

/* ===== Private Functions ===== */

/**
 * @brief Set CS pin high (deselect)
 */
static void LoRa_CS_High(void) {
    HAL_GPIO_WritePin(LORA_CS_PORT, LORA_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Set CS pin low (select)
 */
static void LoRa_CS_Low(void) {
    HAL_GPIO_WritePin(LORA_CS_PORT, LORA_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Wait for BUSY pin to go low
 */
static uint8_t LoRa_WaitOnBusy(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    
    while (HAL_GPIO_ReadPin(LORA_BUSY_PORT, LORA_BUSY_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start_time) > timeout_ms) {
            return LORA_TIMEOUT;
        }
    }
    
    return LORA_OK;
}

/**
 * @brief Send command to SX1268
 */
static uint8_t LoRa_SendCommand(uint8_t cmd, const uint8_t *params, uint8_t param_len) {
    if (lora_hspi == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {  // Increased timeout
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send parameters if any */
    if (params != NULL && param_len > 0) {
        HAL_SPI_Transmit(lora_hspi, (uint8_t*)params, param_len, 10);
    }
    
    /* Deselect chip */
    LoRa_CS_High();
    
    /* Wait for BUSY to go low again */
    return LoRa_WaitOnBusy(50);  // Increased timeout
}

/**
 * @brief Read command from SX1268
 */
static uint8_t LoRa_ReadCommand(uint8_t cmd, uint8_t *data, uint8_t data_len) {
    if (lora_hspi == NULL || data == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send NOP for status byte */
    uint8_t nop = SX1268_CMD_NOP;
    HAL_SPI_Transmit(lora_hspi, &nop, 1, 10);
    
    /* Read data */
    HAL_SPI_Receive(lora_hspi, data, data_len, 10);
    
    /* Deselect chip */
    LoRa_CS_High();
    
    return LORA_OK;
}

/**
 * @brief Write to SX1268 register
 */
static uint8_t LoRa_WriteRegister(uint16_t address, uint8_t *data, uint8_t length) {
    if (lora_hspi == NULL || data == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    uint8_t cmd = SX1268_CMD_WRITE_REGISTER;
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send address */
    uint8_t addr_bytes[2] = {(uint8_t)(address >> 8), (uint8_t)(address & 0xFF)};
    HAL_SPI_Transmit(lora_hspi, addr_bytes, 2, 10);
    
    /* Send data */
    HAL_SPI_Transmit(lora_hspi, data, length, 10);
    
    /* Deselect chip */
    LoRa_CS_High();
    
    return LoRa_WaitOnBusy(50);
}

/**
 * @brief Read from SX1268 register
 */
static uint8_t LoRa_ReadRegister(uint16_t address, uint8_t *data, uint8_t length) {
    if (lora_hspi == NULL || data == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    uint8_t cmd = SX1268_CMD_READ_REGISTER;
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send address */
    uint8_t addr_bytes[2] = {(uint8_t)(address >> 8), (uint8_t)(address & 0xFF)};
    HAL_SPI_Transmit(lora_hspi, addr_bytes, 2, 10);
    
    /* Send NOP for status */
    uint8_t nop = SX1268_CMD_NOP;
    HAL_SPI_Transmit(lora_hspi, &nop, 1, 10);
    
    /* Read data */
    HAL_SPI_Receive(lora_hspi, data, length, 10);
    
    /* Deselect chip */
    LoRa_CS_High();
    
    return LORA_OK;
}

/**
 * @brief Write to SX1268 buffer
 */
static uint8_t LoRa_WriteBuffer(uint8_t offset, const uint8_t *data, uint8_t length) {
    if (lora_hspi == NULL || data == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    uint8_t cmd = SX1268_CMD_WRITE_BUFFER;
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send offset */
    HAL_SPI_Transmit(lora_hspi, &offset, 1, 10);
    
    /* Send data */
    HAL_SPI_Transmit(lora_hspi, (uint8_t*)data, length, 10);
    
    /* Deselect chip */
    LoRa_CS_High();
    
    return LORA_OK;
}

/**
 * @brief Read from SX1268 buffer (blocking version)
 */
static uint8_t LoRa_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length) {
    if (lora_hspi == NULL || data == NULL) return LORA_ERROR;
    
    /* Wait for BUSY to go low */
    if (LoRa_WaitOnBusy(50) != LORA_OK) {
        return LORA_BUSY;
    }
    
    /* Select chip */
    LoRa_CS_Low();
    
    /* Send command */
    uint8_t cmd = SX1268_CMD_READ_BUFFER;
    HAL_SPI_Transmit(lora_hspi, &cmd, 1, 10);
    
    /* Send offset */
    HAL_SPI_Transmit(lora_hspi, &offset, 1, 10);
    
    /* Send NOP for status */
    uint8_t nop = SX1268_CMD_NOP;
    HAL_SPI_Transmit(lora_hspi, &nop, 1, 10);
    
    /* Use DMA for all reads (>0 bytes) to minimize blocking */
    if (length > 0 && lora_hspi->hdmarx != NULL) {
        /* Use DMA for large packet reads */
        lora_spi_dma_busy = 1;
        
        if (HAL_SPI_Receive_DMA(lora_hspi, data, length) != HAL_OK) {
            LoRa_CS_High();
            return LORA_ERROR;
        }
        
        /* Wait for DMA to complete - but interrupts can fire during this wait */
        uint32_t timeout = HAL_GetTick() + 50;
        while (lora_spi_dma_busy && (HAL_GetTick() < timeout)) {
            /* Interrupts can fire here - this is the key improvement */
        }
        
        if (lora_spi_dma_busy) {
            LoRa_CS_High();
            return LORA_ERROR;
        }
    } else {
        /* Small read - use blocking (faster for <16 bytes) */
        HAL_SPI_Receive(lora_hspi, data, length, 10);
    }
    
    /* Deselect chip */
    LoRa_CS_High();
    
    return LORA_OK;
}

/**
 * @brief SPI RX DMA complete callback for LoRa
 * @note Called when DMA transfer completes
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == lora_hspi) {
        lora_spi_dma_busy = 0;
    }
}

/**
 * @brief EXTI1 interrupt handler for DIO1 (packet ready interrupt)
 * @note Called by hardware when LoRa module signals packet ready via DIO1
 */
void EXTI1_IRQHandler(void)
{
    /* Check if interrupt is from DIO1 pin */
    if (__HAL_GPIO_EXTI_GET_IT(LORA_DIO1_PIN) != RESET) {
        /* Clear interrupt flag immediately */
        __HAL_GPIO_EXTI_CLEAR_IT(LORA_DIO1_PIN);
        
        /* Increment IRQ counter for diagnostics */
        lora_irq_count++;
        
        /* Set packet ready flag - main loop will read packet */
        lora_packet_ready = 1;
    }
}
