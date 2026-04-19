/**
 * @file lora.h
 * @brief E22-400M33S LoRa Module Driver Header (SX1268)
 * @note This driver interfaces with the E22-400M33S LoRa module using SPI
 */

#ifndef __LORA_H
#define __LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gps.h"
#include <stdint.h>
#include <stdbool.h>

/* LoRa Configuration - Match transmitter settings */
#define LORA_FREQUENCY_MHZ      433.0f      // 433 MHz
#define LORA_BANDWIDTH_KHZ      125.0f      // 125 kHz bandwidth
#define LORA_SPREADING_FACTOR   9           // SF9 (good range/speed balance)
#define LORA_CODING_RATE        7           // 4/7
#define LORA_SYNC_WORD          0x12        // Private sync word
#define LORA_TX_POWER_DBM       22          // 22 dBm (~160mW - SX1268 chip maximum)
#define LORA_PREAMBLE_LENGTH    8           // Preamble length

/* SX1268 Pin Configuration for STM32F4 
 * Using SPI2: PB13=SCK, PB14=MISO, PB15=MOSI
 */
#define LORA_SPI_INSTANCE       SPI2
#define LORA_CS_PIN             GPIO_PIN_12     // PB12 - Chip Select
#define LORA_CS_PORT            GPIOB
#define LORA_RESET_PIN          GPIO_PIN_8      // PA8 - Reset
#define LORA_RESET_PORT         GPIOA
#define LORA_BUSY_PIN           GPIO_PIN_0      // PB0 - Busy (moved from PA7 to avoid SD card conflict)
#define LORA_BUSY_PORT          GPIOB
#define LORA_DIO1_PIN           GPIO_PIN_1      // PB1 - DIO1 (interrupt) (moved from PA6 to avoid SD card conflict)
#define LORA_DIO1_PORT          GPIOB
#define LORA_TXEN_PIN           GPIO_PIN_10     // PB10 - TXEN (RF switch TX enable) - NOT CONNECTED
#define LORA_TXEN_PORT          GPIOB
#define LORA_RXEN_PIN           GPIO_PIN_9      // PA9 - RXEN (RF switch RX enable)
#define LORA_RXEN_PORT          GPIOA

/* SX1268 Register Commands */
#define SX1268_CMD_NOP                      0x00
#define SX1268_CMD_SET_SLEEP                0x84
#define SX1268_CMD_SET_STANDBY              0x80
#define SX1268_CMD_SET_FS                   0xC1
#define SX1268_CMD_SET_TX                   0x83
#define SX1268_CMD_SET_RX                   0x82
#define SX1268_CMD_SET_RXDUTYCYCLE          0x94
#define SX1268_CMD_SET_CAD                  0xC5
#define SX1268_CMD_SET_TXCONTINUOUSWAVE     0xD1
#define SX1268_CMD_SET_TXCONTINUOUSPREAMBLE 0xD2
#define SX1268_CMD_SET_PACKETTYPE           0x8A
#define SX1268_CMD_GET_PACKETTYPE           0x11
#define SX1268_CMD_SET_RFFREQUENCY          0x86
#define SX1268_CMD_SET_TXPARAMS             0x8E
#define SX1268_CMD_SET_PACONFIG             0x95
#define SX1268_CMD_SET_CADPARAMS            0x88
#define SX1268_CMD_SET_BUFFERBASEADDRESS    0x8F
#define SX1268_CMD_SET_MODULATIONPARAMS     0x8B
#define SX1268_CMD_SET_PACKETPARAMS         0x8C
#define SX1268_CMD_GET_RXBUFFERSTATUS       0x13
#define SX1268_CMD_GET_PACKETSTATUS         0x14
#define SX1268_CMD_GET_RSSIINST             0x15
#define SX1268_CMD_GET_STATS                0x10
#define SX1268_CMD_RESET_STATS              0x00
#define SX1268_CMD_CFG_DIOIRQ               0x08
#define SX1268_CMD_GET_IRQSTATUS            0x12
#define SX1268_CMD_CLR_IRQSTATUS            0x02
#define SX1268_CMD_CALIBRATE                0x89
#define SX1268_CMD_CALIBRATEIMAGE           0x98
#define SX1268_CMD_SET_REGULATORMODE        0x96
#define SX1268_CMD_GET_ERROR                0x17
#define SX1268_CMD_CLR_ERROR                0x07
#define SX1268_CMD_SET_TCXOMODE             0x97
#define SX1268_CMD_SET_TXFALLBACKMODE       0x93
#define SX1268_CMD_SET_RFSWITCHMODE         0x9D
#define SX1268_CMD_SET_STOPRXTIMERONPREAMBLE 0x9F
#define SX1268_CMD_SET_LORASYMBTIMEOUT      0xA0
#define SX1268_CMD_READ_REGISTER            0x1D
#define SX1268_CMD_WRITE_REGISTER           0x0D
#define SX1268_CMD_READ_BUFFER              0x1E
#define SX1268_CMD_WRITE_BUFFER             0x0E
#define SX1268_CMD_GET_STATUS               0xC0

/* SX1268 Packet Types */
#define SX1268_PACKET_TYPE_GFSK             0x00
#define SX1268_PACKET_TYPE_LORA             0x01

/* SX1268 IRQ Masks */
#define SX1268_IRQ_TX_DONE                  (1 << 0)
#define SX1268_IRQ_RX_DONE                  (1 << 1)
#define SX1268_IRQ_PREAMBLE_DETECTED        (1 << 2)
#define SX1268_IRQ_SYNC_WORD_VALID          (1 << 3)
#define SX1268_IRQ_HEADER_VALID             (1 << 4)
#define SX1268_IRQ_HEADER_ERROR             (1 << 5)
#define SX1268_IRQ_CRC_ERROR                (1 << 6)
#define SX1268_IRQ_CAD_DONE                 (1 << 7)
#define SX1268_IRQ_CAD_DETECTED             (1 << 8)
#define SX1268_IRQ_TIMEOUT                  (1 << 9)
#define SX1268_IRQ_ALL                      0xFFFF

/* LoRa Status Codes */
#define LORA_OK                 0
#define LORA_ERROR              1
#define LORA_TIMEOUT            2
#define LORA_BUSY               3
#define LORA_CRC_ERROR          4
#define LORA_NO_DATA            5

/* LoRa Packet Structure */
typedef struct {
    uint8_t data[256];          // Packet payload
    uint8_t length;             // Payload length
    int16_t rssi;               // RSSI value
    int8_t snr;                 // SNR value
    bool crc_error;             // CRC error flag
} LoRa_Packet_t;

/* Function Prototypes */

/**
 * @brief Initialize LoRa module
 * @param hspi Pointer to SPI handle
 * @retval Status code (LORA_OK on success)
 */
uint8_t LoRa_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief Reset LoRa module
 * @retval None
 */
void LoRa_Reset(void);

/**
 * @brief Set LoRa module to receive mode
 * @retval Status code
 */
uint8_t LoRa_SetReceiveMode(void);

/**
 * @brief Set LoRa module to standby mode
 * @retval Status code
 */
uint8_t LoRa_SetStandbyMode(void);

/**
 * @brief Check if packet is available
 * @retval 1 if packet available, 0 otherwise
 */
uint8_t LoRa_PacketAvailable(void);

/**
 * @brief Read received packet
 * @param packet Pointer to packet structure
 * @retval Status code
 */
uint8_t LoRa_ReadPacket(LoRa_Packet_t *packet);

/**
 * @brief Get RSSI of last received packet
 * @retval RSSI in dBm
 */
int16_t LoRa_GetRSSI(void);

/**
 * @brief Get SNR of last received packet
 * @retval SNR in dB
 */
int8_t LoRa_GetSNR(void);

/**
 * @brief Get status of LoRa module
 * @retval Status byte
 */
uint8_t LoRa_GetStatus(void);

/**
 * @brief Clear IRQ flags
 * @param irq_mask IRQ mask to clear
 * @retval Status code
 */
uint8_t LoRa_ClearIRQ(uint16_t irq_mask);

/**
 * @brief Get IRQ status
 * @param irq_status Pointer to store IRQ status
 * @retval Status code
 */
uint8_t LoRa_GetIRQStatus(uint16_t *irq_status);

/**
 * @brief Clear IRQ status
 * @param irq_mask IRQ mask to clear
 * @retval Status code
 */
uint8_t LoRa_ClearIRQStatus(uint16_t irq_mask);

/**
 * @brief Get device status/mode
 * @param status Pointer to store status byte
 * @retval Status code
 */
uint8_t LoRa_GetDeviceStatus(uint8_t *status);

/**
 * @brief Transmit packet (for testing)
 * @param data Pointer to data buffer
 * @param length Length of data
 * @retval Status code
 */
uint8_t LoRa_Transmit(const uint8_t *data, uint8_t length);

/**
 * @brief Get total IRQ count (DIO1 interrupts fired)
 * @retval Number of times DIO1 interrupt has fired
 */
uint32_t LoRa_GetIRQCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
