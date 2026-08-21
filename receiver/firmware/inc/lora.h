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

/* Multi-rocket channel plan: 8 channels at 250 kHz spacing, 433.00-434.75
 * MHz. Sits in the 70cm auxiliary/link segment (433-435 MHz): full
 * Technician privileges, clear of the 432-433 weak-signal/EME segment and
 * the 435-438 amateur-satellite segment. 250 kHz spacing leaves one full
 * signal bandwidth (125 kHz) of guard between adjacent channels, and all
 * channels sit in the same SX1268 image-calibration band (430-440 MHz), so
 * no recalibration is needed when hopping.
 * Each transmitter is flashed with one channel (LORA_CHANNEL, independent
 * of ROCKET_ID - any airframe can fly on any channel); the receiver retunes
 * at runtime via LoRa_SetChannel(). Must match the transmitter's plan in
 * transmitter/firmware/include/mpu_config.h. */
#define LORA_CHANNEL_COUNT      8
#define LORA_CHANNEL_BASE_MHZ   433.0f
#define LORA_CHANNEL_STEP_MHZ   0.25f
#define LORA_CHANNEL_FREQ_MHZ(ch) \
    (LORA_CHANNEL_BASE_MHZ + (float)(ch) * LORA_CHANNEL_STEP_MHZ)

/* LoRa Configuration - Match transmitter settings */
#define LORA_FREQUENCY_MHZ      LORA_CHANNEL_FREQ_MHZ(0)  // Boot channel (CH0)
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
/* TXEN: module pin NOT connected. Its former GPIO assignment (PB10) is the
 * mode button (see button.h) - never configure or drive PB10 from this
 * driver. RF switch = DIO2 automatic mode + wired RXEN. */
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

/* Channel Activity Detection (CAD): the modem samples the channel for a few
 * symbol periods and reports whether a LoRa preamble is on the air - a ~20 ms
 * sniff versus a 6.5 s listen, which is what makes a fast channel scan
 * possible. Detection parameters follow Semtech AN1200.48 for SF9/BW125.
 * Exit mode CAD_RX: on detection the chip drops straight into RX and captures
 * the packet whose preamble it just sniffed. */
#define LORA_CAD_SYMBOLS        0x02    /* 4 symbols (~16.4 ms at SF9/125) */
#define LORA_CAD_DET_PEAK       23      /* AN1200.48 recommendation, SF9 */
#define LORA_CAD_DET_MIN        10
#define LORA_CAD_EXIT_RX        0x01    /* enter RX on detection */
#define LORA_CAD_RX_TIMEOUT_MS  600     /* > max packet airtime at SF9/125 */

/* LoRa_CadResult() return values */
#define LORA_CAD_PENDING        0       /* CAD still running */
#define LORA_CAD_NONE           1       /* done: channel is quiet */
#define LORA_CAD_DETECTED       2       /* done: preamble heard, chip in RX */
#define LORA_CAD_FAIL           3       /* SPI/IRQ read failed */

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
 * @brief Read the instantaneous RSSI at the antenna (GetRssiInst, 0x15)
 * @note Only meaningful while the chip is in RX mode. Between packets this
 *       measures the ambient noise/interference level on the tuned channel.
 * @param rssi_dbm Output: signal level in dBm (negative)
 * @retval Status code
 */
uint8_t LoRa_GetRssiInst(int16_t *rssi_dbm);

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

/**
 * @brief Retune the radio to a different rocket channel and re-enter RX
 * @param channel Channel index (0..LORA_CHANNEL_COUNT-1)
 * @retval LORA_OK on success; on failure the previous channel is restored
 */
uint8_t LoRa_SetChannel(uint8_t channel);

/**
 * @brief Start a single Channel Activity Detection on the tuned channel
 * @note Exit mode is CAD_RX: if a preamble is detected the chip enters RX
 *       (for up to LORA_CAD_RX_TIMEOUT_MS) and receives the packet, firing
 *       DIO1 RX_DONE exactly like normal reception. If nothing is detected
 *       the chip falls back to standby. Poll LoRa_CadResult() to find out.
 * @retval LORA_OK if the CAD was started
 */
uint8_t LoRa_StartCad(void);

/**
 * @brief Poll the outcome of a CAD started with LoRa_StartCad()
 * @retval LORA_CAD_PENDING / LORA_CAD_NONE / LORA_CAD_DETECTED / LORA_CAD_FAIL
 * @note Clears the CAD IRQ bits once the result is read. On LORA_CAD_NONE
 *       the chip is in standby - the caller decides what mode comes next.
 */
uint8_t LoRa_CadResult(void);

/**
 * @brief Get the currently tuned rocket channel
 * @retval Channel index (0..LORA_CHANNEL_COUNT-1)
 */
uint8_t LoRa_GetChannel(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
