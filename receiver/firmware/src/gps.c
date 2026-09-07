/**
 * @file gps.c
 * @brief GPS module driver implementation for NEO6M
 */

#include "gps.h"
#include "gps_parser.h"
#include "stm32f4xx_hal_uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private defines */
#define GPS_UART                    USART2
#define GPS_UART_BAUD               9600  /* Standard 9600 baud (HSE_VALUE now correctly set to 25MHz) */
#define GPS_BUFFER_SIZE             128
#define NMEA_START_CHAR             '$'
#define NMEA_END_CHAR1              '\r'
#define NMEA_END_CHAR2              '\n'


/* Private variables */
static UART_HandleTypeDef huart_gps;
static uint8_t gps_rx_data;

/* Sentence plumbing: the ISR assembles into gps_isr_buffer and, on
 * completion, publishes into a small FIFO that GPS_Update pops and parses.
 *
 * Why not a single ready-flag slot? The main loop periodically stalls
 * ~20-80 ms writing the SSD1306 frame over I2C at 4 Hz - comparable to the
 * inter-sentence gap in a 9600-baud burst - and a single slot then kept
 * only the LAST sentence of a colliding pair, silently dropping the other
 * (GGA or RMC, the only two the parser accepts). Freshly-locked fixes
 * becoming intermittently invisible presented as "slow GPS acquisition".
 * A depth-2 FIFO loses nothing through a lone stall and keeps the last two
 * sentences through longer ones.
 *
 * Why still not parse in the ISR? GPS_ParseNMEA does strtok_r, float math
 * and debug snprintf - far too heavy for interrupt context. */
/* Ring slots = usable depth + 1: full is (head+1)%Q == tail, so one slot is
 * sacrificed to keep "full" unambiguous against "empty". Depth 2 matches
 * the display-stall analysis above (two sentences can complete inside one
 * I2C frame write; longer stalls keep the two newest). */
#define GPS_PENDING_Q 3
static uint8_t gps_isr_buffer[GPS_BUFFER_SIZE];
static volatile uint16_t gps_isr_index = 0;
static uint8_t gps_pending[GPS_PENDING_Q][GPS_BUFFER_SIZE];
static volatile uint8_t gps_pend_head = 0;   /* next slot the ISR publishes to */
static volatile uint8_t gps_pend_tail = 0;   /* next slot GPS_Update parses  */
static uint32_t gps_sentences_dropped = 0;   /* completions with all slots busy */

static uint8_t gps_last_fix = 0;   /* Fix state from the last parsed sentence */

/* Fix-staleness gate: if the module dies (antenna unplugged, UART wedge,
 * brown-out) sentences stop entirely, and nothing ever expired the latch
 * above - GPS_IsFixed() kept returning 1 forever while main.c re-copied
 * frozen coordinates into local_gps_data every loop iteration, presenting
 * a ghost "L:Fix" until the next power cycle. */
static uint32_t gps_last_sentence_ms = 0;
#define GPS_FIX_STALE_MS 5000u   /* 5x the 1 Hz sentence period */

static uint16_t uart_error_count = 0;   /* HAL error-path hits (ORE/FE/NE) */
static uint8_t last_bytes[4] = {0}; /* Debug: Store last 4 raw bytes */
static uint32_t uart_byte_counter = 0; /* Counter for UART activity */
static uint32_t nmea_sentence_counter = 0; /* Counter for complete NMEA sentences */

/* Debug variables for NMEA sentence detection */
static uint8_t first_10_bytes[10] = {0}; /* Store first 10 bytes received */
static uint8_t first_10_bytes_filled = 0; /* Flag to indicate if first_10_bytes is filled */
static uint8_t last_10_bytes[10] = {0};   /* Circular buffer for last 10 bytes */
static uint8_t last_10_index = 0;         /* Current index in circular buffer */
static uint8_t dollar_sign_count = 0;     /* Count of '$' characters received */
static uint8_t cr_count = 0;              /* Count of '\r' characters received */
static uint8_t lf_count = 0;              /* Count of '\n' characters received */

/* Raw byte capture for direct analysis */
#define RAW_CAPTURE_SIZE 64
static uint8_t raw_capture_buffer[RAW_CAPTURE_SIZE] = {0};
static uint8_t raw_capture_index = 0;
static uint8_t raw_capture_filled = 0;

/* Private function prototypes */
static uint8_t GPS_UART_Init(void);
static void    GPS_SendUBXCfgMsg(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

/**
 * @brief Initialize GPS module
 * @retval Status code
 */
uint8_t GPS_Init(void)
{
  /* Initialize UART for GPS communication. Failure is reported, not
   * escalated: local GPS is optional (distance/bearing only) and main
   * degrades gracefully when GPS_Init returns GPS_ERROR. Calling
   * Error_Handler here would reset the whole receiver over a peripheral
   * the mission can live without. */
  if (GPS_UART_Init() != GPS_OK) {
    return GPS_ERROR;
  }
  
  /* Set a non-zero value in last_bytes for debugging */
  last_bytes[0] = 0xA1;
  last_bytes[1] = 0xB2;
  last_bytes[2] = 0xC3;
  last_bytes[3] = 0xD4;
  
  /* Initialize local GPS data structure with safe defaults */
  extern GPS_Data local_gps_data;
  memset(&local_gps_data, 0, sizeof(GPS_Data));
  strcpy(local_gps_data.debug_lat, "NO_DATA");
  strcpy(local_gps_data.debug_lon, "NO_DATA");
  strcpy(local_gps_data.debug_sats, "0");
  
  /* Start receiving data from GPS module */
  if (HAL_UART_Receive_IT(&huart_gps, &gps_rx_data, 1) != HAL_OK) {
    /* If UART receive fails, set error pattern in last_bytes */
    last_bytes[0] = 0xE1;
    last_bytes[1] = 0xE2;
    last_bytes[2] = 0xE3;
    last_bytes[3] = 0xE4;
    return GPS_ERROR;
  }

  /* Shape the module's message set. NEO-6M-era modules default to the full
   * NMEA suite (GGA/GLL/GSA/GSV/RMC/VTG each 1 Hz; GSV scales with visible
   * satellites) - over 400 bytes of burst every second on a 9600-baud
   * (960 B/s) line, which was observed bursting off in the field (GSV
   * outright absent at the park). We only consume GGA and RMC; disabling
   * the rest frees the line and moves RMC delivery ~300 ms earlier in the
   * burst (the compass motion-calibration gate looks at RMC speed).
   * Legacy UBX-CFG-MSG set-rates - valid on u-blox 6/7/8-class receivers
   * like this NEO-6M (the TX's M10 has its own VALSET path); a non-ublox
   * module simply ignores these. No response frames are generated for
   * CFG-MSG, so nothing floods the RX side. RAM-only, re-sent each boot. */
  GPS_SendUBXCfgMsg(0xF0, 0x03, 0);   /* GSV off                     */
  GPS_SendUBXCfgMsg(0xF0, 0x01, 0);   /* GLL off                     */
  GPS_SendUBXCfgMsg(0xF0, 0x05, 0);   /* VTG off                     */
  GPS_SendUBXCfgMsg(0xF0, 0x02, 0);   /* GSA off                     */
  GPS_SendUBXCfgMsg(0xF0, 0x04, 1);   /* RMC every nav solution      */
  GPS_SendUBXCfgMsg(0xF0, 0x00, 1);   /* GGA every nav solution      */

  return GPS_OK;
}

/**
 * @brief Update GPS data from module
 * @param gps_data Pointer to GPS data structure
 * @retval Status code
 */
uint8_t GPS_Update(GPS_Data *gps_data)
{
  uint8_t status = GPS_ERROR;
  
  /* Rotate through different debug information displays more slowly */
  static uint8_t debug_counter = 0;
  static uint8_t debug_mode = 0;
  
  debug_counter++;
  if (debug_counter >= 5) { /* Change display every 5 calls (slower rotation) */
    debug_counter = 0;
    debug_mode = (debug_mode + 1) % 6; /* Now 6 debug modes */
  }
  
  switch (debug_mode) {
    case 0:
      /* Display UART and NMEA counters */
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "UR:%lu", (unsigned long)uart_byte_counter);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "NM:%lu", (unsigned long)nmea_sentence_counter);
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "ST:%02X ER:%02X", last_bytes[2], last_bytes[3]);
      break;
      
    case 1:
      /* Display special character counts */
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "$:%u", dollar_sign_count);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "CR:%u", cr_count);
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "LF:%u", lf_count);
      break;
      
    case 2:
      /* Display first few bytes as hex */
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "%02X %02X", 
               first_10_bytes[0], first_10_bytes[1]);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "%02X %02X", 
               first_10_bytes[2], first_10_bytes[3]);
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "%02X %02X", 
               first_10_bytes[4], first_10_bytes[5]);
      break;
      
    case 3:
      /* Display raw captured bytes as hex in a pattern */
      static uint8_t raw_display_offset = 0;
      
      /* Cycle through different parts of the buffer. The hex dump below
       * reads raw_capture_buffer[offset .. offset+7], so the offset must
       * stay <= RAW_CAPTURE_SIZE - 8 (modulus of SIZE-7 gives max SIZE-8). */
      if (debug_counter == 0) {
        raw_display_offset = (raw_display_offset + 6) % (RAW_CAPTURE_SIZE - 7);
      }
      
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "%02X%02X%02X%02X", 
               raw_capture_buffer[raw_display_offset], 
               raw_capture_buffer[raw_display_offset+1],
               raw_capture_buffer[raw_display_offset+2],
               raw_capture_buffer[raw_display_offset+3]);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "%02X%02X%02X%02X", 
               raw_capture_buffer[raw_display_offset+4],
               raw_capture_buffer[raw_display_offset+5],
               raw_capture_buffer[raw_display_offset+6],
               raw_capture_buffer[raw_display_offset+7]);
      snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "O:%d", 
               raw_display_offset);
      break;
      
    case 4:
      {
        /* Display sentence-queue state: ISR fill position, queued depth,
         * drops (queue full = main-loop stalls outrunning two slots) */
        uint8_t pend_depth = (uint8_t)((gps_pend_head - gps_pend_tail) % GPS_PENDING_Q);
        snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "IDX:%d", gps_isr_index);
        snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "Q:%d D:%lu",
                 pend_depth, (unsigned long)(gps_sentences_dropped % 1000));
        if (gps_isr_index > 0 && gps_isr_index < GPS_BUFFER_SIZE) {
          /* Show first few chars of the sentence being assembled */
          char c0 = gps_isr_buffer[0];
          char c1 = (gps_isr_index > 1) ? gps_isr_buffer[1] : ' ';
          char c2 = (gps_isr_index > 2) ? gps_isr_buffer[2] : ' ';
          char c3 = (gps_isr_index > 3) ? gps_isr_buffer[3] : ' ';
          snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "%c%c%c%c", c0, c1, c2, c3);
        } else {
          snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "EMPTY");
        }
      }
      break;
  }
  
  /* Parse every queued sentence. A lone display-frame stall can complete
   * two sentences between Update calls; the FIFO keeps both (see the
   * plumbing comment at the top). */
  while (gps_pend_tail != gps_pend_head) {
    /* Copy debug bytes to show we're in GPS_Update */
    last_bytes[0] = 0xB1;
    last_bytes[1] = 0xB2;

    /* Use the parser module to parse NMEA sentences. The parser writes
     * gps_data->fix from the actual sentence (GGA quality / RMC A|V) even
     * when it returns PARSER_ERROR for a no-fix sentence, so this latch
     * always reflects the latest on-air fix state. */
    uint8_t parse_result = GPS_ParseNMEA((char*)gps_pending[gps_pend_tail], gps_data);
    gps_pend_tail = (uint8_t)((gps_pend_tail + 1) % GPS_PENDING_Q);
    gps_last_sentence_ms = HAL_GetTick();   /* stream is alive */
    gps_last_fix = gps_data->fix ? 1 : 0;
    if (parse_result == GPS_PARSER_OK) {
      /* Don't override fix status - it's set by the parser based on actual GPS data */
      status = GPS_OK;

      /* Update debug bytes to show successful parsing */
      last_bytes[2] = 0xB3;
      last_bytes[3] = 0xB4;
    } else {
      /* Update debug bytes to show parsing failed */
      last_bytes[2] = 0xB5;
      last_bytes[3] = 0xB6;
    }
  }
  
  /* Start UART receive interrupt if not already running */
  if (huart_gps.RxState == HAL_UART_STATE_READY) {
    if (HAL_UART_Receive_IT(&huart_gps, &gps_rx_data, 1) != HAL_OK) {
      /* Error re-arming interrupt */
      last_bytes[0] = 0xE1;
      last_bytes[1] = 0xE2;
      last_bytes[2] = 0xE3;
      last_bytes[3] = 0xE4;
    }
  }
  
  return status;
}

/**
 * @brief Check if GPS has a valid fix
 * @retval 1 if fixed, 0 otherwise
 */
uint8_t GPS_IsFixed(void)
{
  /* Real fix state from the last parsed sentence - NOT a stub. main.c ORs
   * this into the local-GPS validity condition, so returning a constant 1
   * here used to make stale coordinates (last-known position with fix=0)
   * pass as a current fix whenever they were non-zero and in range.
   *
   * Same for a DEAD stream: when sentences stop entirely (module wedged,
   * antenna unplugged), expire the latch within a few seconds instead of
   * showing a ghost fix until reboot. */
  if (HAL_GetTick() - gps_last_sentence_ms > GPS_FIX_STALE_MS) {
    return 0;
  }
  return gps_last_fix;
}

/**
 * @brief USART2 IRQ Handler for GPS module
 * @retval None
 */
void USART2_IRQHandler(void)
{
  /* Set debug marker to indicate ISR entry */
  last_bytes[0] = 0xAA;
  last_bytes[1] = 0xBB;
  last_bytes[2] = 0xCC;
  last_bytes[3] = 0xDD;
  
  /* Call HAL UART IRQ Handler */
  HAL_UART_IRQHandler(&huart_gps);
}

/**
 * @brief HAL UART Receive Complete Callback
 * @param huart UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Handle GPS UART (USART2) */
  if (huart->Instance == GPS_UART) {
    /* Set a marker in last_bytes to indicate callback was called */
    last_bytes[0] = 0xF1;
    
    /* Store raw byte for display */
    last_bytes[1] = gps_rx_data;
    last_bytes[2] = huart->RxState;
    last_bytes[3] = huart->ErrorCode;
    
    /* Capture raw bytes regardless of errors for direct analysis */
    if (!raw_capture_filled) {
      raw_capture_buffer[raw_capture_index++] = gps_rx_data;
      if (raw_capture_index >= RAW_CAPTURE_SIZE) {
        raw_capture_filled = 1;
      }
    }
    
    /* Track UART state and error codes for debugging */
    if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
      uart_error_count++;
      
      /* Clear error flags to prevent lockup */
      __HAL_UART_CLEAR_PEFLAG(huart);
      __HAL_UART_CLEAR_FEFLAG(huart);
      __HAL_UART_CLEAR_NEFLAG(huart);
      __HAL_UART_CLEAR_OREFLAG(huart);
      
      /* Reject bytes received with framing errors by skipping the rest of processing */
      /* Continue receiving next byte */
      if (HAL_UART_Receive_IT(huart, &gps_rx_data, 1) != HAL_OK) {
        last_bytes[0] = 0xF2; /* Error re-arming interrupt */
      }
      return; /* Skip processing this byte */
    }
    
    /* Increment UART byte counter for every byte received */
    uart_byte_counter++;
    
    /* Store first 10 bytes for debugging */
    if (!first_10_bytes_filled && uart_byte_counter <= 10) {
      first_10_bytes[uart_byte_counter-1] = gps_rx_data;
      if (uart_byte_counter == 10) {
        first_10_bytes_filled = 1;
      }
    }
    
    /* Store last 10 bytes in circular buffer */
    last_10_bytes[last_10_index] = gps_rx_data;
    last_10_index = (last_10_index + 1) % 10;
    
    /* Count special characters for debugging */
    if (gps_rx_data == NMEA_START_CHAR) { /* '$' */
      dollar_sign_count++;

      /* Reset buffer when we see a start character */
      gps_isr_index = 0;
      gps_isr_buffer[gps_isr_index++] = gps_rx_data;
    }
    /* Add character to buffer if we're inside an NMEA sentence */
    else if (gps_isr_index > 0 && gps_isr_index < GPS_BUFFER_SIZE - 1) {
      gps_isr_buffer[gps_isr_index++] = gps_rx_data;

      /* Check for CR character */
      if (gps_rx_data == NMEA_END_CHAR1) {
        cr_count++;
      }
      /* Check for LF character and if we have a complete sentence */
      else if (gps_rx_data == NMEA_END_CHAR2) {
        lf_count++;

        /* Check if we have a complete NMEA sentence */
        if (gps_isr_index > 2 &&
            gps_isr_buffer[gps_isr_index-2] == NMEA_END_CHAR1 &&
            gps_isr_buffer[0] == NMEA_START_CHAR) {
          /* We have a complete sentence: publish it into the pending FIFO
           * (see the plumbing comment at the top). If the FIFO is full -
           * two+ sentences completed inside one main-loop stall - drop the
           * OLDEST is wrong here: keep the newer sentences (position data is
           * freshest at the head), so drop the incoming one instead. */
          uint8_t next = (uint8_t)((gps_pend_head + 1) % GPS_PENDING_Q);
          if (next == gps_pend_tail) {
            gps_sentences_dropped++;
          } else {
            memcpy(gps_pending[gps_pend_head], gps_isr_buffer, gps_isr_index);
            gps_pending[gps_pend_head][gps_isr_index] = '\0';
            gps_pend_head = next;
            nmea_sentence_counter++;
          }
          gps_isr_index = 0;
        }
      }
    }
    /* Buffer overflow - reset */
    else if (gps_isr_index >= GPS_BUFFER_SIZE - 1) {
      gps_isr_index = 0;
    }
    
  /* Continue receiving - if this fails, update last_bytes to indicate error */
  if (HAL_UART_Receive_IT(huart, &gps_rx_data, 1) != HAL_OK) {
    last_bytes[0] = 0xF2; /* Error re-arming interrupt */
  }
  }
  /* Note: RF receiver now uses LoRa (SPI), not UART */
}

/**
 * @brief Get the last 4 raw bytes received from GPS
 * @param bytes Array to store the bytes (must be at least 4 bytes)
 * @retval None
 */
void GPS_GetRawBytes(uint8_t *bytes)
{
  if (bytes != NULL) {
    for (int i = 0; i < 4; i++) {
      bytes[i] = last_bytes[i];
    }
  }
}

/**
 * @brief Get GPS receive statistics (UART bytes, complete sentences,
 *        queue drops, UART error-path hits). All pointers optional.
 * @retval None
 */
void GPS_GetRxStats(uint32_t *bytes, uint32_t *sentences,
                    uint32_t *dropped, uint16_t *uart_errors)
{
  if (bytes)       *bytes       = uart_byte_counter;
  if (sentences)   *sentences   = nmea_sentence_counter;
  if (dropped)     *dropped     = gps_sentences_dropped;
  if (uart_errors) *uart_errors = uart_error_count;
}

/**
 * @brief Send a legacy UBX-CFG-MSG "set rate" frame (u-blox 6/7/8)
 * @param msg_class Message class (0xF0 = NMEA standard)
 * @param msg_id    Message ID (0=GGA, 1=GLL, 2=GSA, 3=GSV, 4=RMC, 5=VTG)
 * @param rate      Rate on UART1 in nav solutions (0=off, 1=every solution)
 * @retval None
 */
static void GPS_SendUBXCfgMsg(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  /* Payload: msgClass, msgID, rate[6] per port
   * (0=DDC/I2C, 1=UART1, 2=UART2, 3=USB, 4=SPI, 5=reserved). Ports we don't
   * wire up are explicitly set to 0 - the set-rate form is all-or-nothing,
   * there is no "leave unchanged" in the legacy message. */
  uint8_t frame[16] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
                        0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00 };
  frame[6] = msg_class;
  frame[7] = msg_id;
  frame[9] = rate;   /* UART1 */
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 14; i++) {
    ck_a = (uint8_t)(ck_a + frame[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  frame[14] = ck_a;
  frame[15] = ck_b;
  (void)HAL_UART_Transmit(&huart_gps, frame, sizeof(frame), 100);
}

/**
 * @brief Initialize UART for GPS communication
 * @retval GPS_OK or GPS_ERROR
 */
static uint8_t GPS_UART_Init(void)
{
  /* Enable USART2 clock */
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure GPIO pins for UART */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* USART2 GPIO Configuration    
     PA2  --> USART2_TX
     PA3 --> USART2_RX 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  /* Add pull-up to improve signal integrity */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Configure UART */
  huart_gps.Instance = USART2;
  huart_gps.Init.BaudRate = GPS_UART_BAUD;
  huart_gps.Init.WordLength = UART_WORDLENGTH_8B;
  huart_gps.Init.StopBits = UART_STOPBITS_1;
  huart_gps.Init.Parity = UART_PARITY_NONE;
  huart_gps.Init.Mode = UART_MODE_TX_RX;
  huart_gps.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_gps.Init.OverSampling = UART_OVERSAMPLING_16;
  
  /* De-initialize first in case it was already initialized */
  HAL_UART_DeInit(&huart_gps);
  
  /* Explicitly disable the USART2 IRQ before initialization */
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  
  if (HAL_UART_Init(&huart_gps) != HAL_OK) {
    return GPS_ERROR;
  }
  
  /* Clear any pending interrupts */
  __HAL_UART_CLEAR_PEFLAG(&huart_gps);
  
  /* Configure NVIC for USART2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0); /* Set to lower priority than RF receiver UART */
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  
  /* Enable UART receive interrupt explicitly */
  SET_BIT(huart_gps.Instance->CR1, USART_CR1_RXNEIE);
  
  /* Ensure global interrupts are enabled */
  __enable_irq();

  return GPS_OK;
}
