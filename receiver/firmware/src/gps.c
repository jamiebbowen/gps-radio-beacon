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

/* Function declarations */
void Error_Handler(void);

/* Private variables */
static UART_HandleTypeDef huart_gps;
static uint8_t gps_rx_data;
static uint8_t gps_nmea_buffer[GPS_BUFFER_SIZE];
static uint16_t gps_nmea_index = 0;
static uint8_t gps_nmea_ready = 0;
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
static void GPS_UART_Init(void);

/**
 * @brief Initialize GPS module
 * @retval Status code
 */
uint8_t GPS_Init(void)
{
  /* Initialize UART for GPS communication */
  GPS_UART_Init();
  
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
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "UR:%lu", uart_byte_counter);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "NM:%lu", nmea_sentence_counter);
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
      
      /* Cycle through different parts of the buffer */
      if (debug_counter == 0) {
        raw_display_offset = (raw_display_offset + 6) % (RAW_CAPTURE_SIZE - 6);
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
      /* Display NMEA buffer state */
      snprintf(gps_data->debug_lat, GPS_DEBUG_BUFFER_SIZE, "IDX:%d", gps_nmea_index);
      snprintf(gps_data->debug_lon, GPS_DEBUG_BUFFER_SIZE, "RDY:%d", gps_nmea_ready);
      if (gps_nmea_index > 0 && gps_nmea_index < GPS_BUFFER_SIZE) {
        /* Show first few chars of NMEA buffer if available */
        snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "%c%c%c%c", 
                 gps_nmea_buffer[0], 
                 gps_nmea_index > 1 ? gps_nmea_buffer[1] : ' ',
                 gps_nmea_index > 2 ? gps_nmea_buffer[2] : ' ',
                 gps_nmea_index > 3 ? gps_nmea_buffer[3] : ' ');
      } else {
        snprintf(gps_data->debug_sats, GPS_DEBUG_BUFFER_SIZE, "EMPTY");
      }
      break;
  }
  
  /* Process any complete NMEA sentences */
  if (gps_nmea_ready) {
    /* Copy debug bytes to show we're in GPS_Update */
    last_bytes[0] = 0xB1;
    last_bytes[1] = 0xB2;
    
    /* Use the parser module to parse NMEA sentences */
    if (GPS_ParseNMEA((char*)gps_nmea_buffer, gps_data) == GPS_PARSER_OK) {
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
    
    gps_nmea_ready = 0;
    gps_nmea_index = 0;
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
  /* Implementation depends on the current state of gps_data */
  /* For now, we'll just return 1 (fixed) for testing */
  return 1;
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
    static uint8_t uart_error_count = 0;
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
      gps_nmea_index = 0;
      gps_nmea_buffer[gps_nmea_index++] = gps_rx_data;
    }
    /* Add character to buffer if we're inside an NMEA sentence */
    else if (gps_nmea_index > 0 && gps_nmea_index < GPS_BUFFER_SIZE - 1) {
      gps_nmea_buffer[gps_nmea_index++] = gps_rx_data;
      
      /* Check for CR character */
      if (gps_rx_data == NMEA_END_CHAR1) {
        cr_count++;
      }
      /* Check for LF character and if we have a complete sentence */
      else if (gps_rx_data == NMEA_END_CHAR2) {
        lf_count++;
        
        /* Null-terminate the buffer */
        gps_nmea_buffer[gps_nmea_index] = '\0';
        
        /* Check if we have a complete NMEA sentence */
        if (gps_nmea_index > 2 && 
            gps_nmea_buffer[gps_nmea_index-2] == NMEA_END_CHAR1 && 
            gps_nmea_buffer[0] == NMEA_START_CHAR) {
          /* We have a complete sentence */
          gps_nmea_ready = 1;
          nmea_sentence_counter++;
        }
      }
    }
    /* Buffer overflow - reset */
    else if (gps_nmea_index >= GPS_BUFFER_SIZE - 1) {
      gps_nmea_index = 0;
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
 * @brief Initialize UART for GPS communication
 * @retval None
 */
static void GPS_UART_Init(void)
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
    /* Error handling */
    Error_Handler();
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
}
