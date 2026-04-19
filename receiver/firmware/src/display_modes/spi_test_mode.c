/**
 ******************************************************************************
 * @file           : spi_test_mode.c
 * @brief          : SPI hardware test mode for SD card debugging
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "display_modes/spi_test_mode.h"
#include "display.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/**
 * @brief Test SPI hardware functionality
 * @retval None
 */
void DisplayMode_SPITest(void)
{
    char buffer[32];
    uint8_t test_data = 0xAA;
    uint8_t received_data;
    HAL_StatusTypeDef spi_status;
    GPIO_PinState cs_state, miso_state;
    
    Display_Clear();
    Display_DrawTextRowCol(0, 0, "SPI Hardware Test");
    
    /* Test CS pin control */
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    cs_state = HAL_GPIO_ReadPin(SD_CS_GPIO_PORT, SD_CS_PIN);
    snprintf(buffer, sizeof(buffer), "CS High: %s", cs_state ? "OK" : "FAIL");
    Display_DrawTextRowCol(1, 0, buffer);
    
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    cs_state = HAL_GPIO_ReadPin(SD_CS_GPIO_PORT, SD_CS_PIN);
    snprintf(buffer, sizeof(buffer), "CS Low: %s", cs_state ? "FAIL" : "OK");
    Display_DrawTextRowCol(2, 0, buffer);
    
    /* Test MISO pin (should be pulled high when no device) */
    miso_state = HAL_GPIO_ReadPin(SD_SPI_MISO_GPIO_PORT, SD_SPI_MISO_PIN);
    snprintf(buffer, sizeof(buffer), "MISO: %s", miso_state ? "HIGH" : "LOW");
    Display_DrawTextRowCol(3, 0, buffer);
    
    /* Test SPI communication */
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET); /* CS Low */
    spi_status = HAL_SPI_TransmitReceive(&hspi1, &test_data, &received_data, 1, 1000);
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET); /* CS High */
    
    if (spi_status == HAL_OK) {
        snprintf(buffer, sizeof(buffer), "SPI: OK");
        Display_DrawTextRowCol(4, 0, buffer);
        snprintf(buffer, sizeof(buffer), "Sent: 0x%02X", test_data);
        Display_DrawTextRowCol(5, 0, buffer);
        snprintf(buffer, sizeof(buffer), "Recv: 0x%02X", received_data);
        Display_DrawTextRowCol(6, 0, buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "SPI: FAIL (%d)", spi_status);
        Display_DrawTextRowCol(4, 0, buffer);
    }
    
    /* Test voltage level indication */
    Display_DrawTextRowCol(7, 0, "Check 3.3V supply");
}
