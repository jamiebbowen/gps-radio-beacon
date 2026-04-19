/**
 ******************************************************************************
 * @file           : sd_card_mode.c
 * @brief          : SD card display mode implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "display_modes/sd_card_mode.h"
#include "display.h"
#include "sd_card.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief Display SD card status and information
 * @retval None
 */
void DisplayMode_SDCard(void)
{
    char buffer[32];
    SD_Card_Info sd_info;
    SD_Card_Status status;
    
    /* Get SD card information */
    status = SD_Card_GetInfo(&sd_info);
    
    /* Display SD card presence */
    if (sd_info.is_present) {
        Display_DrawTextRowCol(0, 0, "SD Card: Present");
    } else {
        Display_DrawTextRowCol(0, 0, "SD Card: Not Found");
        Display_DrawTextRowCol(1, 0, "Insert SD card");
        return;
    }
    
    /* Display initialization status */
    if (sd_info.is_initialized && sd_info.is_mounted) {
        Display_DrawTextRowCol(1, 0, "Status: Ready");
        
        /* Display storage information */
        if (sd_info.total_size_mb >= 1024) {
            snprintf(buffer, sizeof(buffer), "Size: %luGB", sd_info.total_size_mb / 1024);
        } else if (sd_info.total_size_mb > 0) {
            snprintf(buffer, sizeof(buffer), "Size: %luMB", sd_info.total_size_mb);
        } else {
            snprintf(buffer, sizeof(buffer), "Size: ?");
        }
        Display_DrawTextRowCol(2, 0, buffer);

        if (sd_info.free_space_mb >= 1024) {
            snprintf(buffer, sizeof(buffer), "Free: %luGB", sd_info.free_space_mb / 1024);
        } else {
            snprintf(buffer, sizeof(buffer), "Free: %luMB", sd_info.free_space_mb);
        }
        Display_DrawTextRowCol(3, 0, buffer);
        
        /* Display logging statistics */
        snprintf(buffer, sizeof(buffer), "Files: %lu", sd_info.files_logged);
        Display_DrawTextRowCol(4, 0, buffer);
        
        /* Display bytes written this session */
        uint32_t bw = sd_info.bytes_written;
        if (bw >= 1024 * 1024) {
            snprintf(buffer, sizeof(buffer), "Wr: %luMB", bw / (1024 * 1024));
        } else if (bw >= 1024) {
            snprintf(buffer, sizeof(buffer), "Wr: %luKB", bw / 1024);
        } else {
            snprintf(buffer, sizeof(buffer), "Wr: %luB", bw);
        }
        Display_DrawTextRowCol(5, 0, buffer);
    } else {
        Display_DrawTextRowCol(1, 0, "Status: Error");
        
        /* Show error code (temporarily stored in write_errors field) */
        /* FRESULT codes: 1=DISK_ERR, 3=NOT_READY, 13=NO_FILESYSTEM, 99=DRIVER_LINK */
        snprintf(buffer, sizeof(buffer), "Err: %lu", sd_info.write_errors);
        Display_DrawTextRowCol(2, 0, buffer);
        
        /* Decode common errors */
        if (sd_info.write_errors == 1) {
            Display_DrawTextRowCol(3, 0, "DISK_ERR");
        } else if (sd_info.write_errors == 3) {
            Display_DrawTextRowCol(3, 0, "NOT_READY");
        } else if (sd_info.write_errors == 13) {
            Display_DrawTextRowCol(3, 0, "NO_FILESYSTEM");
        } else if (sd_info.write_errors == 99) {
            Display_DrawTextRowCol(3, 0, "DRIVER_LINK");
        } else {
            Display_DrawTextRowCol(3, 0, "UNKNOWN");
        }
        
        Display_DrawTextRowCol(4, 0, "Files: N/A");
        Display_DrawTextRowCol(5, 0, "Written: N/A");
    }
    
    /* Display CMD0 response for hardware debugging */
    snprintf(buffer, sizeof(buffer), "CMD0: 0x%02X", sd_info.cmd0_response);
    Display_DrawTextRowCol(6, 0, buffer);
    
    /* Check MISO pin state (should be HIGH when no SD card communication) */
    GPIO_PinState miso_state = HAL_GPIO_ReadPin(SD_SPI_MISO_GPIO_PORT, SD_SPI_MISO_PIN);
    GPIO_PinState detect_state = HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN);
    snprintf(buffer, sizeof(buffer), "MISO:%s DET:%s", 
             miso_state ? "H" : "L",
             detect_state ? "H" : "L");
    Display_DrawTextRowCol(7, 0, buffer);
}
