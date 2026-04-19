/**
 ******************************************************************************
 * @file           : test_pattern.c
 * @brief          : Test pattern display implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "display_modes/test_pattern.h"
#include "display.h"

/**
 * @brief Display test pattern on screen
 * @retval None
 */
void Display_ShowTestPattern(void)
{
  /* Draw test pattern to diagnose rotation issues - adjusted for 64x128 display with 270° rotation */
  Display_DrawTextRowCol(0, 0, "TL");      /* Top-left */
  Display_DrawTextRowCol(15, 0, "BL");    /* Bottom-left */
  Display_DrawTextRowCol(0, 8, "TR");     /* Top-right, adjusted position */
  Display_DrawTextRowCol(15, 8, "BR");   /* Bottom-right, adjusted position */
  Display_DrawTextRowCol(7, 4, "CENTER"); /* Center, adjusted position */
}
