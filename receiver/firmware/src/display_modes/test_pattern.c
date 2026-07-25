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
  /* Draw corner + centre markers to diagnose orientation issues.
   * Display is 128x64 with a 6x8 font => 21 cols x 8 rows, so the corners
   * are rows 0/7 and cols 0/19 (2-char labels occupy cols 19-20). */
  Display_DrawTextRowCol(0, 0,  "TL");     /* Top-left */
  Display_DrawTextRowCol(7, 0,  "BL");     /* Bottom-left */
  Display_DrawTextRowCol(0, 19, "TR");     /* Top-right */
  Display_DrawTextRowCol(7, 19, "BR");     /* Bottom-right */
  Display_DrawTextRowCol(3, 7,  "CENTER"); /* Centre (6 chars, cols 7-12) */
}
