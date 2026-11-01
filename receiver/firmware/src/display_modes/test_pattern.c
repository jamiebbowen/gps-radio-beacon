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
  /* Draw corner + centre markers to diagnose orientation issues. v2.0: the
   * SHARP panel is 168x144 with the 6x8 font => 28 cols x 18 rows; corners
   * are rows 0/17, and a 2-char label on the right edge starts at col 26. */
  Display_DrawTextRowCol(0, 0,  "TL");      /* Top-left */
  Display_DrawTextRowCol(17, 0, "BL");      /* Bottom-left */
  Display_DrawTextRowCol(0, 26, "TR");      /* Top-right */
  Display_DrawTextRowCol(17, 26, "BR");     /* Bottom-right */
  Display_DrawTextRowCol(8, 11, "CENTER");  /* Centre (6 chars, cols 11-16) */
}
