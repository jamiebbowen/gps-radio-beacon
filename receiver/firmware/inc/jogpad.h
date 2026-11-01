/**
 * @file jogpad.h
 * @brief NULLLAB/emakefun Mini I2C Jogpad (PH2.0 handle) driver
 *
 * Hardware: one X/Y joystick (0..255 per axis, 128 = center; X: left->right,
 * Y: top->bottom) and 5 buttons (A/B/C/D + joystick-click "Z"), all behind
 * one I2C slave at 0x5A (shares I2C1 with the BNO055).
 *
 * Register map (from the emakefun JoystickHandle reference driver):
 *   0x10 X axis, 0x11 Y axis
 *   0x20 Z (joystick click), 0x21 C ("down"), 0x22 A ("up"),
 *   0x23 B ("right"), 0x24 D ("left")
 * Button register states:
 *   0=down 1=up 2=repeat 3=single-click 4=double-click 5=long-press-start
 *   6=long-press-hold 8=none 0xFF=invalid
 * We only use "currently held" = (state != 8 && state != 0xFF); the app's
 * own edge/hysteresis logic (in button.c) is what generates page events.
 */

#ifndef JOGPAD_H
#define JOGPAD_H

#include <stdint.h>

typedef enum {
  JOGPAD_BTN_A = 0,   /* "up" button, reg 0x22 */
  JOGPAD_BTN_B,       /* "right", reg 0x23 */
  JOGPAD_BTN_C,       /* "down",  reg 0x21 */
  JOGPAD_BTN_D,       /* "left",  reg 0x24 */
  JOGPAD_BTN_Z,       /* joystick click, reg 0x20 */
  JOGPAD_BTN_COUNT
} Jogpad_Button;

uint8_t Jogpad_Init(void);          /* returns 1 if the pad ACKed at 0x5A */
void    Jogpad_Update(void);        /* throttled poll - call every main loop */
uint8_t Jogpad_IsPresent(void);

uint8_t Jogpad_GetX(void);          /* 0..255, 128 = centered */
uint8_t Jogpad_GetY(void);          /* 0..255, top = 0, bottom = 255 */
uint8_t Jogpad_ButtonHeld(Jogpad_Button b);  /* 1 = currently held */

#endif /* JOGPAD_H */
