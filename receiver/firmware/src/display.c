/**
 * @file display.c
 * @brief Display driver implementation for SSD1309 OLED
 */

#include "display.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "font.h"

/* Private defines */
#define SSD1309_I2C                  I2C1
#define SSD1309_I2C_ADDR             0x3C
#define SSD1309_I2C_TIMEOUT          5  /* Short timeout to prevent blocking LoRa RX */
#define SSD1309_COMMAND              0x00
#define SSD1309_DATA                 0x40
#define SSD1309_BUFFER_SIZE          (DISPLAY_HW_WIDTH * DISPLAY_HW_HEIGHT / 8) /* 128*64/8 = 1024 bytes */

/* Private variables */
I2C_HandleTypeDef hi2c_display; /* Global for sharing with compass module */
static uint8_t display_buffer[SSD1309_BUFFER_SIZE];
static uint8_t rotated_buffer[SSD1309_BUFFER_SIZE]; /* Buffer for 90 degree fixed orientation */
static const uint8_t ssd1309_init_sequence[] = {
  SSD1309_DISPLAYOFF,
  SSD1309_SETDISPLAYCLOCKDIV, 0x80,
  SSD1309_SETMULTIPLEX, 0x3F,
  SSD1309_SETDISPLAYOFFSET, 0x00,
  SSD1309_SETSTARTLINE | 0x00,
  SSD1309_CHARGEPUMP, 0x14,
  SSD1309_MEMORYMODE, 0x00,
  SSD1309_SEGREMAP | 0x01,
  SSD1309_COMSCANDEC,
  SSD1309_SETCOMPINS, 0x12,
  SSD1309_SETCONTRAST, 0xCF,
  SSD1309_SETPRECHARGE, 0xF1,
  SSD1309_SETVCOMDETECT, 0x40,
  SSD1309_DISPLAYALLON_RESUME,
  SSD1309_NORMALDISPLAY,
  SSD1309_DISPLAYON
};

/* Private function prototypes */
static void Display_I2C_Init(void);
static void Display_SendCommand(uint8_t command);
static void Display_SendData(uint8_t* data, uint16_t size);
static void Display_SetPosition(uint8_t x, uint8_t y);
static void Display_RotateBuffer(uint8_t *source, uint8_t *dest);

/**
 * @brief Initialize display
 * @retval Status code
 */
uint8_t Display_Init(void)
{
  /* Initialize I2C for display communication */
  Display_I2C_Init();
  
  /* Send initialization sequence */
  for (uint8_t i = 0; i < sizeof(ssd1309_init_sequence); i++) {
    Display_SendCommand(ssd1309_init_sequence[i]);
  }
  
  /* Set 180° rotated orientation hardware configuration */
  Display_SendCommand(SSD1309_SEGREMAP | 0x01);  /* Flip horizontal (segment remap) */
  Display_SendCommand(SSD1309_COMSCANDEC);       /* Flip vertical (reverse COM scan) */
  
  /* Clear display buffer */
  Display_Clear();
  Display_Update();
  
  return DISPLAY_OK;
}

/**
 * @brief Clear display buffer
 * @retval None
 */
void Display_Clear(void)
{
  memset(display_buffer, 0, SSD1309_BUFFER_SIZE);
}

/**
 * @brief Update display with buffer contents
 * @retval None
 */
void Display_Update(void)
{
  /* Always rotate buffer for fixed 90° orientation */
  // Display_RotateBuffer(display_buffer, rotated_buffer);
  
  /* Set column address range */
  Display_SendCommand(SSD1309_COLUMNADDR);
  Display_SendCommand(0); /* Start column */
  Display_SendCommand(DISPLAY_HW_WIDTH - 1); /* End column */
  
  /* Set page address range */
  Display_SendCommand(SSD1309_PAGEADDR);
  Display_SendCommand(0);
  Display_SendCommand((DISPLAY_HW_HEIGHT / 8) - 1); /* Use hardware height in pages */
  
  /* Send the rotated buffer */
  Display_SendData(display_buffer, SSD1309_BUFFER_SIZE);
}

/**
 * @brief Rotate the display buffer for fixed 90 degree orientation
 * @param source Source buffer
 * @param dest Destination buffer
 * @retval None
 */
static void Display_RotateBuffer(uint8_t *source, uint8_t *dest)
{
  /* Clear destination buffer */
  memset(dest, 0, SSD1309_BUFFER_SIZE);
  
  /* This is a bit-by-bit transformation for 90° rotation with correction for mirroring */
  for (uint16_t x = 0; x < DISPLAY_HW_WIDTH; x++) {
    for (uint8_t y = 0; y < DISPLAY_HW_HEIGHT; y++) {
      /* Calculate source byte position and bit position */
      uint16_t src_byte_pos = x + (y / 8) * DISPLAY_HW_WIDTH;
      uint8_t src_bit_pos = y % 8;
      
      /* Check if this bit is set in the source buffer */
      uint8_t pixel = (source[src_byte_pos] >> src_bit_pos) & 0x01;
      
      /* If pixel is set, calculate destination position with corrected orientation */
      if (pixel) {
        /* For 90° rotation and flip both X and Y to correct orientation: (x,y) -> (y, DISPLAY_HW_HEIGHT-1-x) */
        uint16_t dest_x = y;
        uint16_t dest_y = DISPLAY_HW_HEIGHT - 1 - x;
        
        /* Ensure destination coordinates are within bounds */
        if (dest_x < DISPLAY_HW_WIDTH && dest_y < DISPLAY_HW_HEIGHT) {
          uint16_t dest_byte_pos = dest_x + (dest_y / 8) * DISPLAY_HW_WIDTH;
          uint8_t dest_bit_pos = dest_y % 8;
          
          /* Ensure we don't write outside the buffer */
          if (dest_byte_pos < SSD1309_BUFFER_SIZE) {
            /* Set the bit in the destination buffer */
            dest[dest_byte_pos] |= (1 << dest_bit_pos);
          }
        }
      }
    }
  }
}

/**
 * @brief Draw a single pixel
 * @param x X coordinate (in logical display space)
 * @param y Y coordinate (in logical display space)
 * @param color Pixel color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
  /* Check if the coordinates are within bounds */
  if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) {
    return;
  }
  
  /* Calculate buffer position using hardware coordinates */
  /* For 90° fixed orientation, we draw directly to the buffer */
  /* The rotation to hardware coordinates happens in Display_RotateBuffer */
  uint16_t byte_pos = x + (y / 8) * DISPLAY_HW_WIDTH;
  uint8_t bit_pos = y % 8;
  
  /* Set or clear the pixel */
  if (color) {
    display_buffer[byte_pos] |= (1 << bit_pos);
  } else {
    display_buffer[byte_pos] &= ~(1 << bit_pos);
  }
}

/**
 * @brief Draw a line using Bresenham's algorithm
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color Line color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
  int16_t dx = abs(x1 - x0);
  int16_t sx = x0 < x1 ? 1 : -1;
  int16_t dy = -abs(y1 - y0);
  int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx + dy;
  int16_t e2;
  
  while (1) {
    Display_DrawPixel(x0, y0, color);
    
    if (x0 == x1 && y0 == y1) {
      break;
    }
    
    e2 = 2 * err;
    
    if (e2 >= dy) {
      if (x0 == x1) {
        break;
      }
      err += dy;
      x0 += sx;
    }
    
    if (e2 <= dx) {
      if (y0 == y1) {
        break;
      }
      err += dx;
      y0 += sy;
    }
  }
}

/**
 * @brief Draw a rectangle
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param w Width of rectangle
 * @param h Height of rectangle
 * @param color Rectangle color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
  Display_DrawLine(x, y, x + w - 1, y, color);
  Display_DrawLine(x, y + h - 1, x + w - 1, y + h - 1, color);
  Display_DrawLine(x, y, x, y + h - 1, color);
  Display_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
}

/**
 * @brief Fill a rectangle
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param w Width of rectangle
 * @param h Height of rectangle
 * @param color Rectangle color (0 = off, 1 = on)
 * @retval None
 */
void Display_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
  for (uint8_t i = 0; i < w; i++) {
    for (uint8_t j = 0; j < h; j++) {
      Display_DrawPixel(x + i, y + j, color);
    }
  }
}

/**
 * @brief Draw a circle using Bresenham's algorithm
 * @param x0 Center X coordinate
 * @param y0 Center Y coordinate
 * @param r Radius
 * @param color Circle color (0 = off, 1 = on)
 * @retval None
 */
void Display_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  
  Display_DrawPixel(x0, y0 + r, color);
  Display_DrawPixel(x0, y0 - r, color);
  Display_DrawPixel(x0 + r, y0, color);
  Display_DrawPixel(x0 - r, y0, color);
  
  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    
    x++;
    ddF_x += 2;
    f += ddF_x;
    
    Display_DrawPixel(x0 + x, y0 + y, color);
    Display_DrawPixel(x0 - x, y0 + y, color);
    Display_DrawPixel(x0 + x, y0 - y, color);
    Display_DrawPixel(x0 - x, y0 - y, color);
    Display_DrawPixel(x0 + y, y0 + x, color);
    Display_DrawPixel(x0 - y, y0 + x, color);
    Display_DrawPixel(x0 + y, y0 - x, color);
    Display_DrawPixel(x0 - y, y0 - x, color);
  }
}

/**
 * @brief Draw a single character
 * @param x X coordinate
 * @param y Y coordinate
 * @param c Character to draw
 * @retval None
 */
void Display_DrawChar(uint8_t x, uint8_t y, char c)
{
  if (c < 32 || c > 126) {
    c = '?';
  }
  
  /* Get character data from font */
  const uint8_t *char_data = &font_5x7[(c - 32) * 5];
  
  /* Draw character */
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t line = char_data[i];
    for (uint8_t j = 0; j < 7; j++) {
      if (line & (1 << j)) {
        Display_DrawPixel(x + i, y + j, 1);
      }
    }
  }
}

/**
 * @brief Draw text string
 * @param x X coordinate
 * @param y Y coordinate
 * @param text Text string to draw
 * @retval None
 */
void Display_DrawText(uint8_t x, uint8_t y, const char *text)
{
  uint8_t cursor_x = x;
  
  while (*text) {
    /* Check if the next character would be cut off */
    /* With fixed 90° orientation, we can use the full width */
    if (cursor_x + 5 > DISPLAY_WIDTH) {
      break;
    }
    
    Display_DrawChar(cursor_x, y, *text);
    cursor_x += 6; /* 5 pixels + 1 pixel space */
    text++;
  }
}

/**
 * @brief Draw text string using row and column coordinates
 * @param row Row (0-based, each row is 8 pixels high)
 * @param col Column (0-based, each column is 6 pixels wide)
 * @param text Text string to draw
 * @retval None
 */
void Display_DrawTextRowCol(uint8_t row, uint8_t col, const char *text)
{
  /* Convert row/col to pixel coordinates */
  /* Each character is 5x7 pixels with 1 pixel spacing */
  uint8_t x = col * 6; /* 6 pixels per column (5 + 1 space) */
  uint8_t y = row * 8; /* 8 pixels per row (7 + 1 space) */
  
  /* Call the pixel-based text drawing function */
  Display_DrawText(x, y, text);
}

/**
 * @brief Draw direction indicator (arrow pointing in specified direction)
 * @param x Center X coordinate
 * @param y Center Y coordinate
 * @param angle Direction angle in degrees (0-359.9)
 * @retval None
 */
void Display_DrawDirectionIndicator(uint8_t x, uint8_t y, float angle)
{
  /* Convert angle to radians */
  float rad = angle * M_PI / 180.0f;
  
  /* Draw compass circle */
  uint8_t radius = 15;
  Display_DrawCircle(x, y, radius, 1);
  
  /* Calculate arrow points - make arrow slightly longer */
  int8_t tip_x = x + (int8_t)(sinf(rad) * (radius - 2));
  int8_t tip_y = y - (int8_t)(cosf(rad) * (radius - 2));
  
  /* Calculate points for arrow base */
  float rad_left = rad - M_PI * 0.7f;  /* Adjusted for better arrow shape */
  float rad_right = rad + M_PI * 0.7f;
  int8_t base_left_x = x + (int8_t)(sinf(rad_left) * radius * 0.5f);
  int8_t base_left_y = y - (int8_t)(cosf(rad_left) * radius * 0.5f);
  int8_t base_right_x = x + (int8_t)(sinf(rad_right) * radius * 0.5f);
  int8_t base_right_y = y - (int8_t)(cosf(rad_right) * radius * 0.5f);
  
  /* Draw arrow */
  Display_DrawLine(x, y, tip_x, tip_y, 1);
  Display_DrawLine(tip_x, tip_y, base_left_x, base_left_y, 1);
  Display_DrawLine(tip_x, tip_y, base_right_x, base_right_y, 1);
  Display_DrawLine(base_left_x, base_left_y, base_right_x, base_right_y, 1);
  
  /* Draw cardinal direction markers */
  /* North marker */
  Display_DrawPixel(x, y - radius, 1);
  Display_DrawPixel(x, y - radius + 1, 1);
  
  /* East marker */
  Display_DrawPixel(x + radius, y, 1);
  Display_DrawPixel(x + radius - 1, y, 1);
  
  /* South marker */
  Display_DrawPixel(x, y + radius, 1);
  Display_DrawPixel(x, y + radius - 1, 1);
  
  /* West marker */
  Display_DrawPixel(x - radius, y, 1);
  Display_DrawPixel(x - radius + 1, y, 1);
}

/* Display rotation functions removed - using fixed 90° orientation */

/**
 * @brief Show boot screen with logo and version info
 * @retval None
 */
void Display_ShowBootScreen(void)
{
  Display_Clear();
  
  /* Draw border */
  Display_DrawRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 1);
  
  /* Draw title */
  Display_DrawTextRowCol(1, 1, "GPS RADIO BEACON");
  Display_DrawTextRowCol(2, 5, "RECEIVER");
  
  /* Draw version info */
  Display_DrawTextRowCol(5, 3, "FW v1.0.0");
  
  Display_Update();
}

/**
 * @brief Initialize I2C for display communication
 * @retval None
 */
static void Display_I2C_Init(void)
{
  /* Enable I2C1 clock */
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure GPIO pins for I2C */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* I2C1 GPIO Configuration    
     PB6 --> I2C1_SCL
     PB7 --> I2C1_SDA 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Configure I2C */
  hi2c_display.Instance = I2C1;
  /* Using fast mode (400kHz) for BNO055 compatibility */
  hi2c_display.Init.ClockSpeed = 100000;
  hi2c_display.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c_display.Init.OwnAddress1 = 0;
  hi2c_display.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c_display.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c_display.Init.OwnAddress2 = 0;
  hi2c_display.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c_display.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  
  if (HAL_I2C_Init(&hi2c_display) != HAL_OK) {
    /* Error handling */
    while(1);
  }
}

/**
 * @brief Send command to display
 * @param command Command byte
 * @retval None
 */
static void Display_SendCommand(uint8_t command)
{
  uint8_t buffer[2] = {SSD1309_COMMAND, command};
  HAL_I2C_Master_Transmit(&hi2c_display, SSD1309_I2C_ADDR << 1, buffer, 2, SSD1309_I2C_TIMEOUT);
}

/**
 * @brief Send data to display
 * @param data Pointer to data buffer
 * @param size Size of data buffer
 * @retval None
 */
static void Display_SendData(uint8_t* data, uint16_t size)
{
  /* We need to send the data in chunks due to I2C buffer limitations */
  uint8_t buffer[17]; /* 1 byte for data/command flag + 16 bytes of data */
  buffer[0] = SSD1309_DATA;
  
  for (uint16_t i = 0; i < size; i += 16) {
    uint16_t chunk_size = (i + 16 > size) ? (size - i) : 16;
    memcpy(&buffer[1], &data[i], chunk_size);
    HAL_I2C_Master_Transmit(&hi2c_display, SSD1309_I2C_ADDR << 1, buffer, chunk_size + 1, SSD1309_I2C_TIMEOUT);
  }
}

/**
 * @brief Set cursor position for text
 * @param x X coordinate (column)
 * @param y Y coordinate (page)
 * @retval None
 */
static __attribute__((unused)) void Display_SetPosition(uint8_t x, uint8_t y)
{
  Display_SendCommand(0xB0 + y);
  Display_SendCommand(((x & 0xF0) >> 4) | 0x10);
  Display_SendCommand(x & 0x0F);
}
