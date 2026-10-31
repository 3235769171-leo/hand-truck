#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"
#include <stdint.h>

#define ST7735_WIDTH       128U
#define ST7735_HEIGHT      160U

#define ST7735_BLACK       0x0000U
#define ST7735_BLUE        0x001FU
#define ST7735_RED         0xF800U
#define ST7735_GREEN       0x07E0U
#define ST7735_CYAN        0x07FFU
#define ST7735_MAGENTA     0xF81FU
#define ST7735_YELLOW      0xFFE0U
#define ST7735_WHITE       0xFFFFU
#define ST7735_ORANGE      0xFD20U

void ST7735_Init(void);
void ST7735_SetRotation(uint8_t rotation);
void ST7735_FillScreen(uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t width,
                     uint16_t height, uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, char character,
                     uint16_t color, uint16_t background, uint8_t size);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *text,
                       uint16_t color, uint16_t background, uint8_t size);

#endif
