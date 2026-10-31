#include "st7735.h"
#include "spi.h"
#include <stddef.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#define ST7735_SWRESET       0x01U
#define ST7735_SLPOUT        0x11U
#define ST7735_NORON         0x13U
#define ST7735_INVOFF        0x20U
#define ST7735_DISPON        0x29U
#define ST7735_CASET         0x2AU
#define ST7735_RASET         0x2BU
#define ST7735_RAMWR         0x2CU
#define ST7735_MADCTL        0x36U
#define ST7735_COLMOD        0x3AU
#define ST7735_FRMCTR1       0xB1U
#define ST7735_FRMCTR2       0xB2U
#define ST7735_FRMCTR3       0xB3U
#define ST7735_INVCTR        0xB4U
#define ST7735_DISSET5       0xB6U
#define ST7735_PWCTR1        0xC0U
#define ST7735_PWCTR2        0xC1U
#define ST7735_PWCTR3        0xC2U
#define ST7735_PWCTR4        0xC3U
#define ST7735_PWCTR5        0xC4U
#define ST7735_VMCTR1        0xC5U
#define ST7735_GMCTRP1       0xE0U
#define ST7735_GMCTRN1       0xE1U

#define ST7735_MADCTL_MY     0x80U
#define ST7735_MADCTL_MX     0x40U
#define ST7735_MADCTL_MV     0x20U
#define ST7735_MADCTL_BGR    0x08U

#define ST7735_XSTART        2U
#define ST7735_YSTART        1U
#define ST7735_TX_BUFFER_SIZE 128U

static uint16_t st7735_width = ST7735_WIDTH;
static uint16_t st7735_height = ST7735_HEIGHT;

static void ST7735_Delay(uint32_t milliseconds)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    HAL_Delay(milliseconds);
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
  }
}

static const uint8_t st7735_font5x7[96][5] =
{
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
  {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
  {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
  {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
  {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
  {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
  {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F},
  {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
  {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07},
  {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00},
  {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
  {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18},
  {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00},
  {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
  {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C},
  {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C},
  {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
  {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00},
  {0x08,0x04,0x08,0x10,0x08}, {0x00,0x00,0x00,0x00,0x00}
};

static void ST7735_WriteCommand(uint8_t command)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_WriteData(const uint8_t *data, uint16_t length)
{
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, 100U);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0,
                                    uint16_t x1, uint16_t y1)
{
  uint8_t data[4];

  x0 = (uint16_t)(x0 + ST7735_XSTART);
  x1 = (uint16_t)(x1 + ST7735_XSTART);
  y0 = (uint16_t)(y0 + ST7735_YSTART);
  y1 = (uint16_t)(y1 + ST7735_YSTART);

  ST7735_WriteCommand(ST7735_CASET);
  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)x0;
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)x1;
  ST7735_WriteData(data, 4U);

  ST7735_WriteCommand(ST7735_RASET);
  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)y0;
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)y1;
  ST7735_WriteData(data, 4U);

  ST7735_WriteCommand(ST7735_RAMWR);
}

static void ST7735_WriteColorData(uint16_t color, uint32_t pixels)
{
  uint8_t buffer[ST7735_TX_BUFFER_SIZE];
  uint16_t index;
  uint16_t chunk_pixels;
  uint8_t high;
  uint8_t low;

  high = (uint8_t)(color >> 8);
  low = (uint8_t)color;

  for (index = 0U; index < ST7735_TX_BUFFER_SIZE; index += 2U)
  {
    buffer[index] = high;
    buffer[index + 1U] = low;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  while (pixels != 0U)
  {
    chunk_pixels = (pixels > (ST7735_TX_BUFFER_SIZE / 2U)) ?
                   (ST7735_TX_BUFFER_SIZE / 2U) : (uint16_t)pixels;
    (void)HAL_SPI_Transmit(&hspi1, buffer,
                           (uint16_t)(chunk_pixels * 2U), 100U);
    pixels -= chunk_pixels;
  }

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void ST7735_SetRotation(uint8_t rotation)
{
  uint8_t madctl;

  switch (rotation & 3U)
  {
    case 1U:
      madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
      st7735_width = ST7735_HEIGHT;
      st7735_height = ST7735_WIDTH;
      break;

    case 2U:
      madctl = ST7735_MADCTL_BGR;
      st7735_width = ST7735_WIDTH;
      st7735_height = ST7735_HEIGHT;
      break;

    case 3U:
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
      st7735_width = ST7735_HEIGHT;
      st7735_height = ST7735_WIDTH;
      break;

    default:
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR;
      st7735_width = ST7735_WIDTH;
      st7735_height = ST7735_HEIGHT;
      break;
  }

  ST7735_WriteCommand(ST7735_MADCTL);
  ST7735_WriteData(&madctl, 1U);
}

void ST7735_Init(void)
{
  static const uint8_t frame_rate_1[] = {0x01U, 0x2CU, 0x2DU};
  static const uint8_t frame_rate_3[] = {0x01U, 0x2CU, 0x2DU,
                                         0x01U, 0x2CU, 0x2DU};
  static const uint8_t gamma_positive[] =
  {
    0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U,
    0x2DU, 0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U
  };
  static const uint8_t gamma_negative[] =
  {
    0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U,
    0x2DU, 0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U
  };
  static const uint8_t color_mode[] = {0x05U};
  static const uint8_t inversion[] = {0x07U};
  static const uint8_t display_function[] = {0x15U, 0x02U};
  static const uint8_t power_1[] = {0xA2U, 0x02U, 0x84U};
  static const uint8_t power_2[] = {0xC5U};
  static const uint8_t power_3[] = {0x0AU, 0x00U};
  static const uint8_t power_4[] = {0x8AU, 0x2AU};
  static const uint8_t power_5[] = {0x8AU, 0xEEU};
  static const uint8_t vcom[] = {0x0EU};

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  ST7735_Delay(10U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  ST7735_Delay(120U);

  ST7735_WriteCommand(ST7735_SWRESET);
  ST7735_Delay(150U);
  ST7735_WriteCommand(ST7735_SLPOUT);
  ST7735_Delay(120U);

  ST7735_WriteCommand(ST7735_FRMCTR1);
  ST7735_WriteData(frame_rate_1, sizeof(frame_rate_1));
  ST7735_WriteCommand(ST7735_FRMCTR2);
  ST7735_WriteData(frame_rate_1, sizeof(frame_rate_1));
  ST7735_WriteCommand(ST7735_FRMCTR3);
  ST7735_WriteData(frame_rate_3, sizeof(frame_rate_3));

  ST7735_WriteCommand(ST7735_INVCTR);
  ST7735_WriteData(inversion, sizeof(inversion));
  ST7735_WriteCommand(ST7735_DISSET5);
  ST7735_WriteData(display_function, sizeof(display_function));

  ST7735_WriteCommand(ST7735_PWCTR1);
  ST7735_WriteData(power_1, sizeof(power_1));
  ST7735_WriteCommand(ST7735_PWCTR2);
  ST7735_WriteData(power_2, sizeof(power_2));
  ST7735_WriteCommand(ST7735_PWCTR3);
  ST7735_WriteData(power_3, sizeof(power_3));
  ST7735_WriteCommand(ST7735_PWCTR4);
  ST7735_WriteData(power_4, sizeof(power_4));
  ST7735_WriteCommand(ST7735_PWCTR5);
  ST7735_WriteData(power_5, sizeof(power_5));
  ST7735_WriteCommand(ST7735_VMCTR1);
  ST7735_WriteData(vcom, sizeof(vcom));

  ST7735_WriteCommand(ST7735_GMCTRP1);
  ST7735_WriteData(gamma_positive, sizeof(gamma_positive));
  ST7735_WriteCommand(ST7735_GMCTRN1);
  ST7735_WriteData(gamma_negative, sizeof(gamma_negative));

  ST7735_WriteCommand(ST7735_COLMOD);
  ST7735_WriteData(color_mode, sizeof(color_mode));
  ST7735_SetRotation(0U);
  ST7735_WriteCommand(ST7735_INVOFF);
  ST7735_WriteCommand(ST7735_NORON);
  ST7735_Delay(10U);
  ST7735_WriteCommand(ST7735_DISPON);
  ST7735_Delay(100U);

  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);
}

void ST7735_FillScreen(uint16_t color)
{
  ST7735_FillRect(0U, 0U, st7735_width, st7735_height, color);
}

void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t width,
                     uint16_t height, uint16_t color)
{
  if ((width == 0U) || (height == 0U) ||
      (x >= st7735_width) || (y >= st7735_height))
  {
    return;
  }

  if ((uint32_t)x + width > st7735_width)
  {
    width = (uint16_t)(st7735_width - x);
  }
  if ((uint32_t)y + height > st7735_height)
  {
    height = (uint16_t)(st7735_height - y);
  }

  ST7735_SetAddressWindow(x, y, (uint16_t)(x + width - 1U),
                          (uint16_t)(y + height - 1U));
  ST7735_WriteColorData(color, (uint32_t)width * height);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint8_t data[2];

  if ((x >= st7735_width) || (y >= st7735_height))
  {
    return;
  }

  ST7735_SetAddressWindow(x, y, x, y);
  data[0] = (uint8_t)(color >> 8);
  data[1] = (uint8_t)color;
  ST7735_WriteData(data, sizeof(data));
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char character,
                     uint16_t color, uint16_t background, uint8_t size)
{
  const uint8_t *glyph;
  uint8_t row_buffer[48];
  uint8_t column;
  uint8_t row;
  uint8_t horizontal;
  uint8_t vertical;
  uint8_t glyph_bit;
  uint16_t row_bytes;
  uint16_t index;
  uint16_t pixel_color;

  if (size == 0U)
  {
    size = 1U;
  }
  if (size > 4U)
  {
    size = 4U;
  }
  if ((x >= st7735_width) || (y >= st7735_height) ||
      ((uint32_t)x + (uint32_t)(6U * size) > st7735_width) ||
      ((uint32_t)y + (uint32_t)(8U * size) > st7735_height))
  {
    return;
  }

  if (((uint8_t)character < 32U) || ((uint8_t)character > 127U))
  {
    character = '?';
  }

  glyph = st7735_font5x7[(uint8_t)character - 32U];
  row_bytes = (uint16_t)(6U * size * 2U);

  ST7735_SetAddressWindow(x, y, (uint16_t)(x + 6U * size - 1U),
                          (uint16_t)(y + 8U * size - 1U));
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  for (row = 0U; row < 8U; row++)
  {
    for (vertical = 0U; vertical < size; vertical++)
    {
      index = 0U;
      for (column = 0U; column < 6U; column++)
      {
        glyph_bit = (column < 5U) ? (uint8_t)((glyph[column] >> row) & 1U) : 0U;
        for (horizontal = 0U; horizontal < size; horizontal++)
        {
          pixel_color = glyph_bit ? color : background;
          row_buffer[index++] = (uint8_t)(pixel_color >> 8);
          row_buffer[index++] = (uint8_t)pixel_color;
        }
      }
      (void)HAL_SPI_Transmit(&hspi1, row_buffer, row_bytes, 100U);
    }
  }

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *text,
                       uint16_t color, uint16_t background, uint8_t size)
{
  uint16_t origin_x;
  uint16_t advance;

  if (text == NULL)
  {
    return;
  }
  if (size == 0U)
  {
    size = 1U;
  }

  origin_x = x;
  advance = (uint16_t)(6U * size);

  while (*text != '\0')
  {
    if (*text == '\n')
    {
      x = origin_x;
      y = (uint16_t)(y + 8U * size);
    }
    else
    {
      if ((uint32_t)x + advance > st7735_width)
      {
        x = origin_x;
        y = (uint16_t)(y + 8U * size);
      }
      if ((uint32_t)y + 8U * size > st7735_height)
      {
        break;
      }
      ST7735_DrawChar(x, y, *text, color, background, size);
      x = (uint16_t)(x + advance);
    }
    text++;
  }
}
