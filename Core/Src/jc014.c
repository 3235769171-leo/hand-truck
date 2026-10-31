#include "jc014.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define JC024_UART_TIMEOUT_MS 1000U
#define JC024_ACK_PROBE_TIMEOUT_MS 100U
#define JC024_ACK_TIMEOUT_MS  5000U
#define JC024_BOX_TIME_MS     50U
#define JC024_TEXT_WAIT_MS    40U
#define JC024_CLEAR_WAIT_MS   120U
/* The screen accepts at most 500 bytes per CRLF-terminated command string. */
#define JC024_COMMAND_MAX_BYTES 500U
#define JC024_DRAW_BATCH_SIZE 480U
#define JC024_SCREEN_WIDTH_PX 320U
#define JC024_SCREEN_HEIGHT_PX 240U
/* 84 px is about 12.85 mm at the JC024 active-display pixel pitch. */
#define JC024_GLYPH_COLUMNS   7U
#define JC024_GLYPH_ROWS      14U
#define JC024_PIXEL_WIDTH_PX  5U
#define JC024_PIXEL_HEIGHT_PX 6U
#define JC024_GLYPH_WIDTH_PX  (JC024_GLYPH_COLUMNS * JC024_PIXEL_WIDTH_PX)
#define JC024_GLYPH_HEIGHT_PX (JC024_GLYPH_ROWS * JC024_PIXEL_HEIGHT_PX)
#define JC024_CHAR_GAP_PX     2U
#define JC024_ROW_GAP_PX      20U
#define JC024_MAX_NUMBER_CHARS_PER_LINE 8U
#define JC024_MAX_NUMBER_CHARS (2U * JC024_MAX_NUMBER_CHARS_PER_LINE)
#define JC024_CLEAR_FULL_SCREEN 1U
#define JC024_TEXT_COMMAND_SIZE (JC014_QR_PAYLOAD_MAX_BYTES + 64U)
#define JC024_NUMBER_Y        ((JC024_SCREEN_HEIGHT_PX - JC024_GLYPH_HEIGHT_PX) / 2U)
#define JC024_FIRST_ROW_Y     ((JC024_SCREEN_HEIGHT_PX - \
                                (2U * JC024_GLYPH_HEIGHT_PX + \
                                 JC024_ROW_GAP_PX)) / 2U)

enum
{
  GLYPH_0 = 0U,
  GLYPH_1,
  GLYPH_2,
  GLYPH_3,
  GLYPH_4,
  GLYPH_5,
  GLYPH_6,
  GLYPH_7,
  GLYPH_8,
  GLYPH_9,
  GLYPH_PLUS,
  GLYPH_MINUS,
  GLYPH_DOT,
  GLYPH_COUNT
};

static volatile JC014_Status_t jc014_status = JC014_STATUS_WAITING;
static SemaphoreHandle_t jc014_mutex;
static uint8_t jc014_screen_configured;
static char jc024_draw_batch[JC024_DRAW_BATCH_SIZE];

typedef enum
{
  JC024_ACK_UNKNOWN = 0U,
  JC024_ACK_AVAILABLE,
  JC024_ACK_UNAVAILABLE
} JC024_AckState_t;

static JC024_AckState_t jc024_ack_state = JC024_ACK_UNKNOWN;

static const uint8_t glyph_rows[GLYPH_COUNT][JC024_GLYPH_ROWS] =
{
  {0x1CU, 0x3EU, 0x63U, 0x63U, 0x63U, 0x63U, 0x63U,
   0x63U, 0x63U, 0x63U, 0x63U, 0x63U, 0x3EU, 0x1CU},
  {0x0CU, 0x1CU, 0x3CU, 0x0CU, 0x0CU, 0x0CU, 0x0CU,
   0x0CU, 0x0CU, 0x0CU, 0x0CU, 0x0CU, 0x3EU, 0x3EU},
  {0x1CU, 0x3EU, 0x63U, 0x03U, 0x03U, 0x06U, 0x0CU,
   0x18U, 0x30U, 0x60U, 0x60U, 0x60U, 0x7FU, 0x7FU},
  {0x7EU, 0x03U, 0x03U, 0x06U, 0x1EU, 0x1EU, 0x06U,
   0x03U, 0x03U, 0x03U, 0x03U, 0x06U, 0x7EU, 0x7CU},
  {0x06U, 0x0EU, 0x1EU, 0x36U, 0x66U, 0x66U, 0x66U,
   0x7FU, 0x7FU, 0x06U, 0x06U, 0x06U, 0x06U, 0x06U},
  {0x7FU, 0x7FU, 0x60U, 0x60U, 0x7EU, 0x7FU, 0x03U,
   0x03U, 0x03U, 0x03U, 0x03U, 0x06U, 0x7EU, 0x7CU},
  {0x1EU, 0x30U, 0x60U, 0x60U, 0x7EU, 0x63U, 0x63U,
   0x63U, 0x63U, 0x63U, 0x63U, 0x63U, 0x3EU, 0x1CU},
  {0x7FU, 0x7FU, 0x03U, 0x06U, 0x0CU, 0x18U, 0x18U,
   0x30U, 0x30U, 0x30U, 0x30U, 0x30U, 0x30U, 0x30U},
  {0x3EU, 0x63U, 0x63U, 0x63U, 0x3EU, 0x63U, 0x63U,
   0x63U, 0x63U, 0x63U, 0x63U, 0x63U, 0x3EU, 0x1CU},
  {0x3EU, 0x63U, 0x63U, 0x63U, 0x63U, 0x63U, 0x63U,
   0x3FU, 0x03U, 0x03U, 0x03U, 0x06U, 0x3CU, 0x38U},
  {0x00U, 0x00U, 0x0CU, 0x0CU, 0x0CU, 0x0CU, 0x7FU,
   0x7FU, 0x0CU, 0x0CU, 0x0CU, 0x0CU, 0x00U, 0x00U},
  {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x7FU,
   0x7FU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U},
  {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
   0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x0CU, 0x0CU}
};

static const uint8_t line_glyphs[2][8] =
{
  {GLYPH_1, GLYPH_2, GLYPH_3, GLYPH_PLUS,
   GLYPH_3, GLYPH_2, GLYPH_1, GLYPH_PLUS},
  {GLYPH_2, GLYPH_5, GLYPH_4, GLYPH_PLUS,
   GLYPH_3, GLYPH_1, GLYPH_2, 0xFFU}
};

static const uint8_t line_lengths[2] = {8U, 7U};

JC014_Status_t JC014_GetStatus(void)
{
  return jc014_status;
}

void JC014_Init(void)
{
  jc014_mutex = xSemaphoreCreateMutex();
  jc014_screen_configured = 0U;
  jc024_ack_state = JC024_ACK_UNKNOWN;
  configASSERT(jc014_mutex != NULL);
}

HAL_StatusTypeDef JC014_SendCommand(const char *command, uint32_t wait_ms)
{
  size_t length;
  HAL_StatusTypeDef status;
  uint32_t start_tick;
  uint32_t elapsed_ms;
  uint32_t timeout_ms;
  uint8_t reply_byte;
  uint8_t matched = 0U;
  static const uint8_t acknowledgement[] = {'O', 'K', '\r', '\n'};

  if (command == NULL)
  {
    return HAL_ERROR;
  }

  length = strlen(command);
  if ((length == 0U) || (length > JC024_COMMAND_MAX_BYTES))
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Transmit(&huart4, (uint8_t *)command,
                             (uint16_t)length, JC024_UART_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  if (jc024_ack_state == JC024_ACK_UNAVAILABLE)
  {
    if (wait_ms > 0U)
    {
      vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
    return HAL_OK;
  }

  timeout_ms = (jc024_ack_state == JC024_ACK_UNKNOWN) ?
               JC024_ACK_PROBE_TIMEOUT_MS : JC024_ACK_TIMEOUT_MS;
  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    elapsed_ms = HAL_GetTick() - start_tick;
    if (elapsed_ms >= timeout_ms)
    {
      break;
    }
    status = HAL_UART_Receive(&huart4, &reply_byte, 1U,
                              timeout_ms - elapsed_ms);
    if (status != HAL_OK)
    {
      break;
    }

    if (reply_byte == acknowledgement[matched])
    {
      matched++;
      if (matched == sizeof(acknowledgement))
      {
        jc024_ack_state = JC024_ACK_AVAILABLE;
        /* An early ACK must not let the next drawing batch overtake this one. */
        elapsed_ms = HAL_GetTick() - start_tick;
        if (elapsed_ms < wait_ms)
        {
          vTaskDelay(pdMS_TO_TICKS(wait_ms - elapsed_ms));
        }
        return HAL_OK;
      }
    }
    else
    {
      matched = (reply_byte == acknowledgement[0]) ? 1U : 0U;
    }
  }

  jc024_ack_state = JC024_ACK_UNAVAILABLE;
  if (wait_ms > 0U)
  {
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
  }
  return HAL_OK;
}

HAL_StatusTypeDef JC014_ClearScreen(void)
{
  HAL_StatusTypeDef status;

  if ((jc014_mutex == NULL) ||
      (xSemaphoreTake(jc014_mutex, portMAX_DELAY) != pdTRUE))
  {
    return HAL_ERROR;
  }

  status = JC014_SendCommand("DIR(1);BOXF(0,0,319,239,15);\r\n",
                             JC024_CLEAR_WAIT_MS + JC024_TEXT_WAIT_MS);
  if (status == HAL_OK)
  {
    jc014_screen_configured = 1U;
    jc014_status = JC014_STATUS_OK;
  }
  else
  {
    jc014_status = JC014_STATUS_UART_ERROR;
  }

  (void)xSemaphoreGive(jc014_mutex);
  return status;
}

static HAL_StatusTypeDef JC024_PrepareScreen(void)
{
  HAL_StatusTypeDef status;

  if (jc014_screen_configured != 0U)
  {
    return HAL_OK;
  }

  status = JC014_SendCommand("DIR(1);\r\n", 50U);
  if (status == HAL_OK)
  {
    jc014_screen_configured = 1U;
  }

  return status;
}

static HAL_StatusTypeDef JC024_DrawGlyph(uint16_t x, uint16_t y,
                                         uint8_t glyph_id,
                                         uint8_t clear_before)
{
  uint8_t row;
  uint8_t row_end;
  uint8_t column;
  uint8_t first_column;
  uint8_t row_bits;
  uint16_t x1;
  uint16_t x2;
  uint16_t y1;
  uint16_t y2;
  size_t batch_length = 0U;
  uint16_t box_count = 0U;
  size_t remaining;
  int written;
  uint32_t wait_ms;
  HAL_StatusTypeDef status;

  if (clear_before == JC024_CLEAR_FULL_SCREEN)
  {
    written = snprintf(jc024_draw_batch, sizeof(jc024_draw_batch),
                       "BOXF(0,0,319,239,15);");
    if ((written < 0) || ((size_t)written >= sizeof(jc024_draw_batch)))
    {
      return HAL_ERROR;
    }
    batch_length = (size_t)written;
    box_count = 1U;
  }
  for (row = 0U; row < JC024_GLYPH_ROWS; row++)
  {
    row_bits = glyph_rows[glyph_id][row];
    row_end = (uint8_t)(row + 1U);
    while ((row_end < JC024_GLYPH_ROWS) &&
           (glyph_rows[glyph_id][row_end] == row_bits))
    {
      row_end++;
    }

    column = 0U;
    while (column < JC024_GLYPH_COLUMNS)
    {
      while ((column < JC024_GLYPH_COLUMNS) &&
             ((row_bits & (1U << (JC024_GLYPH_COLUMNS - 1U - column))) == 0U))
      {
        column++;
      }
      if (column >= JC024_GLYPH_COLUMNS)
      {
        break;
      }

      first_column = column;
      while ((column < JC024_GLYPH_COLUMNS) &&
             ((row_bits & (1U << (JC024_GLYPH_COLUMNS - 1U - column))) != 0U))
      {
        column++;
      }

      x1 = (uint16_t)(x + first_column * JC024_PIXEL_WIDTH_PX);
      x2 = (uint16_t)(x + column * JC024_PIXEL_WIDTH_PX - 1U);
      y1 = (uint16_t)(y + row * JC024_PIXEL_HEIGHT_PX);
      y2 = (uint16_t)(y + row_end * JC024_PIXEL_HEIGHT_PX - 1U);

      remaining = sizeof(jc024_draw_batch) - batch_length;
      written = snprintf(&jc024_draw_batch[batch_length], remaining,
                         "BOXF(%u,%u,%u,%u,0);",
                         (unsigned int)x1, (unsigned int)y1,
                         (unsigned int)x2, (unsigned int)y2);
      if ((remaining < 3U) || (written < 0) ||
          ((size_t)written > remaining - 3U))
      {
        return HAL_ERROR;
      }

      batch_length += (size_t)written;
      box_count++;
    }

    row = (uint8_t)(row_end - 1U);
  }

  if (batch_length > 0U)
  {
    if ((batch_length + 2U) >= sizeof(jc024_draw_batch))
    {
      return HAL_ERROR;
    }
    jc024_draw_batch[batch_length++] = '\r';
    jc024_draw_batch[batch_length++] = '\n';
    jc024_draw_batch[batch_length] = '\0';
    wait_ms = (uint32_t)box_count * JC024_BOX_TIME_MS;
    if (clear_before == JC024_CLEAR_FULL_SCREEN)
    {
      wait_ms += JC024_CLEAR_WAIT_MS;
    }
    status = JC014_SendCommand(jc024_draw_batch, wait_ms);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef JC014_ShowStartupScreen(void)
{
  uint8_t row;
  uint8_t index;
  uint16_t x;
  uint16_t y;
  uint16_t row_width;
  uint16_t row_start_x;
  HAL_StatusTypeDef status;

  if ((jc014_mutex == NULL) ||
      (xSemaphoreTake(jc014_mutex, portMAX_DELAY) != pdTRUE))
  {
    return HAL_ERROR;
  }

  status = JC014_SendCommand("DIR(1);\r\n", 150U);
  if (status != HAL_OK)
  {
    jc014_status = JC014_STATUS_UART_ERROR;
    (void)xSemaphoreGive(jc014_mutex);
    return status;
  }

  status = JC014_SendCommand("CLR(15);\r\n", 150U);
  if (status != HAL_OK)
  {
    jc014_status = JC014_STATUS_UART_ERROR;
    (void)xSemaphoreGive(jc014_mutex);
    return status;
  }
  jc014_screen_configured = 1U;

  for (row = 0U; row < 2U; row++)
  {
    y = (uint16_t)(JC024_FIRST_ROW_Y + row *
                   (JC024_GLYPH_HEIGHT_PX + JC024_ROW_GAP_PX));
    row_width = (uint16_t)(line_lengths[row] * JC024_GLYPH_WIDTH_PX +
                (line_lengths[row] - 1U) * JC024_CHAR_GAP_PX);
    row_start_x = (uint16_t)((JC024_SCREEN_WIDTH_PX - row_width) / 2U);

    for (index = 0U; index < line_lengths[row]; index++)
    {
      x = (uint16_t)(row_start_x +
                     index * (JC024_GLYPH_WIDTH_PX + JC024_CHAR_GAP_PX));
      status = JC024_DrawGlyph(x, y, line_glyphs[row][index], 0U);
      if (status != HAL_OK)
      {
        jc014_status = JC014_STATUS_UART_ERROR;
        (void)xSemaphoreGive(jc014_mutex);
        return status;
      }
    }
  }

  jc014_status = JC014_STATUS_OK;
  (void)xSemaphoreGive(jc014_mutex);
  return HAL_OK;
}

static uint8_t JC024_MapNumberGlyph(char character, uint8_t *glyph_id)
{
  if ((character >= '0') && (character <= '9'))
  {
    *glyph_id = (uint8_t)(GLYPH_0 + (uint8_t)(character - '0'));
    return 1U;
  }

  if (character == '+')
  {
    *glyph_id = GLYPH_PLUS;
    return 1U;
  }
  if (character == '-')
  {
    *glyph_id = GLYPH_MINUS;
    return 1U;
  }
  if (character == '.')
  {
    *glyph_id = GLYPH_DOT;
    return 1U;
  }

  return 0U;
}

HAL_StatusTypeDef JC014_ShowNumber(const char *text)
{
  uint8_t glyph_ids[2][JC024_MAX_NUMBER_CHARS_PER_LINE];
  uint8_t row_char_counts[2];
  uint8_t number_rows;
  uint8_t row;
  uint8_t column;
  size_t length;
  size_t index;
  uint16_t row_width;
  uint16_t x;
  uint16_t y;
  uint16_t row_start_y;
  uint8_t clear_before;
  HAL_StatusTypeDef status;

  if (text == NULL)
  {
    return HAL_ERROR;
  }

  length = strlen(text);
  if ((length == 0U) || (length > JC024_MAX_NUMBER_CHARS))
  {
    return HAL_ERROR;
  }

  number_rows = (length > JC024_MAX_NUMBER_CHARS_PER_LINE) ? 2U : 1U;
  row_char_counts[0] = (length > JC024_MAX_NUMBER_CHARS_PER_LINE) ?
                       JC024_MAX_NUMBER_CHARS_PER_LINE : (uint8_t)length;
  row_char_counts[1] = (number_rows == 2U) ?
                       (uint8_t)(length - row_char_counts[0]) : 0U;

  for (index = 0U; index < length; index++)
  {
    uint8_t glyph_id;

    if (JC024_MapNumberGlyph(text[index], &glyph_id) == 0U)
    {
      return HAL_ERROR;
    }
    if (index < row_char_counts[0])
    {
      glyph_ids[0][index] = glyph_id;
    }
    else
    {
      glyph_ids[1][index - row_char_counts[0]] = glyph_id;
    }
  }

  if ((jc014_mutex == NULL) ||
      (xSemaphoreTake(jc014_mutex, portMAX_DELAY) != pdTRUE))
  {
    return HAL_ERROR;
  }

  status = JC024_PrepareScreen();
  if (status != HAL_OK)
  {
    jc014_status = JC014_STATUS_UART_ERROR;
    (void)xSemaphoreGive(jc014_mutex);
    return status;
  }

  clear_before = JC024_CLEAR_FULL_SCREEN;

  for (row = 0U; row < number_rows; row++)
  {
    row_width = (uint16_t)(row_char_counts[row] * JC024_GLYPH_WIDTH_PX +
                           (row_char_counts[row] - 1U) * JC024_CHAR_GAP_PX);
    x = (uint16_t)((JC024_SCREEN_WIDTH_PX - row_width) / 2U);
    row_start_y = JC024_FIRST_ROW_Y;
    y = (uint16_t)(row_start_y + row *
                   (JC024_GLYPH_HEIGHT_PX + JC024_ROW_GAP_PX));

    for (column = 0U; column < row_char_counts[row]; column++)
    {
      /* Send one complete glyph at a time, below the screen's command limit. */
      status = JC024_DrawGlyph(
          (uint16_t)(x + column *
                     (JC024_GLYPH_WIDTH_PX + JC024_CHAR_GAP_PX)),
          y, glyph_ids[row][column],
          ((row == 0U) && (column == 0U)) ? clear_before : 0U);
      if (status != HAL_OK)
      {
        jc014_status = JC014_STATUS_UART_ERROR;
        (void)xSemaphoreGive(jc014_mutex);
        return status;
      }
    }
  }

  jc014_status = JC014_STATUS_OK;
  (void)xSemaphoreGive(jc014_mutex);
  return HAL_OK;
}

HAL_StatusTypeDef JC014_ShowQrPayload(const char *text)
{
  char command[JC024_TEXT_COMMAND_SIZE];
  size_t length;
  size_t index;
  int command_length;
  uint8_t glyph_id;
  uint8_t use_large_digits = 1U;
  HAL_StatusTypeDef status;

  if (text == NULL)
  {
    return HAL_ERROR;
  }

  length = strlen(text);
  if ((length == 0U) || (length > JC014_QR_PAYLOAD_MAX_BYTES))
  {
    return HAL_ERROR;
  }

  for (index = 0U; index < length; index++)
  {
    uint8_t character = (uint8_t)text[index];

    if ((character < 0x20U) || (character == 0x7FU) ||
        (character == (uint8_t)'\'') || (character == (uint8_t)'\\'))
    {
      return HAL_ERROR;
    }

    if ((use_large_digits != 0U) &&
        ((length > JC024_MAX_NUMBER_CHARS) ||
         (JC024_MapNumberGlyph(text[index], &glyph_id) == 0U)))
    {
      use_large_digits = 0U;
    }
  }

  if (use_large_digits != 0U)
  {
    return JC014_ShowNumber(text);
  }

  command_length = snprintf(command, sizeof(command),
                            "BOXF(0,0,319,239,15);DC16(0,112,'%s',0);\r\n",
                            text);
  if ((command_length < 0) ||
      ((size_t)command_length >= sizeof(command)))
  {
    return HAL_ERROR;
  }

  if ((jc014_mutex == NULL) ||
      (xSemaphoreTake(jc014_mutex, portMAX_DELAY) != pdTRUE))
  {
    return HAL_ERROR;
  }

  status = JC024_PrepareScreen();
  if (status == HAL_OK)
  {
    status = JC014_SendCommand(command,
                               JC024_CLEAR_WAIT_MS + JC024_TEXT_WAIT_MS);
  }
  if (status != HAL_OK)
  {
    jc014_status = JC014_STATUS_UART_ERROR;
  }
  else
  {
    jc014_status = JC014_STATUS_OK;
  }

  (void)xSemaphoreGive(jc014_mutex);
  return status;
}
