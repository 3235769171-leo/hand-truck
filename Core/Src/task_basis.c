#include "task_basis.h"
#include "main.h"
#include "st7735.h"
#include "jc014.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define KEY_SCAN_PERIOD_MS   10U
#define KEY_DEBOUNCE_SAMPLES 3U

QueueHandle_t g_key_event_queue = NULL;
static QueueHandle_t oled_payload_queue;

static GPIO_TypeDef *const key_ports[4] =
{
  K1_GPIO_Port, K2_GPIO_Port, K3_GPIO_Port, K4_GPIO_Port
};
static const uint16_t key_pins[4] = {K1_Pin, K2_Pin, K3_Pin, K4_Pin};

static uint8_t KeyIsPressed(uint8_t index)
{
  return (HAL_GPIO_ReadPin(key_ports[index], key_pins[index]) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void DrawKeyLine(uint8_t key, uint8_t pressed, uint32_t count)
{
  char line[20];
  uint16_t y = (uint16_t)(40U + ((uint16_t)key - 1U) * 16U);

  ST7735_FillRect(0U, y, ST7735_WIDTH, 8U, ST7735_BLACK);
  (void)snprintf(line, sizeof(line), "K%u %s %04lu",
                 (unsigned int)key, pressed ? "DOWN" : "UP",
                 (unsigned long)count);
  ST7735_DrawString(4U, y, line,
                    pressed ? ST7735_YELLOW : ST7735_WHITE,
                    ST7735_BLACK, 1U);
}

static void DrawJc014Status(JC014_Status_t status)
{
  const char *text;
  uint16_t color;

  switch (status)
  {
    case JC014_STATUS_OK:
      text = "JC024 TX: SENT";
      color = ST7735_GREEN;
      break;
    case JC014_STATUS_NO_ACK:
      text = "JC024: NO ACK";
      color = ST7735_RED;
      break;
    case JC014_STATUS_UART_ERROR:
      text = "JC024: UART ERR";
      color = ST7735_RED;
      break;
    default:
      text = "JC024: WAITING";
      color = ST7735_YELLOW;
      break;
  }

  ST7735_FillRect(0U, 132U, ST7735_WIDTH, 8U, ST7735_BLACK);
  ST7735_DrawString(4U, 132U, text, color, ST7735_BLACK, 1U);
}

static size_t Utf8SequenceLength(uint8_t first_byte)
{
  if ((first_byte & 0xE0U) == 0xC0U)
  {
    return 2U;
  }
  if ((first_byte & 0xF0U) == 0xE0U)
  {
    return 3U;
  }
  if ((first_byte & 0xF8U) == 0xF0U)
  {
    return 4U;
  }
  return 1U;
}

static void DrawPayloadOnOled(const char *payload)
{
  char display_text[JC014_QR_PAYLOAD_MAX_BYTES + 1U];
  size_t source_index = 0U;
  size_t destination_index = 0U;
  size_t payload_length = strlen(payload);

  while ((source_index < payload_length) &&
         (destination_index < JC014_QR_PAYLOAD_MAX_BYTES))
  {
    uint8_t character = (uint8_t)payload[source_index];

    if ((character >= 0x20U) && (character <= 0x7EU))
    {
      display_text[destination_index++] = (char)character;
      source_index++;
    }
    else
    {
      size_t sequence_length = Utf8SequenceLength(character);
      display_text[destination_index++] = '?';
      source_index += sequence_length;
    }
  }
  display_text[destination_index] = '\0';

  ST7735_FillRect(0U, 140U, ST7735_WIDTH, 20U, ST7735_BLACK);
  if (destination_index > 40U)
  {
    destination_index = 37U;
    display_text[destination_index++] = '.';
    display_text[destination_index++] = '.';
    display_text[destination_index++] = '.';
    display_text[destination_index] = '\0';
  }

  ST7735_DrawString(4U, 140U, display_text,
                    ST7735_CYAN, ST7735_BLACK, 1U);
}

void TaskBasis_Init(void)
{
  if (g_key_event_queue == NULL)
  {
    g_key_event_queue = xQueueCreate(KEY_EVENT_QUEUE_LENGTH, sizeof(KeyEvent_t));
    configASSERT(g_key_event_queue != NULL);
  }
  oled_payload_queue = xQueueCreate(1U,
                                    JC014_QR_PAYLOAD_MAX_BYTES + 1U);
  configASSERT(oled_payload_queue != NULL);
}

void TaskBasis_PublishPayload(const char *payload)
{
  if ((oled_payload_queue != NULL) && (payload != NULL))
  {
    (void)xQueueOverwrite(oled_payload_queue, payload);
  }
}

void StartOledTask(void *argument)
{
  KeyEvent_t event;
  char payload[JC014_QR_PAYLOAD_MAX_BYTES + 1U];
  char last_line[20];
  uint8_t key;
  JC014_Status_t jc_status = JC014_STATUS_WAITING;

  (void)argument;
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_DrawString(4U, 4U, "HAND TRUCK", ST7735_WHITE,
                    ST7735_BLACK, 1U);
  ST7735_DrawString(4U, 20U, "KEY TEST ONLY", ST7735_CYAN,
                    ST7735_BLACK, 1U);
  for (key = 1U; key <= 4U; key++)
  {
    DrawKeyLine(key, 0U, 0U);
  }
  ST7735_DrawString(4U, 112U, "LAST: NONE", ST7735_GREEN,
                    ST7735_BLACK, 1U);
  DrawJc014Status(jc_status);

  for (;;)
  {
    if (xQueueReceive(g_key_event_queue, &event, 0U) == pdPASS)
    {
      DrawKeyLine(event.key, event.pressed, event.count);
      (void)snprintf(last_line, sizeof(last_line), "LAST K%u %s",
                     (unsigned int)event.key,
                     event.pressed ? "DOWN" : "UP");
      ST7735_FillRect(0U, 112U, ST7735_WIDTH, 8U, ST7735_BLACK);
      ST7735_DrawString(4U, 112U, last_line,
                        event.pressed ? ST7735_YELLOW : ST7735_GREEN,
                        ST7735_BLACK, 1U);
    }

    if (xQueueReceive(oled_payload_queue, payload, 0U) == pdPASS)
    {
      DrawPayloadOnOled(payload);
    }

    if (jc_status != JC014_GetStatus())
    {
      jc_status = JC014_GetStatus();
      DrawJc014Status(jc_status);
    }

    vTaskDelay(pdMS_TO_TICKS(20U));
  }
}

void StartKeyTask(void *argument)
{
  uint8_t raw_state[4] = {0U, 0U, 0U, 0U};
  uint8_t stable_state[4] = {0U, 0U, 0U, 0U};
  uint8_t stable_samples[4] = {0U, 0U, 0U, 0U};
  uint32_t press_count[4] = {0U, 0U, 0U, 0U};
  uint8_t key_armed = 1U;
  uint8_t index;
  uint8_t pressed;

  (void)argument;
  for (index = 0U; index < 4U; index++)
  {
    pressed = KeyIsPressed(index);
    raw_state[index] = pressed;
    stable_state[index] = pressed;
    if (pressed != 0U)
    {
      key_armed = 0U;
    }
  }

  for (;;)
  {
    for (index = 0U; index < 4U; index++)
    {
      pressed = KeyIsPressed(index);
      if (pressed == raw_state[index])
      {
        if (stable_samples[index] < KEY_DEBOUNCE_SAMPLES)
        {
          stable_samples[index]++;
        }
      }
      else
      {
        raw_state[index] = pressed;
        stable_samples[index] = 0U;
      }

      if ((stable_samples[index] >= KEY_DEBOUNCE_SAMPLES) &&
          (stable_state[index] != raw_state[index]))
      {
        KeyEvent_t event;
        stable_state[index] = raw_state[index];
        if ((key_armed != 0U) && (stable_state[index] != 0U))
        {
          press_count[index]++;
        }
        if (key_armed != 0U)
        {
          event.key = (uint8_t)(index + 1U);
          event.pressed = stable_state[index];
          event.reserved = 0U;
          event.count = press_count[index];
          (void)xQueueSend(g_key_event_queue, &event, 0U);
        }
      }
    }

    if (key_armed == 0U)
    {
      key_armed = 1U;
      for (index = 0U; index < 4U; index++)
      {
        if (stable_state[index] != 0U)
        {
          key_armed = 0U;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));
  }
}
