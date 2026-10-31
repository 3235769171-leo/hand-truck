/* USER CODE BEGIN Header */
/**
  * @file freertos.c
  * @brief Native FreeRTOS task and synchronization setup.
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "jc014.h"
#include "motor_control.h"
#include "task_basis.h"
#include "usart.h"

/* Reserve higher levels for future motion control and safety tasks. */
#define KEY_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)
#define JC014_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)
#define LUBANCAT_RX_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define DISPLAY_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define UART_TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define LUBANCAT_RX_QUEUE_LENGTH 64U
#define LUBANCAT_PAYLOAD_BUFFER_SIZE (JC014_QR_PAYLOAD_MAX_BYTES + 1U)

static TaskHandle_t keyTaskHandle;
static TaskHandle_t jc014TaskHandle;
static TaskHandle_t displayTaskHandle;
static TaskHandle_t uartTestTaskHandle;
static TaskHandle_t lubanCatRxTaskHandle;
static TaskHandle_t lubanCatDisplayTaskHandle;
static QueueHandle_t lubanCatRxQueue;
static QueueHandle_t lubanCatDisplayQueue;
static SemaphoreHandle_t jc014ReadySemaphore;
static uint8_t lubanCatRxByte;

typedef enum
{
  LUBANCAT_WAIT_HEADER_AA = 0U,
  LUBANCAT_WAIT_HEADER_55,
  LUBANCAT_IN_PAYLOAD
} LubanCatRxState_t;

static StaticTask_t idleTaskTcb;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

static void StartJc014Task(void *argument);
static void StartUartTestTask(void *argument);
static void StartLubanCatRxTask(void *argument);
static void StartLubanCatDisplayTask(void *argument);
static uint8_t IsValidUtf8(const uint8_t *text, uint16_t length);
static void AppendLubanCatPayloadByte(uint8_t byte, uint8_t *payload,
                                     uint16_t *length, uint8_t *overflowed);
static void PublishLubanCatPayload(uint8_t *payload, uint16_t *length,
                                   uint8_t *overflowed);

void MX_FREERTOS_Init(void)
{
  TaskBasis_Init();
  Motor_Init();

  lubanCatRxQueue = xQueueCreate(LUBANCAT_RX_QUEUE_LENGTH, sizeof(uint8_t));
  lubanCatDisplayQueue = xQueueCreate(1U, LUBANCAT_PAYLOAD_BUFFER_SIZE);
  jc014ReadySemaphore = xSemaphoreCreateBinary();
  configASSERT(lubanCatRxQueue != NULL);
  configASSERT(lubanCatDisplayQueue != NULL);
  configASSERT(jc014ReadySemaphore != NULL);

  configASSERT(xTaskCreate(StartKeyTask, "keys", 256U, NULL,
                           KEY_TASK_PRIORITY, &keyTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(StartJc014Task, "jc014", 256U, NULL,
                           JC014_TASK_PRIORITY, &jc014TaskHandle) == pdPASS);
  configASSERT(xTaskCreate(StartOledTask, "display", 512U, NULL,
                           DISPLAY_TASK_PRIORITY, &displayTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(StartUartTestTask, "uart_test", 256U, NULL,
                           UART_TEST_TASK_PRIORITY, &uartTestTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(StartLubanCatRxTask, "lubancat_rx", 256U, NULL,
                           LUBANCAT_RX_TASK_PRIORITY,
                           &lubanCatRxTaskHandle) == pdPASS);
  configASSERT(xTaskCreate(StartLubanCatDisplayTask, "screen_value", 256U,
                           NULL, DISPLAY_TASK_PRIORITY,
                           &lubanCatDisplayTaskHandle) == pdPASS);
}

static void StartJc014Task(void *argument)
{
  (void)argument;
  JC014_Init();
  vTaskDelay(pdMS_TO_TICKS(1000U));
  (void)JC014_ClearScreen();
  (void)xSemaphoreGive(jc014ReadySemaphore);
  vTaskDelete(NULL);
}

static void StartUartTestTask(void *argument)
{
  static const uint8_t uart5Message[] = "UART5 test: 456\r\n";

  (void)argument;

  for (;;)
  {
    (void)HAL_UART_Transmit(&huart5, (uint8_t *)uart5Message,
                            (uint16_t)(sizeof(uart5Message) - 1U), 20U);
    vTaskDelay(pdMS_TO_TICKS(1000U));
  }
}

static uint8_t IsValidUtf8(const uint8_t *text, uint16_t length)
{
  uint16_t index = 0U;
  uint8_t first;
  uint8_t second;
  uint8_t continuation_count;
  uint8_t second_min;
  uint8_t second_max;

  while (index < length)
  {
    first = text[index];
    if (first <= 0x7FU)
    {
      if ((first == 0U) || (first < 0x20U) || (first == 0x7FU))
      {
        return 0U;
      }
      index++;
      continue;
    }

    second_min = 0x80U;
    second_max = 0xBFU;
    if ((first >= 0xC2U) && (first <= 0xDFU))
    {
      continuation_count = 1U;
    }
    else if ((first >= 0xE0U) && (first <= 0xEFU))
    {
      continuation_count = 2U;
      if (first == 0xE0U)
      {
        second_min = 0xA0U;
      }
      else if (first == 0xEDU)
      {
        second_max = 0x9FU;
      }
    }
    else if ((first >= 0xF0U) && (first <= 0xF4U))
    {
      continuation_count = 3U;
      if (first == 0xF0U)
      {
        second_min = 0x90U;
      }
      else if (first == 0xF4U)
      {
        second_max = 0x8FU;
      }
    }
    else
    {
      return 0U;
    }

    if ((uint16_t)(index + continuation_count) >= length)
    {
      return 0U;
    }
    second = text[index + 1U];
    if ((second < second_min) || (second > second_max))
    {
      return 0U;
    }

    for (uint8_t offset = 2U; offset <= continuation_count; offset++)
    {
      if ((uint16_t)(index + offset) >= length ||
          (text[index + offset] < 0x80U) ||
          (text[index + offset] > 0xBFU))
      {
        return 0U;
      }
    }

    index = (uint16_t)(index + continuation_count + 1U);
  }

  return 1U;
}

static void AppendLubanCatPayloadByte(uint8_t byte, uint8_t *payload,
                                     uint16_t *length, uint8_t *overflowed)
{
  if (*length < JC014_QR_PAYLOAD_MAX_BYTES)
  {
    payload[*length] = byte;
    (*length)++;
  }
  else
  {
    *overflowed = 1U;
  }
}

static void PublishLubanCatPayload(uint8_t *payload, uint16_t *length,
                                   uint8_t *overflowed)
{
  if ((*length > 0U) && (*overflowed == 0U) &&
      (IsValidUtf8(payload, *length) != 0U))
  {
    payload[*length] = '\0';
    (void)xQueueOverwrite(lubanCatDisplayQueue, payload);
    TaskBasis_PublishPayload((const char *)payload);
  }

  *length = 0U;
  *overflowed = 0U;
}

static void StartLubanCatRxTask(void *argument)
{
  uint8_t payload[LUBANCAT_PAYLOAD_BUFFER_SIZE];
  uint16_t payload_length = 0U;
  uint8_t overflowed = 0U;
  uint8_t character;
  uint8_t trailing_55 = 0U;
  LubanCatRxState_t state = LUBANCAT_WAIT_HEADER_AA;

  (void)argument;
  configASSERT(HAL_UART_Receive_IT(&huart1, &lubanCatRxByte, 1U) == HAL_OK);

  for (;;)
  {
    if (xQueueReceive(lubanCatRxQueue, &character, portMAX_DELAY) == pdPASS)
    {
      if (state == LUBANCAT_WAIT_HEADER_AA)
      {
        if (character == 0xAAU)
        {
          state = LUBANCAT_WAIT_HEADER_55;
        }
      }
      else if (state == LUBANCAT_WAIT_HEADER_55)
      {
        if (character == 0x55U)
        {
          state = LUBANCAT_IN_PAYLOAD;
          payload_length = 0U;
          overflowed = 0U;
          trailing_55 = 0U;
        }
        else if (character != 0xAAU)
        {
          state = LUBANCAT_WAIT_HEADER_AA;
        }
      }
      else if (trailing_55 != 0U)
      {
        if (character == 0xAAU)
        {
          PublishLubanCatPayload(payload, &payload_length, &overflowed);
          state = LUBANCAT_WAIT_HEADER_AA;
          trailing_55 = 0U;
        }
        else
        {
          AppendLubanCatPayloadByte(0x55U, payload,
                                    &payload_length, &overflowed);
          trailing_55 = (character == 0x55U) ? 1U : 0U;
          if (trailing_55 == 0U)
          {
            AppendLubanCatPayloadByte(character, payload,
                                      &payload_length, &overflowed);
          }
        }
      }
      else if (character == 0x55U)
      {
        trailing_55 = 1U;
      }
      else
      {
        AppendLubanCatPayloadByte(character, payload,
                                  &payload_length, &overflowed);
      }
    }
  }
}

static void StartLubanCatDisplayTask(void *argument)
{
  char payload[LUBANCAT_PAYLOAD_BUFFER_SIZE];

  (void)argument;
  configASSERT(xSemaphoreTake(jc014ReadySemaphore, portMAX_DELAY) == pdTRUE);
  for (;;)
  {
    if (xQueueReceive(lubanCatDisplayQueue, payload, portMAX_DELAY) == pdPASS)
    {
      (void)JC014_ShowQrPayload(payload);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == USART1)
  {
    (void)xQueueSendFromISR(lubanCatRxQueue, &lubanCatRxByte,
                            &higherPriorityTaskWoken);
    (void)HAL_UART_Receive_IT(&huart1, &lubanCatRxByte, 1U);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &lubanCatRxByte, 1U);
  }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
                                   StackType_t **stack,
                                   uint32_t *stackSize)
{
  *tcb = &idleTaskTcb;
  *stack = idleTaskStack;
  *stackSize = configMINIMAL_STACK_SIZE;
}
