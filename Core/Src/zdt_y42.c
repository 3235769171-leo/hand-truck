#include "zdt_y42.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stddef.h>
#include <string.h>

#define Y42_UART_TIMEOUT 100U
#define Y42_ACK_TIMEOUT 20U

volatile uint8_t g_y42_online = 0U;
volatile uint8_t g_y42_status_flags = 0U;
volatile Y42_Position_t g_y42_position = {0U, 0U, 0};
volatile Y42_Version_t g_y42_version = {0U, 0U, 0U, 0U};

static uint8_t y42_address = Y42_DEFAULT_ADDRESS;
/* This project uses the EMM/Y42 V2.0 serial command layout by default. */
static Y42_FirmwareMode_t y42_firmware_mode = Y42_FIRMWARE_EMM;
static uint8_t y42_response_enabled = 1U;
static uint8_t y42_last_rx[Y42_LAST_RX_MAX];
static uint8_t y42_last_rx_length = 0U;
static HAL_StatusTypeDef y42_last_rx_status = HAL_TIMEOUT;
static SemaphoreHandle_t y42_uart2_mutex;
static SemaphoreHandle_t y42_uart3_mutex;
static StaticSemaphore_t y42_uart2_mutex_storage;
static StaticSemaphore_t y42_uart3_mutex_storage;

void Y42_Init(void)
{
  if (y42_uart2_mutex == NULL)
  {
    y42_uart2_mutex = xSemaphoreCreateMutexStatic(&y42_uart2_mutex_storage);
  }
  if (y42_uart3_mutex == NULL)
  {
    y42_uart3_mutex = xSemaphoreCreateMutexStatic(&y42_uart3_mutex_storage);
  }
}

static SemaphoreHandle_t Y42_GetUartMutex(UART_HandleTypeDef *uart)
{
  if (uart == &huart2)
  {
    return y42_uart2_mutex;
  }
  if (uart == &huart3)
  {
    return y42_uart3_mutex;
  }
  return NULL;
}

static void Y42_LockUart(UART_HandleTypeDef *uart)
{
  SemaphoreHandle_t mutex = Y42_GetUartMutex(uart);
  if (mutex != NULL)
  {
    (void)xSemaphoreTake(mutex, portMAX_DELAY);
  }
}

static void Y42_UnlockUart(UART_HandleTypeDef *uart)
{
  SemaphoreHandle_t mutex = Y42_GetUartMutex(uart);
  if (mutex != NULL)
  {
    (void)xSemaphoreGive(mutex);
  }
}

static void Y42_SaveLastRx(const uint8_t *data, uint16_t length,
                           HAL_StatusTypeDef status)
{
  uint16_t copy_length = length;

  taskENTER_CRITICAL();

  if (copy_length > Y42_LAST_RX_MAX)
  {
    copy_length = Y42_LAST_RX_MAX;
  }

  memset(y42_last_rx, 0, sizeof(y42_last_rx));
  if ((data != NULL) && (copy_length > 0U))
  {
    (void)memcpy(y42_last_rx, data, copy_length);
  }

  y42_last_rx_length = (uint8_t)copy_length;
  y42_last_rx_status = status;
  taskEXIT_CRITICAL();
}

static void PutU16BE(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)(value >> 8);
  buffer[1] = (uint8_t)value;
}

static void PutU32BE(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)(value >> 24);
  buffer[1] = (uint8_t)(value >> 16);
  buffer[2] = (uint8_t)(value >> 8);
  buffer[3] = (uint8_t)value;
}

static uint32_t GetU32BE(const uint8_t *buffer)
{
  return ((uint32_t)buffer[0] << 24) |
         ((uint32_t)buffer[1] << 16) |
         ((uint32_t)buffer[2] << 8) |
         (uint32_t)buffer[3];
}

static HAL_StatusTypeDef Y42_Exchange(const uint8_t *request,
                                      uint16_t request_length,
                                      uint8_t *response,
                                      uint16_t response_length,
                                      uint32_t timeout)
{
  HAL_StatusTypeDef status;

  if ((request == NULL) || (response == NULL) || (request_length == 0U) ||
      (response_length == 0U))
  {
    return HAL_ERROR;
  }

  Y42_LockUart(&huart3);

  status = HAL_UART_Transmit(&huart3, (uint8_t *)request,
                             request_length, Y42_UART_TIMEOUT);
  if (status != HAL_OK)
  {
    goto exit;
  }

  status = HAL_UART_Receive(&huart3, response, response_length, timeout);
  if (status == HAL_OK)
  {
    Y42_SaveLastRx(response, response_length, status);
  }
  else
  {
    Y42_SaveLastRx(NULL, 0U, status);
  }

  if (status != HAL_OK)
  {
    goto exit;
  }

  if ((response[0] != y42_address) ||
      (response[response_length - 1U] != Y42_FIXED_TAIL))
  {
    status = HAL_ERROR;
    goto exit;
  }

exit:
  Y42_UnlockUart(&huart3);
  return status;
}

static HAL_StatusTypeDef Y42_SendAndAck(const uint8_t *frame,
                                        uint16_t length,
                                        uint8_t function,
                                        uint32_t timeout)
{
  uint8_t response[4];
  HAL_StatusTypeDef status;

  if ((frame == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  Y42_LockUart(&huart3);

  status = HAL_UART_Transmit(&huart3, (uint8_t *)frame, length,
                             Y42_UART_TIMEOUT);
  if (status != HAL_OK)
  {
    goto exit;
  }

  if (y42_response_enabled == 0U)
  {
    Y42_SaveLastRx(NULL, 0U, HAL_TIMEOUT);
    goto exit;
  }

  status = HAL_UART_Receive(&huart3, response, sizeof(response), timeout);
  if (status == HAL_OK)
  {
    Y42_SaveLastRx(response, sizeof(response), status);
  }
  else
  {
    Y42_SaveLastRx(NULL, 0U, status);
  }

  if (status == HAL_TIMEOUT)
  {
    /*
     * The motor may be configured with Response=None. The command was
     * transmitted successfully, so a missing acknowledgement is not a
     * reason to suppress the motion command.
     */
    status = HAL_OK;
    goto exit;
  }

  if (status != HAL_OK)
  {
    goto exit;
  }

  if ((response[0] != y42_address) ||
      (response[1] != function) ||
      (response[3] != Y42_FIXED_TAIL))
  {
    status = HAL_ERROR;
    goto exit;
  }

  /* 0x02 = accepted, 0x12 = already at the requested home position. */
  status = ((response[2] == 0x02U) || (response[2] == 0x12U)) ?
           HAL_OK : HAL_ERROR;

exit:
  Y42_UnlockUart(&huart3);
  return status;
}

static HAL_StatusTypeDef Y42_SendAndAckOn(UART_HandleTypeDef *uart,
                                          uint8_t address,
                                          const uint8_t *frame,
                                          uint16_t length,
                                          uint8_t function,
                                          uint32_t timeout)
{
  uint8_t response[4];
  HAL_StatusTypeDef status;

  if (((uart != &huart2) && (uart != &huart3)) || (address == 0U) ||
      (frame == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  Y42_LockUart(uart);

  status = HAL_UART_Transmit(uart, (uint8_t *)frame, length,
                             Y42_UART_TIMEOUT);
  if (status != HAL_OK)
  {
    goto exit;
  }

  if (y42_response_enabled == 0U)
  {
    Y42_SaveLastRx(NULL, 0U, HAL_TIMEOUT);
    goto exit;
  }

  status = HAL_UART_Receive(uart, response, sizeof(response), timeout);
  if (status == HAL_OK)
  {
    Y42_SaveLastRx(response, sizeof(response), status);
  }
  else
  {
    Y42_SaveLastRx(NULL, 0U, status);
  }

  if (status == HAL_TIMEOUT)
  {
    status = HAL_OK;
    goto exit;
  }

  if (status != HAL_OK)
  {
    goto exit;
  }

  if ((response[0] != address) || (response[1] != function) ||
      (response[3] != Y42_FIXED_TAIL))
  {
    status = HAL_ERROR;
    goto exit;
  }

  status = ((response[2] == 0x02U) || (response[2] == 0x12U)) ?
           HAL_OK : HAL_ERROR;

exit:
  Y42_UnlockUart(uart);
  return status;
}

void Y42_SetAddress(uint8_t address)
{
  if (address != 0U)
  {
    y42_address = address;
  }
}

void Y42_SetFirmwareMode(Y42_FirmwareMode_t mode)
{
  y42_firmware_mode = mode;
}

void Y42_SetResponseEnabled(uint8_t enabled)
{
  y42_response_enabled = enabled ? 1U : 0U;
}

uint8_t Y42_GetLastRx(uint8_t *buffer, uint8_t capacity,
                      HAL_StatusTypeDef *status)
{
  uint8_t length;
  uint8_t copy_length;

  if ((buffer == NULL) && (capacity > 0U))
  {
    return 0U;
  }

  taskENTER_CRITICAL();

  length = y42_last_rx_length;
  copy_length = (length < capacity) ? length : capacity;
  if ((buffer != NULL) && (copy_length > 0U))
  {
    (void)memcpy(buffer, y42_last_rx, copy_length);
  }

  if (status != NULL)
  {
    *status = y42_last_rx_status;
  }

  taskEXIT_CRITICAL();
  return length;
}

HAL_StatusTypeDef Y42_SendRaw(const uint8_t *frame, uint16_t length)
{
  HAL_StatusTypeDef status;

  if ((frame == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  Y42_LockUart(&huart3);
  status = HAL_UART_Transmit(&huart3, (uint8_t *)frame, length,
                             Y42_UART_TIMEOUT);
  Y42_SaveLastRx(NULL, 0U, HAL_TIMEOUT);
  Y42_UnlockUart(&huart3);
  return status;
}

HAL_StatusTypeDef Y42_SendAction(uint8_t function, uint8_t auxiliary)
{
  uint8_t frame[4];

  frame[0] = y42_address;
  frame[1] = function;
  frame[2] = auxiliary;
  frame[3] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), function, Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendEnable(uint8_t enable, uint8_t sync)
{
  uint8_t frame[6];

  frame[0] = y42_address;
  frame[1] = 0xF3U;
  frame[2] = 0xABU;
  frame[3] = enable ? 1U : 0U;
  frame[4] = sync ? 1U : 0U;
  frame[5] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendStop(uint8_t sync)
{
  uint8_t frame[5];

  frame[0] = y42_address;
  frame[1] = 0xFEU;
  frame[2] = 0x98U;
  frame[3] = sync ? 1U : 0U;
  frame[4] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendHome(uint8_t home_mode, uint8_t sync)
{
  uint8_t frame[5];

  frame[0] = y42_address;
  frame[1] = 0x9AU;
  frame[2] = home_mode;
  frame[3] = sync ? 1U : 0U;
  frame[4] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendSpeedX(uint8_t direction, uint16_t acceleration,
                                  uint16_t speed_x10, uint8_t sync)
{
  uint8_t frame[9];

  frame[0] = y42_address;
  frame[1] = 0xF6U;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], acceleration);
  PutU16BE(&frame[5], speed_x10);
  frame[7] = sync ? 1U : 0U;
  frame[8] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendSpeedEmm(uint8_t direction, uint16_t speed_rpm,
                                    uint8_t acceleration, uint8_t sync)
{
  uint8_t frame[8];

  frame[0] = y42_address;
  frame[1] = 0xF6U;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], speed_rpm);
  frame[5] = acceleration;
  frame[6] = sync ? 1U : 0U;
  frame[7] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendPositionX(uint8_t direction, uint16_t speed_x10,
                                     uint32_t angle_x10, uint8_t mode,
                                     uint8_t sync)
{
  uint8_t frame[12];

  frame[0] = y42_address;
  frame[1] = 0xFBU;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], speed_x10);
  PutU32BE(&frame[5], angle_x10);
  frame[9] = mode;
  frame[10] = sync ? 1U : 0U;
  frame[11] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendPositionEmm(uint8_t direction, uint16_t speed_rpm,
                                       uint8_t acceleration, uint32_t pulses,
                                       uint8_t mode, uint8_t sync)
{
  uint8_t frame[13];

  frame[0] = y42_address;
  frame[1] = 0xFDU;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], speed_rpm);
  frame[5] = acceleration;
  PutU32BE(&frame[6], pulses);
  frame[10] = mode;
  frame[11] = sync ? 1U : 0U;
  frame[12] = Y42_FIXED_TAIL;
  return Y42_SendAndAck(frame, sizeof(frame), frame[1], Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendEnableOn(UART_HandleTypeDef *uart, uint8_t address,
                                   uint8_t enable, uint8_t sync)
{
  uint8_t frame[6];

  frame[0] = address;
  frame[1] = 0xF3U;
  frame[2] = 0xABU;
  frame[3] = enable ? 1U : 0U;
  frame[4] = sync ? 1U : 0U;
  frame[5] = Y42_FIXED_TAIL;
  return Y42_SendAndAckOn(uart, address, frame, sizeof(frame), frame[1],
                          Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendStopOn(UART_HandleTypeDef *uart, uint8_t address,
                                 uint8_t sync)
{
  uint8_t frame[5];

  frame[0] = address;
  frame[1] = 0xFEU;
  frame[2] = 0x98U;
  frame[3] = sync ? 1U : 0U;
  frame[4] = Y42_FIXED_TAIL;
  return Y42_SendAndAckOn(uart, address, frame, sizeof(frame), frame[1],
                          Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendSpeedEmmOn(UART_HandleTypeDef *uart,
                                     uint8_t address, uint8_t direction,
                                     uint16_t speed_rpm,
                                     uint8_t acceleration, uint8_t sync)
{
  uint8_t frame[8];

  frame[0] = address;
  frame[1] = 0xF6U;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], speed_rpm);
  frame[5] = acceleration;
  frame[6] = sync ? 1U : 0U;
  frame[7] = Y42_FIXED_TAIL;
  return Y42_SendAndAckOn(uart, address, frame, sizeof(frame), frame[1],
                          Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_SendPositionEmmOn(UART_HandleTypeDef *uart,
                                        uint8_t address, uint8_t direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration,
                                        uint32_t pulses, uint8_t mode,
                                        uint8_t sync)
{
  uint8_t frame[13];

  frame[0] = address;
  frame[1] = 0xFDU;
  frame[2] = direction ? 1U : 0U;
  PutU16BE(&frame[3], speed_rpm);
  frame[5] = acceleration;
  PutU32BE(&frame[6], pulses);
  frame[10] = mode;
  frame[11] = sync ? 1U : 0U;
  frame[12] = Y42_FIXED_TAIL;
  return Y42_SendAndAckOn(uart, address, frame, sizeof(frame), frame[1],
                          Y42_ACK_TIMEOUT);
}

HAL_StatusTypeDef Y42_ReadVersion(Y42_Version_t *version, uint32_t timeout)
{
  const uint8_t request[3] = {y42_address, 0x1FU, Y42_FIXED_TAIL};
  uint8_t response[7];
  HAL_StatusTypeDef status;

  if (version == NULL)
  {
    return HAL_ERROR;
  }

  status = Y42_Exchange(request, sizeof(request), response,
                        sizeof(response), timeout);
  if ((status != HAL_OK) || (response[1] != 0x1FU))
  {
    return HAL_ERROR;
  }

  version->firmware_version = (uint16_t)(((uint16_t)response[2] << 8) |
                                         response[3]);
  version->hardware_series = (uint8_t)(response[4] >> 4);
  version->hardware_type = (uint8_t)(response[4] & 0x0FU);
  version->hardware_version = response[5];
  return HAL_OK;
}

HAL_StatusTypeDef Y42_ReadPosition(Y42_Position_t *position, uint32_t timeout)
{
  const uint8_t request[3] = {y42_address, 0x36U, Y42_FIXED_TAIL};
  uint8_t response[8];
  HAL_StatusTypeDef status;

  if (position == NULL)
  {
    return HAL_ERROR;
  }

  status = Y42_Exchange(request, sizeof(request), response,
                        sizeof(response), timeout);
  if ((status != HAL_OK) || (response[1] != 0x36U))
  {
    return HAL_ERROR;
  }

  position->sign = response[2];
  position->raw = GetU32BE(&response[3]);

  if (y42_firmware_mode == Y42_FIRMWARE_X)
  {
    position->angle_x10 = (position->sign != 0U) ?
                          -(int32_t)(position->raw) :
                          (int32_t)(position->raw);
  }
  else
  {
    position->angle_x10 = (int32_t)(((uint64_t)position->raw * 3600ULL) / 65536ULL);
    if (position->sign != 0U)
    {
      position->angle_x10 = -position->angle_x10;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef Y42_ReadStatus(uint8_t *status_flags, uint32_t timeout)
{
  const uint8_t request[3] = {y42_address, 0x3AU, Y42_FIXED_TAIL};
  uint8_t response[4];
  HAL_StatusTypeDef status;

  if (status_flags == NULL)
  {
    return HAL_ERROR;
  }

  status = Y42_Exchange(request, sizeof(request), response,
                        sizeof(response), timeout);
  if ((status != HAL_OK) || (response[1] != 0x3AU))
  {
    return HAL_ERROR;
  }

  *status_flags = response[2];
  return HAL_OK;
}
