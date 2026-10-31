#ifndef __JC014_H
#define __JC014_H

#include "main.h"

#define JC014_QR_PAYLOAD_MAX_BYTES 96U

typedef enum
{
  JC014_STATUS_WAITING = 0U,
  JC014_STATUS_OK,
  JC014_STATUS_NO_ACK,
  JC014_STATUS_UART_ERROR
} JC014_Status_t;

void JC014_Init(void);
HAL_StatusTypeDef JC014_SendCommand(const char *command, uint32_t wait_ms);
HAL_StatusTypeDef JC014_ClearScreen(void);
HAL_StatusTypeDef JC014_ShowStartupScreen(void);
HAL_StatusTypeDef JC014_ShowNumber(const char *text);
HAL_StatusTypeDef JC014_ShowQrPayload(const char *text);
JC014_Status_t JC014_GetStatus(void);

#endif
