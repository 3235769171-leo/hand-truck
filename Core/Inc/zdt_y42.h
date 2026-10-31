#ifndef __ZDT_Y42_H
#define __ZDT_Y42_H

#include "main.h"
#include <stdint.h>

#define Y42_DEFAULT_ADDRESS  0x01U
#define Y42_FIXED_TAIL       0x6BU
#define Y42_LAST_RX_MAX      16U

typedef enum
{
  Y42_FIRMWARE_X = 0U,
  Y42_FIRMWARE_EMM = 1U
} Y42_FirmwareMode_t;

typedef struct
{
  uint8_t sign;
  uint32_t raw;
  int32_t angle_x10;
} Y42_Position_t;

typedef struct
{
  uint16_t firmware_version;
  uint8_t hardware_series;
  uint8_t hardware_type;
  uint8_t hardware_version;
} Y42_Version_t;

extern volatile uint8_t g_y42_online;
extern volatile uint8_t g_y42_status_flags;
extern volatile Y42_Position_t g_y42_position;
extern volatile Y42_Version_t g_y42_version;

void Y42_Init(void);
void Y42_SetAddress(uint8_t address);
void Y42_SetFirmwareMode(Y42_FirmwareMode_t mode);
void Y42_SetResponseEnabled(uint8_t enabled);
uint8_t Y42_GetLastRx(uint8_t *buffer, uint8_t capacity,
                      HAL_StatusTypeDef *status);

HAL_StatusTypeDef Y42_SendRaw(const uint8_t *frame, uint16_t length);
HAL_StatusTypeDef Y42_SendAction(uint8_t function, uint8_t auxiliary);
HAL_StatusTypeDef Y42_SendEnable(uint8_t enable, uint8_t sync);
HAL_StatusTypeDef Y42_SendStop(uint8_t sync);
HAL_StatusTypeDef Y42_SendHome(uint8_t home_mode, uint8_t sync);
HAL_StatusTypeDef Y42_SendSpeedX(uint8_t direction, uint16_t acceleration,
                                  uint16_t speed_x10, uint8_t sync);
HAL_StatusTypeDef Y42_SendSpeedEmm(uint8_t direction, uint16_t speed_rpm,
                                    uint8_t acceleration, uint8_t sync);
HAL_StatusTypeDef Y42_SendPositionX(uint8_t direction, uint16_t speed_x10,
                                     uint32_t angle_x10, uint8_t mode,
                                     uint8_t sync);
HAL_StatusTypeDef Y42_SendPositionEmm(uint8_t direction, uint16_t speed_rpm,
                                       uint8_t acceleration, uint32_t pulses,
                                       uint8_t mode, uint8_t sync);

/* Per-device variants for motors sharing USART2 or USART3. */
HAL_StatusTypeDef Y42_SendEnableOn(UART_HandleTypeDef *uart, uint8_t address,
                                   uint8_t enable, uint8_t sync);
HAL_StatusTypeDef Y42_SendStopOn(UART_HandleTypeDef *uart, uint8_t address,
                                 uint8_t sync);
HAL_StatusTypeDef Y42_SendSpeedEmmOn(UART_HandleTypeDef *uart,
                                     uint8_t address, uint8_t direction,
                                     uint16_t speed_rpm,
                                     uint8_t acceleration, uint8_t sync);
HAL_StatusTypeDef Y42_SendPositionEmmOn(UART_HandleTypeDef *uart,
                                        uint8_t address, uint8_t direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration,
                                        uint32_t pulses, uint8_t mode,
                                        uint8_t sync);

HAL_StatusTypeDef Y42_ReadVersion(Y42_Version_t *version, uint32_t timeout);
HAL_StatusTypeDef Y42_ReadPosition(Y42_Position_t *position, uint32_t timeout);
HAL_StatusTypeDef Y42_ReadStatus(uint8_t *status_flags, uint32_t timeout);

#endif
