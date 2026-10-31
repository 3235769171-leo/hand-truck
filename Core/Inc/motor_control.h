#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include <stdint.h>

typedef enum
{
  MOTOR_ID_1 = 1U,
  MOTOR_ID_2,
  MOTOR_ID_3,
  MOTOR_ID_4,
  MOTOR_ID_5,
  MOTOR_ID_6,
  MOTOR_ID_7
} MotorId_t;

#define MOTOR_PULSE_MIN_HZ  1U
#define MOTOR_PULSE_MAX_HZ  20000U

void Motor_Init(void);
/* Motors on one UART bus need distinct configured addresses; defaults are 1-7. */
HAL_StatusTypeDef Motor_SetUartAddress(MotorId_t motor, uint8_t address);

HAL_StatusTypeDef Motor_UART_Enable(MotorId_t motor, uint8_t enable,
                                    uint8_t sync);
HAL_StatusTypeDef Motor_UART_SetSpeed(MotorId_t motor, uint8_t direction,
                                      uint16_t speed_rpm,
                                      uint8_t acceleration, uint8_t sync);
HAL_StatusTypeDef Motor_UART_MovePulses(MotorId_t motor, uint8_t direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration, uint32_t pulses,
                                        uint8_t mode, uint8_t sync);
HAL_StatusTypeDef Motor_UART_Stop(MotorId_t motor, uint8_t sync);

/* direction_level is the electrical level placed on the DIR pin. */
HAL_StatusTypeDef Motor_PulseMove(MotorId_t motor, uint8_t direction_level,
                                  uint32_t pulse_count,
                                  uint32_t pulse_hz);
HAL_StatusTypeDef Motor_PulseStop(MotorId_t motor);
uint8_t Motor_PulseIsBusy(MotorId_t motor);

#endif
