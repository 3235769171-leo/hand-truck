#include "motor_control.h"
#include "tim.h"
#include "usart.h"
#include "zdt_y42.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define MOTOR_COUNT 7U
#define MOTOR_GROUP_COUNT 2U

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint8_t group;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  UART_HandleTypeDef *uart;
  uint8_t default_address;
} MotorRoute_t;

static const MotorRoute_t motor_routes[MOTOR_COUNT] =
{
  {&htim1, TIM_CHANNEL_2, 0U, DIR1_GPIO_Port, DIR1_Pin, &huart3, 1U},
  {&htim1, TIM_CHANNEL_3, 0U, DIR2_GPIO_Port, DIR2_Pin, &huart3, 2U},
  {&htim1, TIM_CHANNEL_4, 0U, DIR3_GPIO_Port, DIR3_Pin, &huart3, 3U},
  {&htim1, TIM_CHANNEL_1, 0U, DIR4_GPIO_Port, DIR4_Pin, &huart3, 4U},
  {&htim2, TIM_CHANNEL_1, 1U, DIR5_GPIO_Port, DIR5_Pin, &huart2, 5U},
  {&htim2, TIM_CHANNEL_2, 1U, DIR6_GPIO_Port, DIR6_Pin, &huart2, 6U},
  {&htim2, TIM_CHANNEL_4, 1U, DIR7_GPIO_Port, DIR7_Pin, &huart2, 7U}
};

static SemaphoreHandle_t motor_mutex;
static StaticSemaphore_t motor_mutex_storage;
static uint8_t motor_initialized;
static uint8_t motor_uart_addresses[MOTOR_COUNT] = {1U, 2U, 3U, 4U, 5U, 6U, 7U};
static volatile uint32_t motor_remaining[MOTOR_COUNT];
static volatile uint8_t group_active[MOTOR_GROUP_COUNT];
static volatile uint8_t group_pending[MOTOR_GROUP_COUNT];
static volatile uint32_t group_frequency[MOTOR_GROUP_COUNT];
static volatile int8_t group_tick_owner[MOTOR_GROUP_COUNT] = {-1, -1};
static volatile uint8_t group_owner_switch_pending[MOTOR_GROUP_COUNT];

static uint32_t Motor_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return primask;
}

static void Motor_ExitCritical(uint32_t primask)
{
  __DMB();
  __set_PRIMASK(primask);
}

static uint8_t Motor_GetIndex(MotorId_t motor, uint8_t *index)
{
  if ((motor < MOTOR_ID_1) || (motor > MOTOR_ID_7) || (index == NULL))
  {
    return 0U;
  }

  *index = (uint8_t)(motor - MOTOR_ID_1);
  return 1U;
}

static uint32_t Motor_GetChannelInterrupt(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1: return TIM_IT_CC1;
    case TIM_CHANNEL_2: return TIM_IT_CC2;
    case TIM_CHANNEL_3: return TIM_IT_CC3;
    case TIM_CHANNEL_4: return TIM_IT_CC4;
    default: return 0U;
  }
}

static uint32_t Motor_GetChannelFlag(uint32_t channel)
{
  switch (channel)
  {
    case TIM_CHANNEL_1: return TIM_FLAG_CC1;
    case TIM_CHANNEL_2: return TIM_FLAG_CC2;
    case TIM_CHANNEL_3: return TIM_FLAG_CC3;
    case TIM_CHANNEL_4: return TIM_FLAG_CC4;
    default: return 0U;
  }
}

static void Motor_StopPwmChannel(TIM_HandleTypeDef *timer, uint32_t channel)
{
  uint32_t interrupt = Motor_GetChannelInterrupt(channel);
  uint32_t flag = Motor_GetChannelFlag(channel);

  if (interrupt != 0U)
  {
    __HAL_TIM_DISABLE_IT(timer, interrupt);
  }
  TIM_CCxChannelCmd(timer->Instance, channel, TIM_CCx_DISABLE);
  if (flag != 0U)
  {
    __HAL_TIM_CLEAR_FLAG(timer, flag);
  }
  TIM_CHANNEL_STATE_SET(timer, channel, HAL_TIM_CHANNEL_STATE_READY);
}

static void Motor_SelectTickOwner(uint8_t group)
{
  int8_t owner = group_tick_owner[group];
  uint8_t index;
  uint8_t bit;
  uint32_t interrupt;
  uint32_t flag;
  TIM_HandleTypeDef *timer = (group == 0U) ? &htim1 : &htim2;

  if ((owner >= 0) &&
      ((group_active[group] & (uint8_t)(1U << owner)) != 0U))
  {
    return;
  }

  group_tick_owner[group] = -1;
  if (owner >= 0)
  {
    interrupt = Motor_GetChannelInterrupt(motor_routes[(uint8_t)owner].channel);
    if (interrupt != 0U)
    {
      __HAL_TIM_DISABLE_IT(timer, interrupt);
    }
  }

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    bit = (uint8_t)(1U << index);
    if ((motor_routes[index].group == group) &&
        ((group_active[group] & bit) != 0U))
    {
      interrupt = Motor_GetChannelInterrupt(motor_routes[index].channel);
      flag = Motor_GetChannelFlag(motor_routes[index].channel);
      if ((interrupt != 0U) && (flag != 0U))
      {
        __HAL_TIM_CLEAR_FLAG(timer, flag);
        __HAL_TIM_ENABLE_IT(timer, interrupt);
        group_tick_owner[group] = (int8_t)index;
      }
      return;
    }
  }
}

static void Motor_StartPending(uint8_t group, TIM_HandleTypeDef *timer)
{
  uint8_t index;
  uint8_t bit;
  uint8_t pending = group_pending[group];
  const MotorRoute_t *route;

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    bit = (uint8_t)(1U << index);
    if ((pending & bit) != 0U)
    {
      route = &motor_routes[index];
      group_pending[group] &= (uint8_t)~bit;
      if (HAL_TIM_PWM_Start(timer, route->channel) == HAL_OK)
      {
        group_active[group] |= bit;
      }
      else
      {
        motor_remaining[index] = 0U;
      }
    }
  }

}

static uint8_t Motor_ComputeTiming(TIM_HandleTypeDef *timer, uint32_t pulse_hz,
                                   uint16_t *prescaler, uint32_t *period,
                                   uint32_t *pulse_width)
{
  RCC_ClkInitTypeDef clocks;
  uint32_t flash_latency;
  uint32_t timer_clock;
  uint64_t prescaler_divider;
  uint64_t period_counts;
  uint64_t denominator;

  if ((timer == NULL) || (prescaler == NULL) || (period == NULL) ||
      (pulse_width == NULL) || (pulse_hz < MOTOR_PULSE_MIN_HZ) ||
      (pulse_hz > MOTOR_PULSE_MAX_HZ))
  {
    return 0U;
  }

  HAL_RCC_GetClockConfig(&clocks, &flash_latency);
  if (timer->Instance == TIM1)
  {
    timer_clock = HAL_RCC_GetPCLK2Freq();
    if (clocks.APB2CLKDivider != RCC_HCLK_DIV1)
    {
      timer_clock *= 2U;
    }
  }
  else
  {
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1)
    {
      timer_clock *= 2U;
    }
  }

  prescaler_divider = ((uint64_t)timer_clock +
                       ((uint64_t)pulse_hz * 65536ULL) - 1ULL) /
                      ((uint64_t)pulse_hz * 65536ULL);
  if (prescaler_divider == 0ULL)
  {
    prescaler_divider = 1ULL;
  }
  if (prescaler_divider > 65536ULL)
  {
    return 0U;
  }

  denominator = (uint64_t)pulse_hz * prescaler_divider;
  period_counts = ((uint64_t)timer_clock + denominator / 2ULL) / denominator;
  if ((period_counts < 2ULL) || (period_counts > 65536ULL))
  {
    return 0U;
  }

  *prescaler = (uint16_t)(prescaler_divider - 1ULL);
  *period = (uint32_t)(period_counts - 1ULL);
  *pulse_width = (uint32_t)(period_counts / 2ULL);
  return 1U;
}

static void Motor_ConfigureTimer(TIM_HandleTypeDef *timer, uint16_t prescaler,
                                 uint32_t period)
{
  __HAL_TIM_SET_PRESCALER(timer, prescaler);
  __HAL_TIM_SET_AUTORELOAD(timer, period);
  __HAL_TIM_SET_COUNTER(timer, 0U);
  timer->Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
}

void Motor_Init(void)
{
  uint8_t index;

  if (motor_initialized != 0U)
  {
    return;
  }

  if (motor_mutex == NULL)
  {
    motor_mutex = xSemaphoreCreateMutexStatic(&motor_mutex_storage);
  }

  Y42_Init();

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    motor_uart_addresses[index] = motor_routes[index].default_address;
    motor_remaining[index] = 0U;
    HAL_GPIO_WritePin(motor_routes[index].dir_port,
                      motor_routes[index].dir_pin, GPIO_PIN_RESET);
  }

  group_active[0] = 0U;
  group_active[1] = 0U;
  group_pending[0] = 0U;
  group_pending[1] = 0U;
  group_frequency[0] = 0U;
  group_frequency[1] = 0U;
  group_tick_owner[0] = -1;
  group_tick_owner[1] = -1;
  group_owner_switch_pending[0] = 0U;
  group_owner_switch_pending[1] = 0U;

  /* These handlers never call FreeRTOS APIs, so keep pulse counts unmasked. */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  HAL_NVIC_SetPriority(TIM2_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  motor_initialized = 1U;
}

static HAL_StatusTypeDef Motor_GetUartTarget(MotorId_t motor,
                                             UART_HandleTypeDef **uart,
                                             uint8_t *address)
{
  uint8_t index;

  if ((!Motor_GetIndex(motor, &index)) || (uart == NULL) ||
      (address == NULL) || (motor_mutex == NULL))
  {
    return HAL_ERROR;
  }

  if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE)
  {
    return HAL_ERROR;
  }
  *uart = motor_routes[index].uart;
  *address = motor_uart_addresses[index];
  (void)xSemaphoreGive(motor_mutex);
  return HAL_OK;
}

HAL_StatusTypeDef Motor_SetUartAddress(MotorId_t motor, uint8_t address)
{
  uint8_t index;

  if ((!Motor_GetIndex(motor, &index)) || (address == 0U) ||
      (motor_mutex == NULL))
  {
    return HAL_ERROR;
  }
  if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE)
  {
    return HAL_ERROR;
  }
  motor_uart_addresses[index] = address;
  (void)xSemaphoreGive(motor_mutex);
  return HAL_OK;
}

HAL_StatusTypeDef Motor_UART_Enable(MotorId_t motor, uint8_t enable,
                                    uint8_t sync)
{
  UART_HandleTypeDef *uart;
  uint8_t address;
  HAL_StatusTypeDef status = Motor_GetUartTarget(motor, &uart, &address);
  return (status == HAL_OK) ? Y42_SendEnableOn(uart, address, enable, sync) :
                             status;
}

HAL_StatusTypeDef Motor_UART_SetSpeed(MotorId_t motor, uint8_t direction,
                                      uint16_t speed_rpm,
                                      uint8_t acceleration, uint8_t sync)
{
  UART_HandleTypeDef *uart;
  uint8_t address;
  HAL_StatusTypeDef status;

  if (direction > 1U)
  {
    return HAL_ERROR;
  }
  status = Motor_GetUartTarget(motor, &uart, &address);
  return (status == HAL_OK) ?
         Y42_SendSpeedEmmOn(uart, address, direction, speed_rpm,
                            acceleration, sync) : status;
}

HAL_StatusTypeDef Motor_UART_MovePulses(MotorId_t motor, uint8_t direction,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration, uint32_t pulses,
                                        uint8_t mode, uint8_t sync)
{
  UART_HandleTypeDef *uart;
  uint8_t address;
  HAL_StatusTypeDef status;

  if (direction > 1U)
  {
    return HAL_ERROR;
  }
  status = Motor_GetUartTarget(motor, &uart, &address);
  return (status == HAL_OK) ?
         Y42_SendPositionEmmOn(uart, address, direction, speed_rpm,
                               acceleration, pulses, mode, sync) : status;
}

HAL_StatusTypeDef Motor_UART_Stop(MotorId_t motor, uint8_t sync)
{
  UART_HandleTypeDef *uart;
  uint8_t address;
  HAL_StatusTypeDef status = Motor_GetUartTarget(motor, &uart, &address);
  return (status == HAL_OK) ? Y42_SendStopOn(uart, address, sync) : status;
}

HAL_StatusTypeDef Motor_PulseMove(MotorId_t motor, uint8_t direction_level,
                                  uint32_t pulse_count, uint32_t pulse_hz)
{
  uint8_t index;
  uint8_t bit;
  uint8_t group;
  uint16_t prescaler;
  uint32_t period;
  uint32_t pulse_width;
  uint32_t primask;
  HAL_StatusTypeDef status = HAL_OK;
  const MotorRoute_t *route;

  if ((!Motor_GetIndex(motor, &index)) || (direction_level > 1U) ||
      (pulse_count == 0U) || (motor_mutex == NULL))
  {
    return HAL_ERROR;
  }
  if (!Motor_ComputeTiming(motor_routes[index].timer, pulse_hz,
                           &prescaler, &period, &pulse_width))
  {
    return HAL_ERROR;
  }
  if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE)
  {
    return HAL_ERROR;
  }

  route = &motor_routes[index];
  group = route->group;
  bit = (uint8_t)(1U << index);
  primask = Motor_EnterCritical();

  if (((group_active[group] | group_pending[group]) & bit) != 0U)
  {
    status = HAL_BUSY;
  }
  else if (((group_active[group] | group_pending[group]) != 0U) &&
           (group_frequency[group] != pulse_hz))
  {
    status = HAL_BUSY;
  }
  else
  {
    if ((group_active[group] | group_pending[group]) == 0U)
    {
      Motor_ConfigureTimer(route->timer, prescaler, period);
      group_frequency[group] = pulse_hz;
      __HAL_TIM_SET_COMPARE(route->timer, route->channel, pulse_width);
      __HAL_TIM_SET_COUNTER(route->timer, 0U);
      __HAL_TIM_CLEAR_FLAG(route->timer, TIM_FLAG_UPDATE);
      motor_remaining[index] = pulse_count;
      HAL_GPIO_WritePin(route->dir_port, route->dir_pin,
                        direction_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
      __HAL_TIM_CLEAR_FLAG(route->timer,
                           Motor_GetChannelFlag(route->channel));
      status = HAL_TIM_PWM_Start_IT(route->timer, route->channel);
      if (status == HAL_OK)
      {
        __HAL_TIM_SET_COUNTER(route->timer, 0U);
        __HAL_TIM_CLEAR_FLAG(route->timer, TIM_FLAG_UPDATE);
        group_active[group] |= bit;
        group_tick_owner[group] = (int8_t)index;
      }
      else
      {
        motor_remaining[index] = 0U;
        group_frequency[group] = 0U;
        group_tick_owner[group] = -1;
      }
    }
    else
    {
      __HAL_TIM_SET_COMPARE(route->timer, route->channel, pulse_width);
      motor_remaining[index] = pulse_count;
      HAL_GPIO_WritePin(route->dir_port, route->dir_pin,
                        direction_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
      group_pending[group] |= bit;
      __HAL_TIM_CLEAR_FLAG(route->timer, TIM_FLAG_UPDATE);
      __HAL_TIM_ENABLE_IT(route->timer, TIM_IT_UPDATE);
    }
  }

  Motor_ExitCritical(primask);
  (void)xSemaphoreGive(motor_mutex);
  return status;
}

HAL_StatusTypeDef Motor_PulseStop(MotorId_t motor)
{
  uint8_t index;
  uint8_t bit;
  uint8_t group;
  uint8_t mask;
  uint32_t primask;
  const MotorRoute_t *route;

  if ((!Motor_GetIndex(motor, &index)) || (motor_mutex == NULL))
  {
    return HAL_ERROR;
  }
  if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE)
  {
    return HAL_ERROR;
  }

  route = &motor_routes[index];
  group = route->group;
  bit = (uint8_t)(1U << index);
  primask = Motor_EnterCritical();
  if ((group_active[group] & bit) != 0U)
  {
    Motor_StopPwmChannel(route->timer, route->channel);
    if (group_tick_owner[group] == (int8_t)index)
    {
      group_tick_owner[group] = -1;
    }
    group_active[group] &= (uint8_t)~bit;
  }
  group_pending[group] &= (uint8_t)~bit;
  motor_remaining[index] = 0U;
  Motor_SelectTickOwner(group);
  mask = (uint8_t)(group_active[group] | group_pending[group]);
  if (mask == 0U)
  {
    __HAL_TIM_DISABLE_IT(route->timer, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(route->timer, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE(route->timer);
    group_tick_owner[group] = -1;
    group_frequency[group] = 0U;
  }
  Motor_ExitCritical(primask);
  (void)xSemaphoreGive(motor_mutex);
  return HAL_OK;
}

uint8_t Motor_PulseIsBusy(MotorId_t motor)
{
  uint8_t index;
  uint8_t busy;
  uint8_t group;
  uint8_t bit;
  uint32_t primask;

  if (!Motor_GetIndex(motor, &index))
  {
    return 0U;
  }
  group = motor_routes[index].group;
  bit = (uint8_t)(1U << index);
  primask = Motor_EnterCritical();
  busy = (((group_active[group] | group_pending[group]) & bit) != 0U) ?
         1U : 0U;
  Motor_ExitCritical(primask);
  return busy;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  uint8_t group;
  uint8_t mask;

  if ((timer->Instance == TIM1) && (timer == &htim1))
  {
    group = 0U;
  }
  else if ((timer->Instance == TIM2) && (timer == &htim2))
  {
    group = 1U;
  }
  else
  {
    return;
  }

  Motor_StartPending(group, timer);

  if ((group_owner_switch_pending[group] != 0U) ||
      (group_tick_owner[group] < 0))
  {
    group_owner_switch_pending[group] = 0U;
    Motor_SelectTickOwner(group);
  }

  mask = (uint8_t)(group_active[group] | group_pending[group]);
  if (mask == 0U)
  {
    __HAL_TIM_DISABLE_IT(timer, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE(timer);
    group_tick_owner[group] = -1;
    group_frequency[group] = 0U;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer)
{
  uint8_t group;
  uint8_t index;
  uint8_t bit;
  uint32_t channel;
  const MotorRoute_t *route;

  if ((timer == &htim1) && (timer->Instance == TIM1))
  {
    group = 0U;
  }
  else if ((timer == &htim2) && (timer->Instance == TIM2))
  {
    group = 1U;
  }
  else
  {
    return;
  }

  switch (timer->Channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1: channel = TIM_CHANNEL_1; break;
    case HAL_TIM_ACTIVE_CHANNEL_2: channel = TIM_CHANNEL_2; break;
    case HAL_TIM_ACTIVE_CHANNEL_3: channel = TIM_CHANNEL_3; break;
    case HAL_TIM_ACTIVE_CHANNEL_4: channel = TIM_CHANNEL_4; break;
    default: return;
  }

  if (group_tick_owner[group] < 0)
  {
    return;
  }
  index = (uint8_t)group_tick_owner[group];
  if (motor_routes[index].channel != channel)
  {
    return;
  }

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    route = &motor_routes[index];
    bit = (uint8_t)(1U << index);
    if ((route->group == group) && ((group_active[group] & bit) != 0U))
    {
      if (motor_remaining[index] > 0U)
      {
        motor_remaining[index]--;
      }
      if (motor_remaining[index] == 0U)
      {
        Motor_StopPwmChannel(timer, route->channel);
        if (group_tick_owner[group] == (int8_t)index)
        {
          group_tick_owner[group] = -1;
        }
        group_active[group] &= (uint8_t)~bit;
      }
    }
  }

  if (group_tick_owner[group] < 0)
  {
    if (group_active[group] != 0U)
    {
      group_owner_switch_pending[group] = 1U;
      __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
      __HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
    }
  }
  if ((group_active[group] | group_pending[group]) == 0U)
  {
    __HAL_TIM_DISABLE_IT(timer, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE(timer);
    group_tick_owner[group] = -1;
    group_owner_switch_pending[group] = 0U;
    group_frequency[group] = 0U;
  }
}
