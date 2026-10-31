#ifndef JC014_HOST_PLATFORM_H
#define JC014_HOST_PLATFORM_H

/* Replace only the MCU/RTOS boundary while compiling the real driver. */
#define __MAIN_H
#define INC_FREERTOS_H
#define SEMAPHORE_H
#define INC_TASK_H

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { unsigned int instance; } UART_HandleTypeDef;
typedef void *SemaphoreHandle_t;

#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define configASSERT(condition) assert(condition)

SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout);
int xSemaphoreGive(SemaphoreHandle_t mutex);
void vTaskDelay(uint32_t ticks);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                    uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t length, uint32_t timeout);

#endif
