
#ifndef __TASK_BASIS_H
#define __TASK_BASIS_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

typedef struct
{
  uint8_t key;
  uint8_t pressed;
  uint16_t reserved;
  uint32_t count;
} KeyEvent_t;

#define KEY_EVENT_QUEUE_LENGTH 16U

extern QueueHandle_t g_key_event_queue;

void TaskBasis_Init(void);
void TaskBasis_PublishPayload(const char *payload);
void StartOledTask(void *argument);
void StartKeyTask(void *argument);

#endif 
