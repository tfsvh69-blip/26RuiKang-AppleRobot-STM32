#ifndef __SC16_TASKS_H__
#define __SC16_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"

extern SemaphoreHandle_t g_sc16RxSemA;
extern SemaphoreHandle_t g_sc16RxSemB;
extern EventGroupHandle_t g_sc16EventGroup;

#define SC16_EVENT_IRQ_TRIGGERED  (1u << 0)

void StartSc16RxATask(void *argument);
void StartSc16RxBTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __SC16_TASKS_H__ */
