#pragma once
#include "freertos/FreeRTOS.h"
using TaskHandle_t = void*;
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void*), const char*, unsigned, void*, unsigned, TaskHandle_t*, int);
void vTaskDelete(TaskHandle_t);
void vTaskDelay(TickType_t);
TickType_t xTaskGetTickCount();
BaseType_t xTaskDelayUntil(TickType_t*, TickType_t);
BaseType_t xTaskCreate(void (*entry)(void*), const char*, unsigned, void*, unsigned, TaskHandle_t*);
TaskHandle_t xTaskGetCurrentTaskHandle();
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
std::uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
BaseType_t xTaskNotifyGive(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*);
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
configRUN_TIME_COUNTER_TYPE ulTaskGetIdleRunTimeCounterForCore(BaseType_t);
struct TaskStatus_t {
    TaskHandle_t xHandle;
    const char* pcTaskName;
    UBaseType_t xTaskNumber;
    configRUN_TIME_COUNTER_TYPE ulRunTimeCounter;
    UBaseType_t usStackHighWaterMark = 4096;
};
UBaseType_t uxTaskGetSystemState(TaskStatus_t*, UBaseType_t, configRUN_TIME_COUNTER_TYPE*);
TaskHandle_t xTaskGetIdleTaskHandleForCore(BaseType_t);
#endif
