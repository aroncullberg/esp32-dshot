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
std::uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
BaseType_t xTaskNotifyGive(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t*);
