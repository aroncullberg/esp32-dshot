#pragma once
#include "freertos/FreeRTOS.h"
using QueueHandle_t = void*;
QueueHandle_t xQueueCreate(unsigned, unsigned);
BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t);
void vQueueDelete(QueueHandle_t);
#define pdMS_TO_TICKS(ms) (ms)
