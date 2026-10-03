#pragma once
#include "freertos/FreeRTOS.h"
struct HostSemaphore;
using SemaphoreHandle_t = HostSemaphore*;
SemaphoreHandle_t xSemaphoreCreateBinary();
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t);
BaseType_t xSemaphoreGive(SemaphoreHandle_t);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t*);
void vSemaphoreDelete(SemaphoreHandle_t);
