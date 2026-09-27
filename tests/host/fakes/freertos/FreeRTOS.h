#pragma once
#include <cstdint>
#include <mutex>
#include "sdkconfig.h"
using TickType_t = std::uint32_t;
using BaseType_t = int;
using UBaseType_t = unsigned;
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(mux) (mux)->lock()
#define portEXIT_CRITICAL(mux) (mux)->unlock()
#define portENTER_CRITICAL_ISR(mux) (mux)->lock()
#define portEXIT_CRITICAL_ISR(mux) (mux)->unlock()
#define configTICK_RATE_HZ CONFIG_FREERTOS_HZ
#define configMAX_PRIORITIES 25
#define configNUMBER_OF_CORES CONFIG_FREERTOS_NUMBER_OF_CORES
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define tskNO_AFFINITY (-1)
