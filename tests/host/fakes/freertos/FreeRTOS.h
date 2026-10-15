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
#define configMAX_TASK_NAME_LEN 16
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define tskNO_AFFINITY (-1)
#define pdMS_TO_TICKS(ms) ((ms) * configTICK_RATE_HZ / 1000)
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#if CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64
#define configRUN_TIME_COUNTER_TYPE std::uint64_t
#else
#define configRUN_TIME_COUNTER_TYPE std::uint32_t
#endif
configRUN_TIME_COUNTER_TYPE xPortGetRunTimeCounterValue();
#define portGET_RUN_TIME_COUNTER_VALUE() xPortGetRunTimeCounterValue()
#endif
