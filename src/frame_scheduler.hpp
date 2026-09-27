#pragma once

#include <cstdint>
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace dshot::detail {
// One timer per driver. Open before launching the worker; start/close in the
// worker. After worker exit, the owner may open it again. wake() is concurrent.
class FrameScheduler {
public:
    FrameScheduler() = default;
    FrameScheduler(const FrameScheduler&) = delete;
    FrameScheduler& operator=(const FrameScheduler&) = delete;
    esp_err_t open(unsigned frame_hz);
    esp_err_t start();
    void close();
    void wake();
    std::uint32_t period_us() const { return period_us_; }

private:
    static bool on_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
    gptimer_handle_t timer_ = nullptr;
    bool enabled_ = false;
    bool running_ = false;
    std::uint32_t period_us_ = 0;
    portMUX_TYPE wake_lock_ = portMUX_INITIALIZER_UNLOCKED;
    TaskHandle_t wake_task_ = nullptr;
};
}
