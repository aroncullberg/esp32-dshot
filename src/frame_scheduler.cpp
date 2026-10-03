#include "frame_scheduler.hpp"

#include <cstdlib>
#include "esp_attr.h"
#include "esp_log.h"

namespace dshot::detail {
namespace {
void require_cleanup(esp_err_t error) {
    if (error != ESP_OK) {
        ESP_LOGE("dshot_timer", "Timer cleanup failed: %s", esp_err_to_name(error));
        std::abort();
    }
}
}

esp_err_t FrameScheduler::open(unsigned frame_hz) {
    if (timer_) return ESP_ERR_INVALID_STATE;
    if (frame_hz == 0 || frame_hz > 8000) return ESP_ERR_INVALID_ARG;
    constexpr std::uint32_t resolution_hz = 1'000'000;
    period_us_ = (resolution_hz + frame_hz / 2) / frame_hz;
    gptimer_config_t config{};
    config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    config.direction = GPTIMER_COUNT_UP;
    config.resolution_hz = resolution_hz;
    auto error = gptimer_new_timer(&config, &timer_);
    if (error != ESP_OK) return error;

    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = on_alarm;
    error = gptimer_register_event_callbacks(timer_, &callbacks, this);
    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = period_us_;
    alarm.flags.auto_reload_on_alarm = true;
    if (error == ESP_OK) error = gptimer_set_alarm_action(timer_, &alarm);
    if (error == ESP_OK) {
        error = gptimer_enable(timer_);
        if (error == ESP_OK) enabled_ = true;
        else {
            // gptimer_enable() can fail after enabling the timer.
            const auto cleanup = gptimer_disable(timer_);
            if (cleanup != ESP_ERR_INVALID_STATE) require_cleanup(cleanup);
        }
    }
    if (error != ESP_OK) {
        close();
        return error;
    }
    return ESP_OK;
}

esp_err_t FrameScheduler::start() {
    portENTER_CRITICAL(&wake_lock_);
    wake_task_ = xTaskGetCurrentTaskHandle();
    portEXIT_CRITICAL(&wake_lock_);
    const auto error = gptimer_start(timer_);
    running_ = error == ESP_OK;
    return error;
}

bool IRAM_ATTR FrameScheduler::on_alarm(gptimer_handle_t,
                                       const gptimer_alarm_event_data_t*, void* context) {
    auto& self = *static_cast<FrameScheduler*>(context);
    BaseType_t woken = pdFALSE;
    portENTER_CRITICAL_ISR(&self.wake_lock_);
    if (self.wake_task_) vTaskNotifyGiveFromISR(self.wake_task_, &woken);
    portEXIT_CRITICAL_ISR(&self.wake_lock_);
    return woken == pdTRUE;
}

void FrameScheduler::wake() {
    portENTER_CRITICAL(&wake_lock_);
    if (wake_task_) xTaskNotifyGive(wake_task_);
    portEXIT_CRITICAL(&wake_lock_);
}

void FrameScheduler::close() {
    portENTER_CRITICAL(&wake_lock_);
    wake_task_ = nullptr;
    portEXIT_CRITICAL(&wake_lock_);
    if (running_) {
        require_cleanup(gptimer_stop(timer_));
        running_ = false;
    }
    if (enabled_) {
        require_cleanup(gptimer_disable(timer_));
        enabled_ = false;
    }
    if (timer_) {
        require_cleanup(gptimer_del_timer(timer_));
        timer_ = nullptr;
    }
}
}
