#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "config_validation.hpp"
#include "frame_scheduler.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <new>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

namespace dshot {
namespace {
constexpr const char* kTag = "dshot";
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
constexpr bool kDetailedTiming = true;
void record(TimingStatistics& timing, std::uint64_t elapsed_us) {
    if (timing.count == 0 || elapsed_us < timing.min_us) timing.min_us = elapsed_us;
    if (elapsed_us > timing.max_us) timing.max_us = elapsed_us;
    ++timing.count;
    timing.total_us += elapsed_us;
}
#else
constexpr bool kDetailedTiming = false;
#endif
#if CONFIG_IDF_TARGET_ESP32C3
constexpr std::size_t kMaxLinks = 2;
#elif CONFIG_IDF_TARGET_ESP32S3
constexpr std::size_t kMaxLinks = 4;
#else
#error Unsupported DShot target
#endif
}

struct Driver::Impl {
    std::array<Link*, kMaxLinks> links{};
    std::size_t link_count = 0;
    detail::FrameScheduler scheduler;
    SemaphoreHandle_t worker_exited = nullptr;
    bool join_pending = false;
    std::atomic<bool> stop_requested{false};
    std::atomic<esp_err_t> last_error{ESP_OK};
    portMUX_TYPE statistics_lock = portMUX_INITIALIZER_UNLOCKED;
    DriverStatistics statistics;
    std::int64_t started_at_us = 0;
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    std::array<std::int64_t, kMaxLinks> last_completed_us{};
#endif
};

Driver::Driver(const Config& config) : config_(config) {}
Driver::~Driver() {
    stop();
    if (!impl_) return;
    if (impl_->link_count != 0) {
        ESP_LOGE(kTag, "Driver must outlive its links");
        std::abort();
    }
    if (impl_->worker_exited) vSemaphoreDelete(impl_->worker_exited);
    delete impl_;
}

esp_err_t Driver::prepare_link(const Link& link) {
    if (passthrough_owner_) return ESP_ERR_INVALID_STATE;
    const auto error = validate_config(config_);
    if (error != ESP_OK) return error;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(link.pin_)) return ESP_ERR_INVALID_ARG;
    if (link.impl_) return ESP_ERR_INVALID_STATE;
    if (!impl_) {
        impl_ = new (std::nothrow) Impl;
        if (!impl_) return ESP_ERR_NO_MEM;
    }
    if (impl_->join_pending) return ESP_ERR_INVALID_STATE;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (impl_->links[i] == &link || impl_->links[i]->pin_ == link.pin_) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    return impl_->link_count == kMaxLinks ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t Driver::validate_passthrough_links(Link* const* links, std::size_t count) const {
    if (!links || count == 0 || count > kMaxLinks) return ESP_ERR_INVALID_ARG;
    for (std::size_t i = 0; i < count; ++i) {
        if (!links[i] || &links[i]->driver_ != this) return ESP_ERR_INVALID_ARG;
        for (std::size_t j = 0; j < i; ++j) {
            if (links[j] == links[i] || links[j]->pin_ == links[i]->pin_)
                return ESP_ERR_INVALID_ARG;
        }
    }
    if (impl_) {
        for (std::size_t i = 0; i < impl_->link_count; ++i) {
            bool included = false;
            for (std::size_t j = 0; j < count; ++j)
                if (impl_->links[i] == links[j]) included = true;
            if (!included) return ESP_ERR_INVALID_STATE;
        }
    }
    return ESP_OK;
}

esp_err_t Driver::attach_link(Link& link) {
    if (!impl_ || impl_->join_pending || !link.ready()) return ESP_ERR_INVALID_STATE;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (impl_->links[i] == &link || impl_->links[i]->pin_ == link.pin_) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (impl_->link_count == kMaxLinks) return ESP_ERR_NOT_FOUND;
    impl_->links[impl_->link_count++] = &link;
    return ESP_OK;
}

void Driver::detach_link(Link& link) {
    if (!impl_) return;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (impl_->links[i] != &link) continue;
        for (std::size_t j = i + 1; j < impl_->link_count; ++j) {
            impl_->links[j - 1] = impl_->links[j];
        }
        impl_->links[--impl_->link_count] = nullptr;
        return;
    }
}

esp_err_t Driver::start() {
    if (passthrough_owner_) return ESP_ERR_INVALID_STATE;
    const auto error = validate_config(config_);
    if (error != ESP_OK) return error;
    if (!impl_ || impl_->link_count == 0 || impl_->join_pending) return ESP_ERR_INVALID_STATE;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (!impl_->links[i]->ready()) return ESP_ERR_INVALID_STATE;
    }
    if (!impl_->worker_exited) {
        impl_->worker_exited = xSemaphoreCreateBinary();
        if (!impl_->worker_exited) return ESP_ERR_NO_MEM;
    }
    const auto timer_error = impl_->scheduler.open(config_.frame_hz);
    if (timer_error != ESP_OK) return timer_error;
    (void)xSemaphoreTake(impl_->worker_exited, 0);
    impl_->stop_requested.store(false);
    impl_->last_error.store(ESP_OK);
    portENTER_CRITICAL(&impl_->statistics_lock);
    impl_->statistics = {};
    impl_->statistics.running = true;
    impl_->statistics.detailed_timing = kDetailedTiming;
    impl_->statistics.timer_period_us = impl_->scheduler.period_us();
    impl_->statistics.link_count = impl_->link_count;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        impl_->statistics.links[i].gpio = impl_->links[i]->pin_;
    }
    impl_->started_at_us = esp_timer_get_time();
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    impl_->last_completed_us.fill(0);
#endif
    portEXIT_CRITICAL(&impl_->statistics_lock);
    impl_->join_pending = true;
    const BaseType_t result = xTaskCreatePinnedToCore(
        worker_entry, "dshot_tx", config_.task_stack_bytes, this, config_.task_priority,
        nullptr, config_.task_core == -1 ? tskNO_AFFINITY : config_.task_core);
    if (result != pdPASS) {
        impl_->scheduler.close();
        impl_->join_pending = false;
        impl_->last_error.store(ESP_ERR_NO_MEM);
        portENTER_CRITICAL(&impl_->statistics_lock);
        impl_->statistics.running = false;
        portEXIT_CRITICAL(&impl_->statistics_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void Driver::stop() {
    if (!impl_) return;
    if (!impl_->join_pending) {
        for (std::size_t i = 0; i < impl_->link_count; ++i) impl_->links[i]->cancel_pending_command();
        return;
    }
    impl_->stop_requested.store(true);
    impl_->scheduler.wake();
    if (xSemaphoreTake(impl_->worker_exited, portMAX_DELAY) != pdTRUE) std::abort();
    impl_->join_pending = false;
}

esp_err_t Driver::last_error() const {
    return impl_ ? impl_->last_error.load() : ESP_OK;
}

DriverStatistics Driver::statistics() const {
    DriverStatistics result;
    result.detailed_timing = kDetailedTiming;
    if (!impl_) return result;
    portENTER_CRITICAL(&impl_->statistics_lock);
    result = impl_->statistics;
    result.detailed_timing = kDetailedTiming;
    if (result.running) result.elapsed_us = esp_timer_get_time() - impl_->started_at_us;
    portEXIT_CRITICAL(&impl_->statistics_lock);
    return result;
}

esp_err_t Driver::send_once(std::uint64_t& wait_us) {
    wait_us = 0;
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (impl_->stop_requested.load()) break;
        std::uint64_t link_wait_us = 0;
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        const auto started = esp_timer_get_time();
#endif
        bool transmitted = false;
        std::uint64_t rx_wait_us = 0;
        const auto error = impl_->links[i]->transmit_pending(&link_wait_us, transmitted, rx_wait_us);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        const auto completed = esp_timer_get_time();
#endif
        wait_us += link_wait_us + rx_wait_us;
        portENTER_CRITICAL(&impl_->statistics_lock);
        auto& link = impl_->statistics.links[i];
        if (transmitted) ++link.completed_frames;
        else if (error != ESP_OK) ++link.tx_errors;
        else ++link.command_pauses;
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        if (transmitted || error != ESP_OK) {
            record(link.outside_tx_wait, completed - started - link_wait_us);
            record(link.tx_wait, link_wait_us);
            if (config_.signaling == Signaling::Bidirectional) record(link.rx_wait, rx_wait_us);
        }
        if (transmitted) {
            if (link.completed_frames > 1) {
                record(link.completion_interval, completed - impl_->last_completed_us[i]);
            }
            impl_->last_completed_us[i] = completed;
        }
#endif
        portEXIT_CRITICAL(&impl_->statistics_lock);
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

void Driver::worker_entry(void* argument) {
    static_cast<Driver*>(argument)->worker_loop();
    vTaskDelete(nullptr); // stop() can return before this task deletes itself.
}

void Driver::worker_loop() {
    if (!impl_->stop_requested.load()) {
        const auto error = impl_->scheduler.start();
        if (error != ESP_OK) {
            impl_->last_error.store(error);
            ESP_LOGE(kTag, "Timer start failed: %s", esp_err_to_name(error));
        }
    }
    while (!impl_->stop_requested.load() && impl_->last_error.load() == ESP_OK) {
        const auto alarms = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (impl_->stop_requested.load()) break;
        if (alarms == 0) continue;
        portENTER_CRITICAL(&impl_->statistics_lock);
        impl_->statistics.missed_slots += alarms - 1;
        portEXIT_CRITICAL(&impl_->statistics_lock);
        std::uint64_t wait_us = 0;
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        const auto started = esp_timer_get_time();
#endif
        const auto error = send_once(wait_us);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        const auto elapsed = esp_timer_get_time() - started;
#endif
        portENTER_CRITICAL(&impl_->statistics_lock);
        ++impl_->statistics.worker_iterations;
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        record(impl_->statistics.worker_pass, elapsed);
        record(impl_->statistics.outside_tx_wait, elapsed - wait_us);
#endif
        portEXIT_CRITICAL(&impl_->statistics_lock);
        if (error != ESP_OK) {
            impl_->last_error.store(error);
            ESP_LOGE(kTag, "TX failed: %s", esp_err_to_name(error));
            break;
        }
        if (impl_->stop_requested.load()) break;
        // Skip missed slots to avoid a burst of catch-up frames.
        const auto skipped = ulTaskNotifyTake(pdTRUE, 0);
        if (impl_->stop_requested.load()) break;
        portENTER_CRITICAL(&impl_->statistics_lock);
        impl_->statistics.missed_slots += skipped;
        portEXIT_CRITICAL(&impl_->statistics_lock);
    }
    impl_->scheduler.close();
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        impl_->links[i]->cancel_pending_command();
    }
    portENTER_CRITICAL(&impl_->statistics_lock);
    impl_->statistics.elapsed_us = esp_timer_get_time() - impl_->started_at_us;
    impl_->statistics.running = false;
    portEXIT_CRITICAL(&impl_->statistics_lock);
    const auto exited = impl_->worker_exited;
    if (xSemaphoreGive(exited) != pdTRUE) std::abort();
}
}
