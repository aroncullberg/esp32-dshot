#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "config_validation.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <new>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

namespace dshot {
namespace {
constexpr const char* kTag = "dshot";
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
    TaskHandle_t worker_task = nullptr;
    SemaphoreHandle_t worker_exited = nullptr;
    bool join_pending = false; // Accessed only by serialized lifecycle calls.
    std::atomic<bool> stop_requested{false};
    std::atomic<esp_err_t> last_error{ESP_OK};
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
    (void)xSemaphoreTake(impl_->worker_exited, 0);
    impl_->stop_requested.store(false);
    impl_->last_error.store(ESP_OK);
    impl_->join_pending = true;
    const BaseType_t result = xTaskCreatePinnedToCore(
        worker_entry, "dshot_tx", config_.task_stack_bytes, this, config_.task_priority,
        &impl_->worker_task, config_.task_core == -1 ? tskNO_AFFINITY : config_.task_core);
    if (result != pdPASS) {
        impl_->worker_task = nullptr;
        impl_->join_pending = false;
        impl_->last_error.store(ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void Driver::stop() {
    if (!impl_ || !impl_->join_pending) return;
    impl_->stop_requested.store(true);
    // The worker finishes/aborts its active TX and acknowledges its last owner access.
    if (xSemaphoreTake(impl_->worker_exited, portMAX_DELAY) != pdTRUE) std::abort();
    impl_->join_pending = false;
    impl_->worker_task = nullptr;
}

esp_err_t Driver::last_error() const {
    return impl_ ? impl_->last_error.load() : ESP_OK;
}

esp_err_t Driver::send_once() {
    for (std::size_t i = 0; i < impl_->link_count; ++i) {
        if (impl_->stop_requested.load()) break;
        const auto error = impl_->links[i]->transmit_pending();
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

void Driver::worker_entry(void* argument) {
    static_cast<Driver*>(argument)->worker_loop();
    vTaskDelete(nullptr); // No access to the Driver after the exit acknowledgement.
}

void Driver::worker_loop() {
    const TickType_t period_ticks = configTICK_RATE_HZ / config_.frame_hz;
    TickType_t last_wake_tick = xTaskGetTickCount();
    while (!impl_->stop_requested.load()) {
        const auto error = send_once();
        if (error != ESP_OK) {
            impl_->last_error.store(error);
            ESP_LOGE(kTag, "TX failed: %s", esp_err_to_name(error));
            break;
        }
        if (impl_->stop_requested.load()) break;
        if (xTaskDelayUntil(&last_wake_tick, period_ticks) == pdFALSE) {
            // Skip missed slots instead of sending a burst of catch-up frames.
            last_wake_tick = xTaskGetTickCount();
            (void)xTaskDelayUntil(&last_wake_tick, period_ticks);
        }
    }
    const auto exited = impl_->worker_exited;
    if (xSemaphoreGive(exited) != pdTRUE) std::abort();
}
}
