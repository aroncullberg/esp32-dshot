#include <cstdint>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "esp_err.h"

#include "dshot/config.hpp"
#include "config_validation.hpp"
#include "protocol.hpp"


// using namespace dshot;
namespace dshot {


    #ifdef CONFIG_DSHOT_DEFAULT_BIDIRECTIONAL
    constexpr Signaling kDefaultSignaling = Signaling::Bidirectional;
    #else
    constexpr Signaling kDefaultSignaling = Signaling::Regular;
    #endif

    constexpr unsigned kMinTaskStackBytes = 2048;
    constexpr unsigned kMaxTaskStackBytes = 16384;

    constexpr unsigned kMaxTaskPriority = static_cast<unsigned>(configMAX_PRIORITIES - 1);

    bool is_valid_task_core(int core) {
        return core == -1 || (core >= 0 && core < static_cast<int>(configNUMBER_OF_CORES));
    }

    bool is_valid_framerate(unsigned frame_hz) {
        // GPTimer cadence is independent of the FreeRTOS tick rate.
        return frame_hz != 0 && frame_hz <= 8000;
    }




    Config::Config()
        : protocol(static_cast<Protocol>(CONFIG_DSHOT_DEFAULT_BITRATE)),
            signaling(kDefaultSignaling),
            frame_hz(CONFIG_DSHOT_DEFAULT_FRAME_HZ),
            task_priority(CONFIG_DSHOT_DEFAULT_TASK_PRIORITY),
            task_stack_bytes(CONFIG_DSHOT_DEFAULT_TASK_STACK_BYTES),
            task_core(CONFIG_DSHOT_DEFAULT_TASK_CORE),
            motor_poles(CONFIG_DSHOT_DEFAULT_MOTOR_POLES),
            erpm_max_age_us(CONFIG_DSHOT_DEFAULT_ERPM_MAX_AGE_US),
            edt_max_age_us(CONFIG_DSHOT_DEFAULT_EDT_MAX_AGE_US),
            edt_version(CONFIG_DSHOT_DEFAULT_EDT_VERSION),
            edt_response_timeout_ms(CONFIG_DSHOT_DEFAULT_EDT_RESPONSE_TIMEOUT_MS),
            edt_attempts(CONFIG_DSHOT_DEFAULT_EDT_ATTEMPTS),
            startup_low_ms(CONFIG_DSHOT_DEFAULT_STARTUP_LOW_MS) { }

    esp_err_t validate_config(const Config& config) {
        if (!detail::is_supported_protocol(config.protocol)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (config.frame_hz == 0) {
            return ESP_ERR_INVALID_ARG;
        }
        if (config.task_priority == 0 || config.task_priority > kMaxTaskPriority) {
            return ESP_ERR_INVALID_ARG;
        }
        if (config.task_stack_bytes < kMinTaskStackBytes || config.task_stack_bytes > kMaxTaskStackBytes) {
            return ESP_ERR_INVALID_ARG;
        }
        if (!is_valid_task_core(config.task_core)) {
            return ESP_ERR_INVALID_ARG;
        }
        if (config.signaling != Signaling::Regular && config.signaling != Signaling::Bidirectional)
            return ESP_ERR_INVALID_ARG;
        if (config.startup_low_ms > 1000) return ESP_ERR_INVALID_ARG;
        if (config.motor_poles < 2 || config.motor_poles > 254 || config.motor_poles % 2 != 0)
            return ESP_ERR_INVALID_ARG;
        if (!config.erpm_max_age_us || !config.edt_max_age_us ||
            (config.edt_version != 1 && config.edt_version != 2) ||
            config.edt_response_timeout_ms < 50 || config.edt_response_timeout_ms > 5000 ||
            config.edt_attempts == 0 || config.edt_attempts > 10) return ESP_ERR_INVALID_ARG;
        if (!is_valid_framerate(config.frame_hz)) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        // Reserve 30 us for turnaround, 40 us for RX idle termination, and 20 us margin.
        if (config.signaling == Signaling::Bidirectional) {
            if (config.protocol == Protocol::DShot150) return ESP_ERR_NOT_SUPPORTED;
            const auto bitrate = static_cast<unsigned>(config.protocol);
            const auto wire_us = (32'800'000u + bitrate - 1) / bitrate;
            const auto period_us = (1'000'000u + config.frame_hz / 2) / config.frame_hz;
            if (period_us < wire_us + 90) return ESP_ERR_NOT_SUPPORTED;
        }
        return ESP_OK;
    }

}
