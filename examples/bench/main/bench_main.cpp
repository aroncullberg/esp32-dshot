#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "sdkconfig.h"
#include "dshot/config.hpp"
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {
enum class Action { Arm, Run, Stop };
struct Command {
    Action action;
    std::uint16_t payload = 0;
};
QueueHandle_t commands;

int enqueue(Command command) {
    if (xQueueSend(commands, &command, 0) == pdTRUE) return 0;
    std::puts("command queue full");
    return 1;
}

int arm_command(int argc, char**) {
    if (argc != 1) { std::puts("arm"); return 1; }
    return enqueue({Action::Arm});
}

int stop_command(int argc, char**) {
    if (argc != 1) { std::puts("stop"); return 1; }
    return enqueue({Action::Stop});
}

int run_command(int argc, char** argv) {
    if (argc != 2) { std::puts("run <0..2047>"); return 1; }
    const char* text = argv[1];
    if (!text || *text < '0' || *text > '9') {
        std::puts("run <0..2047>");
        return 1;
    }
    errno = 0;
    char* end = nullptr;
    const auto value = std::strtoul(text, &end, 10);
    if (errno != 0 || *end != '\0' || value > 2047) {
        std::puts("run <0..2047>");
        return 1;
    }
    return enqueue({Action::Run, static_cast<std::uint16_t>(value)});
}

void register_command(const char* name, const char* help, esp_console_cmd_func_t callback) {
    esp_console_cmd_t command{};
    command.command = name;
    command.help = help;
    command.func = callback;
    ESP_ERROR_CHECK(esp_console_cmd_register(&command));
}

void bench_worker(void*) {
    dshot::Config config;
    dshot::Driver driver(config);
    dshot::Link link(driver, static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO));
    bool opened = false;
    int64_t arm_until_us = 0;
    std::uint16_t requested_payload = 0;

    auto close_on_error = [&]() {
        driver.stop();
        link.end();
        opened = false;
        arm_until_us = 0;
        requested_payload = 0;
    };
    auto set_payload = [&](std::uint16_t payload) {
        const auto error = link.set(payload);
        if (error == ESP_OK) return true;
        std::printf("set: %s\n", esp_err_to_name(error));
        close_on_error();
        return false;
    };

    for (;;) {
        Command command{};
        const bool received = xQueueReceive(commands, &command, pdMS_TO_TICKS(10)) == pdTRUE;
        if (opened && driver.last_error() != ESP_OK) {
            std::printf("TX: %s\n", esp_err_to_name(driver.last_error()));
            close_on_error();
        }
        if (opened && arm_until_us != 0 && esp_timer_get_time() >= arm_until_us) {
            arm_until_us = 0;
            if (set_payload(requested_payload)) {
                std::puts("armed");
                if (requested_payload != 0) std::printf("run %u\n", requested_payload);
            }
        }
        if (!received) continue;

        switch (command.action) {
        case Action::Arm: {
            if (!opened) {
                auto error = link.begin();
                if (error == ESP_OK) {
                    opened = true;
                    error = driver.start(); // A freshly opened link starts with zero pending.
                }
                if (error != ESP_OK) {
                    std::printf("arm: %s\n", esp_err_to_name(error));
                    close_on_error();
                    break;
                }
            }
            requested_payload = 0;
            if (set_payload(0)) {
                arm_until_us = esp_timer_get_time() +
                    static_cast<int64_t>(CONFIG_DSHOT_BENCH_ARM_HOLD_MS) * 1000;
                std::puts("arming");
            }
            break;
        }
        case Action::Run:
            if (!opened) {
                std::puts("arm first");
                break;
            }
            requested_payload = command.payload;
            if (arm_until_us != 0) {
                std::printf("queued %u until armed\n", command.payload);
            } else if (set_payload(command.payload)) {
                std::printf("run %u\n", command.payload);
            }
            break;
        case Action::Stop:
            requested_payload = 0;
            if (!opened || set_payload(0)) std::puts("stopped");
            break;
        }
    }
}
}

extern "C" void app_main() {
    commands = xQueueCreate(8, sizeof(Command));
    if (!commands) { std::puts("command queue allocation failed"); return; }
    if (xTaskCreate(bench_worker, "dshot_bench", 4096, nullptr, 5, nullptr) != pdPASS) {
        vQueueDelete(commands);
        std::puts("bench task creation failed");
        return;
    }
    esp_console_repl_config_t settings = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    settings.prompt = "dshot>";
    settings.max_cmdline_length = 64;
    esp_console_repl_t* repl = nullptr;
    ESP_ERROR_CHECK(esp_console_new_repl_stdio(&settings, &repl));
    register_command("arm", "Start continuous zero frames", arm_command);
    register_command("run", "run <0..2047>: hold this raw payload", run_command);
    register_command("stop", "Return to continuous zero frames", stop_command);
    std::printf("GPIO %d | DShot%d | %d Hz\n", CONFIG_DSHOT_BENCH_GPIO,
                CONFIG_DSHOT_DEFAULT_BITRATE / 1000, CONFIG_DSHOT_DEFAULT_FRAME_HZ);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
