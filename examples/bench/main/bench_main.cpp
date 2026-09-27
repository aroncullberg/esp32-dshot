#include <cerrno>
#include <cstdint>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
enum class Action { Arm, Run, Stop, Stats, EscCommand, CommandStatus };
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

int stats_command(int argc, char**) {
    if (argc != 1) { std::puts("stats"); return 1; }
    return enqueue({Action::Stats});
}

int beep_command(int argc, char** argv) {
    unsigned tone = 1;
    if (argc == 2 && argv[1][0] >= '1' && argv[1][0] <= '5' && argv[1][1] == '\0') {
        tone = argv[1][0] - '0';
    } else if (argc != 1) { std::puts("beep [1..5]"); return 1; }
    return enqueue({Action::EscCommand, static_cast<std::uint16_t>(tone)});
}

int direction_command(int argc, char** argv) {
    if (argc == 2) {
        const char* names[] = {"normal", "reversed", "1", "2"};
        const std::uint16_t ids[] = {20,21,7,8};
        for (unsigned i = 0; i < 4; ++i) {
            if (std::strcmp(argv[1], names[i]) == 0) return enqueue({Action::EscCommand, ids[i]});
        }
    }
    std::puts("direction <normal|reversed|1|2>");
    return 1;
}

int save_command(int argc, char**) {
    if (argc != 1) { std::puts("save"); return 1; }
    return enqueue({Action::EscCommand, static_cast<std::uint16_t>(dshot::Command::SaveSettings)});
}

int esc_command(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && std::strcmp(argv[1], "list") == 0)) {
        for (unsigned id = 1; id <= 47; ++id) {
            if (const auto* info = dshot::command_info(static_cast<dshot::Command>(id))) {
                std::printf("%2u  %s\n", id, info->name);
            }
        }
        return 0;
    }
    if (argc == 2) {
        if (std::strcmp(argv[1], "status") == 0) return enqueue({Action::CommandStatus});
        for (unsigned id = 1; id <= 47; ++id) {
            const auto* info = dshot::command_info(static_cast<dshot::Command>(id));
            if (!info) continue;
            char numeric[3];
            std::snprintf(numeric, sizeof(numeric), "%u", id);
            if (std::strcmp(argv[1], info->name) == 0 || std::strcmp(argv[1], numeric) == 0) {
                return enqueue({Action::EscCommand, static_cast<std::uint16_t>(id)});
            }
        }
    }
    std::puts("command <name|id|list|status>");
    return 1;
}

void print_command_status(const dshot::CommandStatus& status) {
    if (status.id == 0) { std::puts("command idle"); return; }
    const auto* info = dshot::command_info(status.command);
    std::printf("command %" PRIu64 " %s: %s (%u/%u)\n", status.id,
                info ? info->name : "unknown", dshot::command_state_name(status.state),
                status.completed_repetitions, status.total_repetitions);
}

void print_timing(const char* label, const dshot::TimingStatistics& timing) {
    if (timing.count == 0) return;
    std::printf("  %s: min/avg/max %" PRIu64 "/%.2f/%" PRIu64 " us (%" PRIu64 " samples)\n",
                label, timing.min_us, static_cast<double>(timing.total_us) / timing.count,
                timing.max_us, timing.count);
}

void print_statistics(const dshot::DriverStatistics& current,
                      const dshot::DriverStatistics& previous) {
    const auto elapsed = current.elapsed_us - previous.elapsed_us;
    std::printf("stats: %s | %.3f s total | %.3f s interval | missed slots %" PRIu64 " | timer %" PRIu32 " us\n",
                current.running ? "running" : "stopped",
                current.elapsed_us / 1e6, elapsed / 1e6, current.missed_slots,
                current.timer_period_us);
    for (std::size_t i = 0; i < current.link_count; ++i) {
        const auto& link = current.links[i];
        const auto frames = link.completed_frames - previous.links[i].completed_frames;
        std::printf("GPIO %d: frames %" PRIu64 " | TX errors %" PRIu64 " | command pauses %" PRIu64 " | ",
                    link.gpio, link.completed_frames, link.tx_errors, link.command_pauses);
        if (elapsed == 0) std::puts("rate --");
        else std::printf("%.1f frames/s\n", frames * 1e6 / elapsed);
        if (current.detailed_timing) {
            print_timing("outside TX wait", link.outside_tx_wait);
            print_timing("TX wait", link.tx_wait);
            print_timing("completion interval", link.completion_interval);
        }
    }
    if (current.detailed_timing) {
        print_timing("worker pass", current.worker_pass);
        print_timing("worker outside TX waits", current.outside_tx_wait);
    }
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
    dshot::DriverStatistics previous_statistics;

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
        if (error == ESP_ERR_INVALID_STATE) { std::puts("command busy"); return false; }
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
                    previous_statistics = {};
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
            if (arm_until_us != 0) {
                requested_payload = command.payload;
                std::printf("queued %u until armed\n", command.payload);
            } else if (set_payload(command.payload)) {
                requested_payload = command.payload;
                std::printf("run %u\n", command.payload);
            }
            break;
        case Action::Stop:
            requested_payload = 0;
            if (!opened || set_payload(0)) std::puts("stopped");
            break;
        case Action::Stats: {
            const auto snapshot = driver.statistics();
            print_statistics(snapshot, previous_statistics);
            previous_statistics = snapshot;
            break;
        }
        case Action::EscCommand: {
            if (!opened) { std::puts("arm first"); break; }
            if (arm_until_us != 0) { std::puts("wait until armed"); break; }
            if (requested_payload != 0) { std::puts("stop first"); break; }
            const auto id = static_cast<dshot::Command>(command.payload);
            const auto error = link.send_command(id);
            if (error == ESP_OK) std::printf("%s queued\n", dshot::command_info(id)->name);
            else if (error == ESP_ERR_INVALID_STATE) std::puts("command busy");
            else std::printf("command: %s\n", esp_err_to_name(error));
            break;
        }
        case Action::CommandStatus:
            print_command_status(link.command_status());
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
    register_command("stats", "Show frame rate, counters, and enabled timing diagnostics", stats_command);
    register_command("beep", "beep [1..5]", beep_command);
    register_command("direction", "direction <normal|reversed|1|2>", direction_command);
    register_command("save", "Save ESC settings", save_command);
    register_command("command", "command <name|id|list|status>", esc_command);
    std::printf("GPIO %d | DShot%d | %d Hz\n", CONFIG_DSHOT_BENCH_GPIO,
                CONFIG_DSHOT_DEFAULT_BITRATE / 1000, CONFIG_DSHOT_DEFAULT_FRAME_HZ);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
