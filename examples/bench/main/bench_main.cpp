#include <cerrno>
#include <array>
#include <cstdint>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>

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
#if DSHOT_BENCH_FOUR_MOTORS
#include "bench_profile.hpp"
#endif

namespace {
#if DSHOT_BENCH_FOUR_MOTORS
constexpr std::array<gpio_num_t, 4> motor_pins{
    static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO),
    static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO_2),
    static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO_3),
    static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO_4)};
#else
constexpr std::array<gpio_num_t, 1> motor_pins{static_cast<gpio_num_t>(CONFIG_DSHOT_BENCH_GPIO)};
#endif
constexpr unsigned all_motors = (1u << motor_pins.size()) - 1;
enum class Action { Arm, Run, Stop, Stats, EscCommand, CommandStatus, Telemetry, TelemetryRaw, Edt, Motor, Profile };
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

#if DSHOT_BENCH_FOUR_MOTORS
int profile_command(int argc, char** argv) {
    const bool basic = argc >= 2 && std::strcmp(argv[argc - 1], "basic") == 0;
    argc -= basic;
    const std::uint16_t flags = basic ? 0x8000 : 0;
    if (argc == 1) return enqueue({Action::Profile, static_cast<std::uint16_t>(3 | flags)});
    if (argc == 2 && argv[1][0] >= '0' && argv[1][0] <= '9') {
        errno = 0;
        char* end = nullptr;
        const auto seconds = std::strtoul(argv[1], &end, 10);
        if (!errno && *end == '\0' && seconds >= 1 && seconds <= 30)
            return enqueue({Action::Profile, static_cast<std::uint16_t>(seconds | flags)});
    }
    std::puts("profile [seconds: 1..30; default 3] [basic]");
    return 1;
}

int cpu_command(int argc, char**) {
    if (argc != 1) { std::puts("cpu"); return 1; }
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    using Counter = configRUN_TIME_COUNTER_TYPE;
    std::array<Counter, configNUMBER_OF_CORES> idle_before{}, idle_after{};
    std::puts("Sampling CPU usage for 1 second...");
    const auto start = portGET_RUN_TIME_COUNTER_VALUE();
    for (BaseType_t core = 0; core < configNUMBER_OF_CORES; ++core)
        idle_before[core] = ulTaskGetIdleRunTimeCounterForCore(core);

    // Sample in the console task so the bench worker keeps processing commands and arming.
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (BaseType_t core = 0; core < configNUMBER_OF_CORES; ++core)
        idle_after[core] = ulTaskGetIdleRunTimeCounterForCore(core);
    const Counter elapsed = portGET_RUN_TIME_COUNTER_VALUE() - start;
    if (elapsed == 0) { std::puts("cpu: no elapsed run time"); return 1; }
    double total = 0;
    for (BaseType_t core = 0; core < configNUMBER_OF_CORES; ++core) {
        const Counter idle = idle_after[core] - idle_before[core];
        // Idle accounting is updated on context switches; clamp sampling skew.
        const double usage = idle >= elapsed ? 0.0 : 100.0 * (1.0 - static_cast<double>(idle) / elapsed);
        std::printf("CPU%d: %.1f%% busy\n", static_cast<int>(core), usage);
        total += usage;
    }
    std::printf("CPU average: %.1f%% busy\n", total / configNUMBER_OF_CORES);
    return 0;
#else
    std::puts("cpu: enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS and rebuild");
    return 1;
#endif
}

int motor_command(int argc, char** argv) {
    if (argc == 2) {
        if (std::strcmp(argv[1], "all") == 0) return enqueue({Action::Motor, all_motors});
        if (argv[1][0] >= '1' && argv[1][0] <= '4' && argv[1][1] == '\0')
            return enqueue({Action::Motor, static_cast<std::uint16_t>(1u << (argv[1][0] - '1'))});
    }
    std::puts("motor <1..4|all>");
    return 1;
}
#endif

int telemetry_command(int argc, char** argv) {
    if (argc == 1) return enqueue({Action::Telemetry});
    if (argc == 2 && std::strcmp(argv[1],"raw") == 0) return enqueue({Action::TelemetryRaw});
    std::puts("telemetry [raw]");
    return 1;
}
int edt_command(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && std::strcmp(argv[1],"status") == 0))
        return enqueue({Action::Telemetry});
    if (argc == 2 && std::strcmp(argv[1],"on") == 0) return enqueue({Action::Edt,1});
    if (argc == 2 && std::strcmp(argv[1],"off") == 0) return enqueue({Action::Edt,0});
    std::puts("edt <on|off|status>");
    return 1;
}

void print_receive_stats(const dshot::Telemetry& data) {
    if (!data.available) return;
    const auto& c = data.counters;
    std::printf("RX: replies %" PRIu64 " | no response %" PRIu64 " | malformed %" PRIu64
                " | GCR %" PRIu64 " | CRC %" PRIu64 " | overflow %" PRIu64 " | errors %" PRIu64 "\n",
                c.replies,c.no_response,c.malformed,c.gcr_errors,c.checksum_errors,c.overflows,c.receive_errors);
    std::printf("  no response breakdown: timeouts %" PRIu64 " | empty/idle captures %" PRIu64 "\n",
                c.rx_timeouts, c.empty_captures);
}
void print_capture(const dshot::ReceiveCapture& capture) {
    if (!capture.pulse_count) { std::puts("RX capture: no data"); return; }
    const char* result = "unknown";
    switch (capture.result) {
    case dshot::ReceiveResult::Valid: result = "valid"; break;
    case dshot::ReceiveResult::NoResponse: result = "no response"; break;
    case dshot::ReceiveResult::Malformed: result = "malformed"; break;
    case dshot::ReceiveResult::GcrError: result = "GCR error"; break;
    case dshot::ReceiveResult::ChecksumError: result = "CRC error"; break;
    case dshot::ReceiveResult::Overflow: result = "overflow"; break;
    case dshot::ReceiveResult::ReceiveError: result = "receive error"; break;
    }
    std::printf("RX capture: %s | age %.1f ms | clock %" PRIu32 " Hz | D%" PRIu32 "\n",
        result, (esp_timer_get_time()-capture.captured_at_us)/1000.0,
        capture.resolution_hz, capture.bitrate/1000);
    std::puts("  level:duration_ticks (0 duration = end marker)");
    for (unsigned i = 0; i < capture.pulse_count; ++i) {
        const auto& pulse = capture.pulses[i];
        std::printf(" %u:%u", pulse.high ? 1u : 0u, static_cast<unsigned>(pulse.ticks));
        if (i%8 == 7 || i+1 == capture.pulse_count) std::putchar('\n');
    }
}

void print_telemetry(const dshot::Telemetry& data) {
    if (!data.available) { std::puts("telemetry: select bidirectional signaling and arm"); return; }
    const auto now = esp_timer_get_time();
    const auto& speed = data.speed;
    std::printf("eRPM: %s", dshot::erpm_state_name(speed.state));
    if (speed.state != dshot::ErpmState::NoData)
        std::printf(" | %" PRIu32 " electrical RPM | %" PRIu32 " motor RPM (%u poles) | period %" PRIu32 " us | age %.1f ms",
                    speed.erpm,speed.rpm,speed.motor_poles,speed.period_us,(now-speed.updated_at_us)/1000.0);
    std::printf("\nEDT: %s | layout v%u | attempts %u\n", dshot::edt_state_name(data.edt.state),
                data.edt.interpretation_version,data.edt.attempts);
    auto field = [&](const char* name, const dshot::TelemetryField& value, const char* unit) {
        if (!value.valid) return;
        std::printf("  %s: %" PRIu32 " %s | age %.1f ms%s\n",name,value.value,unit,
                    (now-value.updated_at_us)/1000.0,value.fresh ? "" : " | stale");
    };
    field("acknowledgement",data.edt.acknowledgement,"raw");
    field("temperature",data.temperature_c,"C");
    field("voltage",data.voltage_mv,"mV");
    field("current",data.current_ma,"mA");
    field("debug1",data.debug1,"raw"); field("debug2",data.debug2,"raw");
    field("debug3",data.debug3,"raw"); field("stress",data.stress,"/255");
    field("status",data.status,"raw"); field("unknown",data.unknown,"raw payload");
    field("last payload",data.last_payload,"raw");
    if (data.status.valid && data.edt.interpretation_version == 2 && data.status.value != 255)
        std::printf("  status: alert %" PRIu32 " | warning %" PRIu32 " | error %" PRIu32 " | max stress %" PRIu32 "/15\n",
                    (data.status.value >> 7)&1,(data.status.value >> 6)&1,
                    (data.status.value >> 5)&1,data.status.value&15);
    print_receive_stats(data);
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
            print_timing("RX wait/decode", link.rx_wait);
            print_timing("completion interval", link.completion_interval);
        }
    }
    if (current.detailed_timing) {
        print_timing("worker pass", current.worker_pass);
        print_timing("worker outside TX/RX waits", current.outside_tx_wait);
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

// Keep persistent state and statistics buffers on the heap. At -O2, inlining
// otherwise combines these with reporting temporaries in the worker stack frame.
struct BenchState {
    dshot::Config config;
    dshot::Driver driver{config};
#if DSHOT_BENCH_FOUR_MOTORS
    std::array<dshot::Link, 4> links{{{driver, motor_pins[0]}, {driver, motor_pins[1]},
                                   {driver, motor_pins[2]}, {driver, motor_pins[3]}}};
#else
    std::array<dshot::Link, 1> links{{{driver, motor_pins[0]}}};
#endif
    std::array<std::uint16_t, motor_pins.size()> requested_payloads{};
    dshot::DriverStatistics previous_statistics;
    dshot::DriverStatistics statistics_snapshot;
};

void bench_worker(void*) {
    std::unique_ptr<BenchState> state(new (std::nothrow) BenchState);
    if (!state) {
        std::puts("bench state allocation failed");
        vTaskDelete(nullptr);
        return;
    }
    auto& driver = state->driver;
    auto& links = state->links;
    auto& requested_payloads = state->requested_payloads;
    auto& previous_statistics = state->previous_statistics;
    bool opened = false;
    unsigned opened_motors = 0;
    unsigned selected_motors = all_motors;
    int64_t arm_until_us = 0;
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
    std::unique_ptr<bench::RuntimeProfile> profile;
#endif
    auto cancel_profile = [&]() {
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
        if (profile) {
            (void)driver.set_stage_profiling(false);
            profile.reset();
            std::puts("profile cancelled: workload changed");
        }
#endif
    };

    auto selected = [&](std::size_t i) { return (selected_motors & (1u << i)) != 0; };
    auto label = [&](std::size_t i) {
        if (motor_pins.size() > 1) std::printf("Motor %u (GPIO %d):\n", static_cast<unsigned>(i + 1), motor_pins[i]);
    };
    auto close_on_error = [&]() {
        cancel_profile();
        driver.stop();
        for (auto& link : links) link.end();
        opened = false;
        opened_motors = 0;
        arm_until_us = 0;
        requested_payloads.fill(0);
    };
    auto set_payload = [&](std::uint16_t payload, unsigned motors) {
        // Check every target before changing any motor's throttle.
        if (payload != 0) {
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!(motors & (1u << i))) continue;
                const auto state = links[i].command_status().state;
                if (state == dshot::CommandState::Pending || state == dshot::CommandState::Preparing ||
                    state == dshot::CommandState::Transmitting || state == dshot::CommandState::Cooldown) {
                    std::puts("command busy");
                    return false;
                }
            }
        }
        for (std::size_t i = 0; i < links.size(); ++i) {
            if (!(motors & (1u << i))) continue;
            const auto error = links[i].set(payload);
            if (error == ESP_OK) continue;
            std::printf("set motor %u: %s\n", static_cast<unsigned>(i + 1), esp_err_to_name(error));
            // Cancel any partial group update before returning an error.
            for (std::size_t j = 0; j < links.size(); ++j)
                if (opened_motors & (1u << j)) (void)links[j].set(0);
            close_on_error();
            return false;
        }
        return true;
    };
    auto targets_open = [&]() {
        if (!opened || (selected_motors & opened_motors) != selected_motors) {
            std::puts("arm first");
            return false;
        }
        return true;
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
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!(opened_motors & (1u << i))) continue;
                if (!set_payload(requested_payloads[i], 1u << i)) break;
                if (requested_payloads[i] != 0) {
                    label(i);
                    std::printf("run %u\n", requested_payloads[i]);
                }
            }
            if (opened) std::puts("armed");
        }
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
        if (profile && esp_timer_get_time() >= profile->deadline_us) {
            profile->finish(state->config, driver, links, opened_motors, requested_payloads);
            profile.reset();
        }
#endif
        if (!received) continue;

        if (command.action == Action::Arm || command.action == Action::Run || command.action == Action::Stop ||
            command.action == Action::EscCommand || command.action == Action::Edt) cancel_profile();

        switch (command.action) {
        case Action::Profile:
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
            if (profile) { std::puts("profile: sample already running"); break; }
            if (arm_until_us) { std::puts("profile: wait until armed"); break; }
            profile.reset(new (std::nothrow) bench::RuntimeProfile);
            if (!profile) { std::puts("profile: allocation failed"); break; }
            std::printf("Sampling profile for %u seconds; keep the workload unchanged...\n", static_cast<unsigned>(command.payload & 0x7fff));
            if (!profile->start(command.payload & 0x7fff, driver, links, opened_motors,
                                !(command.payload & 0x8000))) profile.reset();
#else
            std::puts("profile: enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS and CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER, then rebuild");
#endif
            break;
        case Action::Arm: {
            if (opened && opened_motors != selected_motors) close_on_error();
            if (!opened) {
                esp_err_t error = ESP_OK;
                for (std::size_t i = 0; i < links.size(); ++i) {
                    if (!selected(i)) continue;
                    error = links[i].begin();
                    if (error != ESP_OK) break;
                }
                if (error == ESP_OK) {
                    opened = true;
                    opened_motors = selected_motors;
                    previous_statistics = {};
                    error = driver.start();
                }
                if (error != ESP_OK) {
                    std::printf("arm: %s\n", esp_err_to_name(error));
                    close_on_error();
                    break;
                }
            }
            requested_payloads.fill(0);
            if (set_payload(0, opened_motors)) {
                arm_until_us = esp_timer_get_time() +
                    static_cast<int64_t>(CONFIG_DSHOT_BENCH_ARM_HOLD_MS) * 1000;
                std::puts("arming");
            }
            break;
        }
        case Action::Run:
            if (!targets_open()) break;
            if (arm_until_us != 0) {
                for (std::size_t i = 0; i < links.size(); ++i)
                    if (selected(i)) requested_payloads[i] = command.payload;
                std::printf("queued %u until armed\n", command.payload);
            } else if (set_payload(command.payload, selected_motors)) {
                for (std::size_t i = 0; i < links.size(); ++i)
                    if (selected(i)) requested_payloads[i] = command.payload;
                std::printf("run %u\n", command.payload);
            }
            break;
        case Action::Stop:
            requested_payloads.fill(0);
            if (!opened || set_payload(0, opened_motors)) std::puts("stopped");
            break;
        case Action::Stats: {
            auto& snapshot = state->statistics_snapshot;
            driver.statistics(snapshot);
            print_statistics(snapshot, previous_statistics);
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!(opened_motors & (1u << i))) continue;
                label(i);
                print_receive_stats(links[i].telemetry());
            }
            previous_statistics = snapshot;
            break;
        }
        case Action::EscCommand: {
            if (!targets_open()) break;
            if (arm_until_us != 0) { std::puts("wait until armed"); break; }
            bool running = false;
            for (std::size_t i = 0; i < links.size(); ++i)
                if (selected(i) && requested_payloads[i] != 0) running = true;
            if (running) { std::puts("stop first"); break; }
            const auto id = static_cast<dshot::Command>(command.payload);
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!selected(i)) continue;
                label(i);
                const auto error = links[i].send_command(id);
                if (error == ESP_OK) std::printf("%s queued\n", dshot::command_info(id)->name);
                else if (error == ESP_ERR_INVALID_STATE) std::puts("command busy");
                else std::printf("command: %s\n", esp_err_to_name(error));
            }
            break;
        }
        case Action::CommandStatus:
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!selected(i)) continue;
                label(i);
                print_command_status(links[i].command_status());
            }
            break;
        case Action::Telemetry:
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!selected(i)) continue;
                label(i);
                print_telemetry(links[i].telemetry());
            }
            break;
        case Action::TelemetryRaw:
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!selected(i)) continue;
                label(i);
                print_capture(links[i].receive_capture());
            }
            break;
        case Action::Edt: {
            if (!targets_open()) break;
            if (arm_until_us) { std::puts("wait until armed"); break; }
            bool running = false;
            for (std::size_t i = 0; i < links.size(); ++i)
                if (selected(i) && requested_payloads[i] != 0) running = true;
            if (running) { std::puts("stop first"); break; }
            for (std::size_t i = 0; i < links.size(); ++i) {
                if (!selected(i)) continue;
                label(i);
                const auto error = links[i].request_edt(command.payload != 0);
                if (error == ESP_OK) std::puts(command.payload ? "EDT enable queued" : "EDT disable queued");
                else std::printf("EDT: %s\n",esp_err_to_name(error));
            }
            break;
        }
        case Action::Motor:
            selected_motors = command.payload;
            std::printf("selected motors:");
            for (std::size_t i = 0; i < links.size(); ++i)
                if (selected(i)) std::printf(" %u", static_cast<unsigned>(i + 1));
            std::putchar('\n');
            break;
        }
    }
}
}

extern "C" void app_main() {
    commands = xQueueCreate(8, sizeof(Command));
    if (!commands) { std::puts("command queue allocation failed"); return; }
    if (xTaskCreate(bench_worker, "dshot_bench", 8192, nullptr, 5, nullptr) != pdPASS) {
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
    register_command("telemetry", "Show eRPM/EDT and counters; telemetry raw shows captured pulses", telemetry_command);
    register_command("edt", "edt <on|off|status>", edt_command);
#if DSHOT_BENCH_FOUR_MOTORS
    register_command("profile", "profile [1..30 seconds] [basic]: task/runtime and sampled stages (default 3 seconds)", profile_command);
    register_command("cpu", "Show per-core CPU usage over a 1-second sample", cpu_command);
    register_command("motor", "motor <1..4|all>: select targets; arm opens only the selection; stop always stops all", motor_command);
#endif
    for (std::size_t i = 0; i < motor_pins.size(); ++i)
        std::printf("Motor %u: GPIO %d\n", static_cast<unsigned>(i + 1), motor_pins[i]);
    std::printf("DShot%d | target %d Hz per motor | FreeRTOS %d Hz | RX timeout %u ticks\n",
                CONFIG_DSHOT_DEFAULT_BITRATE / 1000, CONFIG_DSHOT_DEFAULT_FRAME_HZ,
                CONFIG_FREERTOS_HZ, static_cast<unsigned>(pdMS_TO_TICKS(2) + 1));
    std::printf("Startup low hold: %u ms per output\n", dshot::Config{}.startup_low_ms);
    if (motor_pins.size() > 1) std::puts("All motors selected. motor 1..4 selects one; motor all selects all.");
    std::puts(dshot::Config{}.signaling == dshot::Signaling::Bidirectional ?
              "Bidirectional (inverted)" : "Regular (uninverted)");
    if (dshot::Config{}.signaling == dshot::Signaling::Bidirectional)
        std::puts("RX starts before TX; outgoing echo removed");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
