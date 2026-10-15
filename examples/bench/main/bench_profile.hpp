#pragma once

#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
#define DSHOT_BENCH_RUNTIME_PROFILE_ENABLED 1

namespace bench {
// Allocated only while profiling; keep the task snapshots off the bench task's stack.
class RuntimeProfile {
    using Counter = configRUN_TIME_COUNTER_TYPE;
    static constexpr unsigned max_tasks = 32;
    struct Task {
        TaskHandle_t handle = nullptr;
        UBaseType_t number = 0;
        Counter runtime = 0;
        UBaseType_t stack_free_bytes = 0;
        char name[configMAX_TASK_NAME_LEN]{};
    };
    struct Snapshot {
        Counter time = 0;
        unsigned count = 0;
        std::array<Task, max_tasks> tasks{};
        dshot::DriverStatistics driver;
        std::array<dshot::TelemetryCounters, 4> rx{};
        std::int64_t capture_us = 0;
    };
    Snapshot before_, after_;
    std::array<TaskStatus_t, max_tasks> scratch_{};
    bool stages_enabled_ = false;

    void print_stages(Counter elapsed) const {
        if (!stages_enabled_) {
            std::puts("Stage profile: off (basic sample, not compiled, or worker not pinned)");
            return;
        }
        const auto& current = after_.driver.stage_profile;
        const auto& previous = before_.driver.stage_profile;
        const auto samples = current.sampled_passes - previous.sampled_passes;
        const auto passes = after_.driver.worker_iterations - before_.driver.worker_iterations;
        std::printf("Stage profile: %" PRIu64 " sampled updates / %" PRIu64 " | interval 1/%u | idle clamps %" PRIu64 "\n",
                    samples, passes, current.sample_interval, current.idle_clamps - previous.idle_clamps);
        if (!samples) { std::puts("  No stage samples (worker stopped or sample interval not reached)."); return; }
        constexpr const char* names[] = {"empty probe", "frame preparation", "symbol encoding", "RX setup",
            "TX submission", "TX wait", "RX wait", "RX finish", "echo removal", "reply decoding",
            "telemetry update", "raw capture copy", "link finish", "bookkeeping"};
        static_assert(sizeof(names) / sizeof(*names) == static_cast<std::size_t>(dshot::ProfileStage::Count));
        std::printf("  %-19s %8s %13s %13s %11s\n", "stage", "visits", "elapsed us/v", "busy us/v", "est core %");
        for (std::size_t i = 0; i < current.stages.size(); ++i) {
            const auto count = current.stages[i].count - previous.stages[i].count;
            if (!count) continue;
            const auto wall = current.stages[i].elapsed_us - previous.stages[i].elapsed_us;
            const auto busy = current.stages[i].busy_us - previous.stages[i].busy_us;
            std::printf("  %-19s %8" PRIu64 " %13.2f %13.2f %10.2f%%\n", names[i], count,
                        static_cast<double>(wall) / count, static_cast<double>(busy) / count,
                        100.0 * busy * passes / samples / elapsed);
        }
        std::puts("  Elapsed includes blocking; busy subtracts core idle and can include other tasks.");
        std::puts("  Core % extrapolates sampled updates; probe/measurement overhead and sampling bias apply.");
        std::puts("  Scheduler work outside the sampled pass, aggregate merging, and separate ISR costs are not covered.");
    }

    static const Task* find(const Snapshot& snapshot, TaskHandle_t handle, UBaseType_t number) {
        for (unsigned i = 0; i < snapshot.count; ++i)
            if (snapshot.tasks[i].handle == handle && snapshot.tasks[i].number == number)
                return &snapshot.tasks[i];
        return nullptr;
    }

    bool capture(Snapshot& snapshot, const dshot::Driver& driver,
                 const std::array<dshot::Link, 4>& links, unsigned opened_motors) {
        const auto start = esp_timer_get_time();
        driver.statistics(snapshot.driver);
        for (unsigned i = 0; i < links.size(); ++i)
            if (opened_motors & (1u << i)) snapshot.rx[i] = links[i].telemetry_counters();
        snapshot.count = uxTaskGetSystemState(scratch_.data(), scratch_.size(), &snapshot.time);
        if (!snapshot.count) {
            std::puts("profile: task snapshot failed (maximum 32 tasks); no report collected");
            return false;
        }
        for (unsigned i = 0; i < snapshot.count; ++i) {
            const auto& source = scratch_[i];
            auto& task = snapshot.tasks[i];
            task.handle = source.xHandle;
            task.number = source.xTaskNumber;
            task.runtime = source.ulRunTimeCounter;
            task.stack_free_bytes = source.usStackHighWaterMark;
            // Copy names at the boundary; TaskStatus_t names point into live task storage.
            std::snprintf(task.name, sizeof(task.name), "%s", source.pcTaskName);
        }
        snapshot.capture_us = esp_timer_get_time() - start;
        return true;
    }

public:
    std::int64_t deadline_us = 0;

    bool start(unsigned seconds, dshot::Driver& driver,
               const std::array<dshot::Link, 4>& links, unsigned opened_motors, bool stages = true) {
        const auto error = driver.set_stage_profiling(stages);
        stages_enabled_ = stages && error == ESP_OK;
        if (error != ESP_OK && error != ESP_ERR_NOT_SUPPORTED) {
            std::printf("profile: stage setup failed: %s\n", esp_err_to_name(error));
            return false;
        }
        if (!capture(before_, driver, links, opened_motors)) {
            (void)driver.set_stage_profiling(false);
            return false;
        }
        deadline_us = esp_timer_get_time() + static_cast<std::int64_t>(seconds) * 1000000;
        return true;
    }

    void finish(const dshot::Config& config, dshot::Driver& driver,
                const std::array<dshot::Link, 4>& links, unsigned opened_motors,
                const std::array<std::uint16_t, 4>& payloads) {
        const bool captured = capture(after_, driver, links, opened_motors);
        (void)driver.set_stage_profiling(false);
        if (!captured) return;
        const Counter elapsed = after_.time - before_.time;
        if (!elapsed) { std::puts("profile: no elapsed run time"); return; }
        if (before_.driver.running != after_.driver.running ||
            before_.driver.link_count != after_.driver.link_count ||
            after_.driver.worker_iterations < before_.driver.worker_iterations) {
            std::puts("profile cancelled: driver state changed");
            return;
        }
        std::printf("profile: %.3f s | D%u %s | target %u Hz | worker core %d\n",
                    elapsed / 1e6, static_cast<unsigned>(config.protocol) / 1000,
                    config.signaling == dshot::Signaling::Bidirectional ? "bidirectional" : "regular",
                    config.frame_hz, config.task_core);
        std::printf("  stream %s | detailed timing %s | snapshot overhead %" PRId64 "/%" PRId64 " us\n",
                    after_.driver.running ? "running" : "stopped",
                    after_.driver.detailed_timing ? "on" : "off",
                    before_.capture_us, after_.capture_us);
        std::printf("  active outputs:");
        for (unsigned i = 0; i < links.size(); ++i)
            if (opened_motors & (1u << i)) std::printf(" %u(raw %u)", i + 1, payloads[i]);
        if (!opened_motors) std::printf(" none");
        std::putchar('\n');

        double total_busy = 0;
        bool all_idle_found = true;
        for (BaseType_t core = 0; core < configNUMBER_OF_CORES; ++core) {
            const auto handle = xTaskGetIdleTaskHandleForCore(core);
            const Task* idle = nullptr;
            for (unsigned i = 0; i < after_.count; ++i)
                if (after_.tasks[i].handle == handle) idle = &after_.tasks[i];
            const auto* previous = idle ? find(before_, handle, idle->number) : nullptr;
            if (!previous) {
                std::printf("CPU%d: unavailable (idle task missing)\n", static_cast<int>(core));
                all_idle_found = false;
                continue;
            }
            const Counter delta = idle->runtime - previous->runtime;
            const double busy = delta >= elapsed ? 0.0 : 100.0 * (1.0 - static_cast<double>(delta) / elapsed);
            std::printf("CPU%d: %.1f%% busy\n", static_cast<int>(core), busy);
            total_busy += busy;
        }
        if (all_idle_found) std::printf("CPU average: %.1f%% busy\n", total_busy / configNUMBER_OF_CORES);
        std::puts("Task runtime: 100% = one core; idle tasks included");
        std::printf("  %-*s %12s %9s %12s\n", configMAX_TASK_NAME_LEN, "task", "runtime us", "CPU %", "stack free B");
        bool changed = false;
        for (unsigned i = 0; i < after_.count; ++i) {
            const auto& task = after_.tasks[i];
            const auto* previous = find(before_, task.handle, task.number);
            const Counter delta = previous ? static_cast<Counter>(task.runtime - previous->runtime) : task.runtime;
            std::printf("  %-*s %12" PRIu64 " %8.2f%% %12u%s\n", configMAX_TASK_NAME_LEN, task.name,
                        static_cast<std::uint64_t>(delta), 100.0 * delta / elapsed,
                        static_cast<unsigned>(task.stack_free_bytes),
                        previous ? "" : " (new)");
            if (!previous) changed = true;
        }
        for (unsigned i = 0; i < before_.count; ++i) {
            const auto& task = before_.tasks[i];
            if (find(after_, task.handle, task.number)) continue;
            std::printf("  %-*s exited; final runtime unavailable\n", configMAX_TASK_NAME_LEN, task.name);
            changed = true;
        }
        if (changed) std::puts("  Task set changed; tasks that exited cannot be fully accounted for.");
        std::puts("  RTOS task accounting includes interrupt time in the interrupted task; ISR cost is not separated.");
        const auto iterations = after_.driver.worker_iterations - before_.driver.worker_iterations;
        std::printf("DShot window: passes %" PRIu64 " (%.1f/s) | missed slots %" PRIu64 "\n",
                    iterations, iterations * 1e6 / elapsed,
                    after_.driver.missed_slots - before_.driver.missed_slots);
        for (std::size_t i = 0; i < after_.driver.link_count; ++i) {
            const auto& current = after_.driver.links[i];
            const auto& previous = before_.driver.links[i];
            const auto frames = current.completed_frames - previous.completed_frames;
            std::printf("  GPIO %d: frames %" PRIu64 " (%.1f/s) | TX errors %" PRIu64 " | command pauses %" PRIu64 "\n",
                        current.gpio, frames, frames * 1e6 / elapsed,
                        current.tx_errors - previous.tx_errors, current.command_pauses - previous.command_pauses);
        }
        print_stages(elapsed);
        std::printf("Bench stack minimum free after reporting: %u bytes\n",
                    static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
        if (config.signaling != dshot::Signaling::Bidirectional) return;
        for (unsigned i = 0; i < links.size(); ++i) {
            if (!(opened_motors & (1u << i))) continue;
            const auto& c = after_.rx[i];
            const auto& p = before_.rx[i];
            std::printf("  Motor %u RX: replies %" PRIu64 " | no response %" PRIu64
                        " (timeouts %" PRIu64 ", empty/idle %" PRIu64 ")\n",
                        i + 1, c.replies - p.replies, c.no_response - p.no_response,
                        c.rx_timeouts - p.rx_timeouts, c.empty_captures - p.empty_captures);
            std::printf("    malformed %" PRIu64 " | GCR %" PRIu64 " | CRC %" PRIu64
                        " | overflow %" PRIu64 " | errors %" PRIu64 "\n",
                        c.malformed - p.malformed, c.gcr_errors - p.gcr_errors,
                        c.checksum_errors - p.checksum_errors, c.overflows - p.overflows,
                        c.receive_errors - p.receive_errors);
        }
    }
};
}
#else
#define DSHOT_BENCH_RUNTIME_PROFILE_ENABLED 0
#endif
