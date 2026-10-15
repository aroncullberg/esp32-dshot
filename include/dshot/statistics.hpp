#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dshot {
// Elapsed microseconds, including preemption.
struct TimingStatistics {
    std::uint64_t count = 0;
    std::uint64_t total_us = 0;
    std::uint64_t min_us = 0;
    std::uint64_t max_us = 0;
};

enum class ProfileStage : std::size_t {
    Probe, FramePrepare, SymbolEncode, RxSetup, TxSubmit, TxWait, RxWait,
    RxFinish, EchoStrip, ReplyDecode, TelemetryUpdate, CaptureCopy, LinkFinish, Bookkeeping, Count
};
struct StageTimingStatistics {
    std::uint64_t count = 0;
    std::uint64_t elapsed_us = 0;
    // Elapsed time minus this core's accounted idle time, not exclusive task/ISR time.
    std::uint64_t busy_us = 0;
};
struct StageProfileStatistics {
    bool available = false;
    unsigned sample_interval = 0;
    std::uint64_t sampled_passes = 0;
    std::uint64_t idle_clamps = 0;
    std::array<StageTimingStatistics, static_cast<std::size_t>(ProfileStage::Count)> stages{};
};

struct LinkStatistics {
    int gpio = -1;
    std::uint64_t completed_frames = 0;
    std::uint64_t tx_errors = 0;
    std::uint64_t command_pauses = 0;
    // Per-link submission-to-collection time minus its TX queue wait. Exchanges
    // overlap across links; these elapsed times must not be summed across motors.
    TimingStatistics outside_tx_wait;
    TimingStatistics tx_wait;
    TimingStatistics rx_wait; // Time waiting for RX completion, excluding decode.
    // Time between transmission completions as observed by the worker.
    TimingStatistics completion_interval;
};

struct DriverStatistics {
    static constexpr std::size_t max_links = 4;
    bool running = false;
    bool detailed_timing = false;
    std::uint32_t timer_period_us = 0;
    std::uint64_t elapsed_us = 0;
    std::uint64_t worker_iterations = 0;
    std::uint64_t missed_slots = 0;
    TimingStatistics worker_pass;
    TimingStatistics outside_tx_wait;
    std::size_t link_count = 0;
    std::array<LinkStatistics, max_links> links{};
    StageProfileStatistics stage_profile;
};
}
