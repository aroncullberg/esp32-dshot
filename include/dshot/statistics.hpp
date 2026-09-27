#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dshot {
// Elapsed microseconds, including preemption. An empty sample has count == 0.
struct TimingStatistics {
    std::uint64_t count = 0;
    std::uint64_t total_us = 0;
    std::uint64_t min_us = 0;
    std::uint64_t max_us = 0;
};

struct LinkStatistics {
    int gpio = -1;
    std::uint64_t completed_frames = 0;
    std::uint64_t tx_errors = 0;
    std::uint64_t command_pauses = 0;
    TimingStatistics outside_tx_wait;
    TimingStatistics tx_wait;
    // Intervals between successful completion observations, not GPIO edges.
    TimingStatistics completion_interval;
};

struct DriverStatistics {
    // Maximum on the supported targets; link_count identifies populated entries.
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
};
}
