#pragma once

#include "sdkconfig.h"
#include "dshot/statistics.hpp"
#if CONFIG_DSHOT_STAGE_PROFILING
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if !CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS || !CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
#error Stage profiling requires FreeRTOS run-time statistics using ESP Timer
#endif
#endif

namespace dshot::detail {
class StageProfiler {
#if CONFIG_DSHOT_STAGE_PROFILING
    int core_;
    configRUN_TIME_COUNTER_TYPE idle_;
    std::int64_t time_;
    ProfileStage stage_ = ProfileStage::Probe;

    void record() {
        const auto idle = ulTaskGetIdleRunTimeCounterForCore(core_);
        const auto now = esp_timer_get_time();
        const auto elapsed = static_cast<std::uint64_t>(now - time_);
        const configRUN_TIME_COUNTER_TYPE idle_delta = idle - idle_;
        auto& entry = result.stages[static_cast<std::size_t>(stage_)];
        ++entry.count;
        entry.elapsed_us += elapsed;
        entry.busy_us += idle_delta >= elapsed ? 0 : elapsed - idle_delta;
        if (idle_delta > elapsed) ++result.idle_clamps;
        time_ = now;
        idle_ = idle;
    }

public:
    StageProfileStatistics result;

    explicit StageProfiler(int core) : core_(core) {
        // Initialize the result array before starting the empty timestamp probe.
        idle_ = ulTaskGetIdleRunTimeCounterForCore(core);
        time_ = esp_timer_get_time();
    }
    void enter(ProfileStage stage) { record(); stage_ = stage; }
    void finish() { record(); result.sampled_passes = 1; }
#endif
};

inline void profile_stage(StageProfiler* profile, ProfileStage stage) {
#if CONFIG_DSHOT_STAGE_PROFILING
    if (profile) profile->enter(stage);
#else
    (void)profile;
    (void)stage;
#endif
}
}
