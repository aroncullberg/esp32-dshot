#include <cassert>
#include <cstdio>
#include <limits>
#include "stage_profiler.hpp"

namespace {
std::int64_t now_us = 100;
configRUN_TIME_COUNTER_TYPE idle_us = 20;
}
std::int64_t esp_timer_get_time() { return now_us; }
configRUN_TIME_COUNTER_TYPE ulTaskGetIdleRunTimeCounterForCore(BaseType_t core) {
    assert(core == 0);
    return idle_us;
}
int main() {
    using namespace dshot;
    detail::StageProfiler profile(0);
    now_us += 3;
    profile.enter(ProfileStage::TxWait);
    now_us += 1000;
    idle_us += 980; // A long blocking wait must contribute only its non-idle portion.
    profile.enter(ProfileStage::ReplyDecode);
    now_us += 20;
    profile.enter(ProfileStage::CaptureCopy);
    now_us += 5;
    profile.finish();
    const auto& wait = profile.result.stages[static_cast<std::size_t>(ProfileStage::TxWait)];
    const auto& decode = profile.result.stages[static_cast<std::size_t>(ProfileStage::ReplyDecode)];
    const auto& copy = profile.result.stages[static_cast<std::size_t>(ProfileStage::CaptureCopy)];
    assert(wait.count == 1 && wait.elapsed_us == 1000 && wait.busy_us == 20);
    assert(decode.count == 1 && decode.elapsed_us == 20 && decode.busy_us == 20);
    assert(copy.count == 1 && copy.elapsed_us == 5 && copy.busy_us == 5);
    std::uint64_t wall = 0, busy = 0;
    for (const auto& stage : profile.result.stages) { wall += stage.elapsed_us; busy += stage.busy_us; }
    assert(wall == 1028 && busy == 48 && profile.result.sampled_passes == 1);

    idle_us = std::numeric_limits<configRUN_TIME_COUNTER_TYPE>::max() - 4;
    detail::StageProfiler wrapped(0);
    now_us += 20;
    idle_us = 5; // Unsigned idle-counter rollover: ten microseconds elapsed idle.
    wrapped.finish();
    const auto& wrap = wrapped.result.stages[static_cast<std::size_t>(ProfileStage::Probe)];
    assert(wrap.elapsed_us == 20 && wrap.busy_us == 10);
    detail::StageProfiler skew(0);
    now_us += 5;
    idle_us += 6;
    skew.finish();
    assert(skew.result.idle_clamps == 1);
    assert(skew.result.stages[static_cast<std::size_t>(ProfileStage::Probe)].busy_us == 0);
    std::puts("PASS: stage scopes separate blocked idle from busy time, preserve totals, handle rollover and clamp skew");
}
