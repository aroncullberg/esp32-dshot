#include "dshot/command.hpp"

namespace dshot {
namespace {
constexpr CommandInfo commands[] = {
    {Command::Beep1, "beep1", 1, 260000},
    {Command::Beep2, "beep2", 1, 260000},
    {Command::Beep3, "beep3", 1, 260000},
    {Command::Beep4, "beep4", 1, 260000},
    {Command::Beep5, "beep5", 1, 260000},
    // Six requests also cover AM32's non-beacon command acceptance threshold.
    {Command::EscInfo, "esc-info", 6, 12000, 12000},
    {Command::Direction1, "direction-1", 10, 1000},
    {Command::Direction2, "direction-2", 10, 1000},
    {Command::Mode3DOff, "3d-off", 10, 1000},
    {Command::Mode3DOn, "3d-on", 10, 1000},
    {Command::SaveSettings, "save", 10, 35000},
    {Command::ExtendedTelemetryEnable, "edt-enable", 10, 1000},
    {Command::ExtendedTelemetryDisable, "edt-disable", 10, 1000},
    {Command::DirectionNormal, "direction-normal", 10, 1000},
    {Command::DirectionReversed, "direction-reversed", 10, 1000},
    {Command::Led0On, "led0-on", 1, 1000},
    {Command::Led1On, "led1-on", 1, 1000},
    {Command::Led2On, "led2-on", 1, 1000},
    {Command::Led3On, "led3-on", 1, 1000},
    {Command::Led0Off, "led0-off", 1, 1000},
    {Command::Led1Off, "led1-off", 1, 1000},
    {Command::Led2Off, "led2-off", 1, 1000},
    {Command::Led3Off, "led3-off", 1, 1000},
    {Command::SignalTelemetryDisable, "signal-telemetry-disable", 6, 1000},
    {Command::SignalTelemetryEnable, "signal-telemetry-enable", 6, 1000},
    {Command::ContinuousErpm, "continuous-erpm", 6, 1000},
    {Command::ContinuousErpmPeriod, "continuous-erpm-period", 6, 1000},
    {Command::Temperature, "temperature", 1, 1000},
    {Command::Voltage, "voltage", 1, 1000},
    {Command::Current, "current", 1, 1000},
    {Command::Consumption, "consumption", 1, 1000},
    {Command::Erpm, "erpm", 1, 1000},
    {Command::ErpmPeriod, "erpm-period", 1, 1000},
};
}
const CommandInfo* command_info(Command command) {
    for (const auto& info : commands) if (info.command == command) return &info;
    return nullptr;
}
const char* command_state_name(CommandState state) {
    switch (state) {
    case CommandState::Idle: return "idle";
    case CommandState::Pending: return "pending";
    case CommandState::Preparing: return "preparing";
    case CommandState::Transmitting: return "transmitting";
    case CommandState::Cooldown: return "cooldown";
    case CommandState::Complete: return "complete";
    case CommandState::Cancelled: return "cancelled";
    case CommandState::Failed: return "failed";
    }
    return "unknown";
}
}
