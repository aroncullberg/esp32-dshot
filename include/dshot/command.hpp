#pragma once
#include <cstdint>
#include "esp_err.h"

namespace dshot {
enum class Command : std::uint8_t {
    Beep1 = 1, Beep2 = 2, Beep3 = 3, Beep4 = 4, Beep5 = 5,
    EscInfo = 6, Direction1 = 7, Direction2 = 8,
    Mode3DOff = 9, Mode3DOn = 10, SaveSettings = 12,
    ExtendedTelemetryEnable = 13, ExtendedTelemetryDisable = 14,
    DirectionNormal = 20, DirectionReversed = 21,
    Led0On = 22, Led1On = 23, Led2On = 24, Led3On = 25,
    Led0Off = 26, Led1Off = 27, Led2Off = 28, Led3Off = 29,
    SignalTelemetryDisable = 32, SignalTelemetryEnable = 33,
    ContinuousErpm = 34, ContinuousErpmPeriod = 35,
    Temperature = 42, Voltage = 43, Current = 44, Consumption = 45,
    Erpm = 46, ErpmPeriod = 47,
};

struct CommandInfo {
    Command command;
    const char* name;
    std::uint8_t repetitions;
    std::uint32_t post_delay_us;
    std::uint32_t repeat_interval_us = 1000;
};
// Null for reserved/unsupported IDs. Describes transmission, not ESC capability.
const CommandInfo* command_info(Command command);

enum class CommandState { Idle, Pending, Preparing, Transmitting, Cooldown, Complete, Cancelled, Failed };
struct CommandStatus {
    std::uint64_t id = 0;
    Command command = Command::Beep1;
    CommandState state = CommandState::Idle;
    std::uint8_t completed_repetitions = 0;
    std::uint8_t total_repetitions = 0;
    std::int64_t first_completed_us = 0;
    std::int64_t last_completed_us = 0;
    esp_err_t error = ESP_OK;
};
const char* command_state_name(CommandState state);
}
