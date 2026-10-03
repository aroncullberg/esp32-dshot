#include "command_sequence.hpp"
#include <algorithm>

namespace dshot::detail {
bool CommandSequence::active() const {
    return status_.state == CommandState::Pending || status_.state == CommandState::Preparing ||
           status_.state == CommandState::Transmitting || status_.state == CommandState::Cooldown;
}
bool CommandSequence::busy(std::int64_t now) const {
    return active() || command_inflight_ || now < next_command_at_us_;
}
esp_err_t CommandSequence::begin(Command command, std::int64_t now) {
    const auto* info = command_info(command);
    if (!info) return ESP_ERR_INVALID_ARG;
    if (busy(now)) return ESP_ERR_INVALID_STATE;
    const auto id = status_.id + 1;
    status_ = {};
    status_.id = id;
    status_.command = command;
    status_.total_repetitions = info->repetitions;
    status_.state = CommandState::Pending;
    info_ = info;
    ready_at_us_ = 0;
    return ESP_OK;
}
CommandFrame CommandSequence::select(std::uint16_t pending, std::int64_t now) {
    if (!active()) return {true, pending};
    if (status_.state == CommandState::Pending) return {true, 0, status_.id, false, true};
    if (status_.state == CommandState::Preparing) {
        if (now < ready_at_us_) return {true, 0, status_.id};
        status_.state = CommandState::Transmitting;
    }
    if (status_.state == CommandState::Cooldown) {
        if (now >= ready_at_us_) status_.state = CommandState::Complete;
        return {true, 0, status_.id};
    }
    if (command_inflight_ || now < ready_at_us_) return {false, 0, status_.id};
    command_inflight_ = true;
    const auto body = static_cast<std::uint16_t>((static_cast<unsigned>(status_.command) << 1) | 1u);
    return {true, body, status_.id, true};
}
void CommandSequence::complete(const CommandFrame& frame, esp_err_t error, std::int64_t now) {
    if (frame.id == 0 || frame.id != status_.id) return;
    if (frame.command) {
        command_inflight_ = false;
        // Cancellation cannot retract a frame already selected by the worker.
        if (error == ESP_OK) {
            next_command_at_us_ = std::max(next_command_at_us_, now + info_->post_delay_us);
            if (status_.completed_repetitions == 0) status_.first_completed_us = now;
            ++status_.completed_repetitions;
            status_.last_completed_us = now;
        }
    }
    if (!active()) return;
    if (error != ESP_OK) {
        status_.state = CommandState::Failed;
        status_.error = error;
        return;
    }
    if (frame.preamble && status_.state == CommandState::Pending) {
        status_.state = CommandState::Preparing;
        ready_at_us_ = now + 10000;
    }
    if (!frame.command) return;
    if (status_.completed_repetitions == status_.total_repetitions) {
        status_.state = CommandState::Cooldown;
        ready_at_us_ = next_command_at_us_;
    } else {
        ready_at_us_ = now + info_->repeat_interval_us; // Command repetitions have no intervening zero or throttle frames.
    }
}
void CommandSequence::cancel() {
    if (active()) status_.state = CommandState::Cancelled;
}
}
