#pragma once
#include "dshot/command.hpp"

namespace dshot::detail {
struct CommandFrame {
    bool send = true;
    std::uint16_t body = 0;
    std::uint64_t id = 0;
    bool command = false;
    bool preamble = false;
};

// Pure state machine. The owning Link serializes access and supplies time.
class CommandSequence {
public:
    esp_err_t begin(Command command, std::int64_t now);
    bool active() const;
    bool busy(std::int64_t now) const;
    CommandFrame select(std::uint16_t pending_body, std::int64_t now);
    void complete(const CommandFrame& frame, esp_err_t error, std::int64_t now);
    void cancel();
    CommandStatus status() const { return status_; }
private:
    CommandStatus status_;
    const CommandInfo* info_ = nullptr;
    std::int64_t ready_at_us_ = 0;
    std::int64_t next_command_at_us_ = 0;
    bool command_inflight_ = false;
};
}
