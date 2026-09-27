#pragma once

#include <cstdint>
#include "dshot/command.hpp"
#include "driver/gpio.h"
#include "esp_err.h"

namespace dshot {
class Driver;

// One GPIO and its latest pending payload. Driver must outlive this object.
// Serialize lifecycle calls and setters; the TX worker snapshots concurrently.
class Link {
public:
    Link(Driver& driver, gpio_num_t pin);
    ~Link();

    esp_err_t begin();
    // Stops the associated driver before releasing this link; safe to repeat.
    void end();
    // Publishes a raw value, not a command-completion or ESC acknowledgement.
    esp_err_t set(std::uint16_t payload, bool request_telemetry = false);
    // Requires pending payload zero. One sequence at a time; set(0) cancels it.
    // Nonzero set() returns INVALID_STATE while a sequence/cooldown is pending.
    esp_err_t send_command(Command command);
    CommandStatus command_status() const;

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;
    Link(Link&&) = delete;
    Link& operator=(Link&&) = delete;

private:
    friend class Driver;
    bool ready() const;
    esp_err_t transmit_pending(std::uint64_t* wait_us, bool& transmitted);
    void cancel_pending_command();

    struct Impl;
    Impl* impl_ = nullptr;
    Driver& driver_;
    gpio_num_t pin_;
    CommandStatus closed_command_status_;
};
}
