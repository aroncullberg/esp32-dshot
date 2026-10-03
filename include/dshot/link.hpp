#pragma once

#include <cstdint>
#include "dshot/command.hpp"
#include "dshot/telemetry.hpp"
#include "driver/gpio.h"
#include "esp_err.h"

namespace dshot {
class Driver;

class Link {
public:
    Link(Driver& driver, gpio_num_t pin);
    ~Link();

    esp_err_t begin();
    // Closing this link also stops transmission on the driver's other links.
    void end();
    // Nonzero payloads will be rejected during a command sequence or cooldown.
    esp_err_t set(std::uint16_t payload, bool request_telemetry = false);
    // commands require you to do a set(0) first, or rather to stop the motor firstlol
    esp_err_t send_command(Command command);
    CommandStatus command_status() const;
    // EDT requests require bidirectional signaling and a zero payload (like command, since its a command)
    esp_err_t request_edt(bool enabled);
    Telemetry telemetry() const;
    ReceiveCapture receive_capture() const;

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;
    Link(Link&&) = delete;
    Link& operator=(Link&&) = delete;

private:
    friend class Driver;
    friend class Passthrough;
    bool ready() const;
    esp_err_t transmit_pending(std::uint64_t* wait_us, bool& transmitted, std::uint64_t& rx_wait_us);
    void cancel_pending_command();

    struct Impl;
    Impl* impl_ = nullptr;
    Driver& driver_;
    gpio_num_t pin_;
    CommandStatus closed_command_status_;
    Telemetry closed_telemetry_;
};
}
