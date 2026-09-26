#pragma once

#include <cstdint>
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

    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;
    Link(Link&&) = delete;
    Link& operator=(Link&&) = delete;

private:
    friend class Driver;
    bool ready() const;
    std::uint16_t snapshot_pending();
    esp_err_t transmit_pending();

    struct Impl;
    Impl* impl_ = nullptr;
    Driver& driver_;
    gpio_num_t pin_;
};
}
