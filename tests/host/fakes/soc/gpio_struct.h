#pragma once
#include <array>
#include <atomic>
struct gpio_dev_t {
    std::array<std::atomic<bool>, 49> enabled{};
    std::array<std::atomic<bool>, 49> peripheral_oe{};
    std::array<std::atomic<bool>, 49> rmt_routed{};
};
inline gpio_dev_t GPIO;
inline bool pad_driven(unsigned pin) {
    // With peripheral output enable selected, RMT keeps the pad driven at idle.
    return GPIO.peripheral_oe.at(pin) || GPIO.enabled.at(pin);
}
