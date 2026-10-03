#pragma once

#include <cstddef>
#include "dshot/config.hpp"
#include "dshot/statistics.hpp"
#include "esp_err.h"

namespace dshot {
class Link;
class Passthrough;

// Sends DShot frames at the configured rate.
class Driver {
public:
    explicit Driver(const Config& config);
    ~Driver();

    esp_err_t start();
    void stop();
    // Cleared before creating a new transmission task.
    esp_err_t last_error() const;
    DriverStatistics statistics() const;

    Driver(const Driver&) = delete;
    Driver& operator=(const Driver&) = delete;
    Driver(Driver&&) = delete;
    Driver& operator=(Driver&&) = delete;

private:
    friend class Link;
    friend class Passthrough;
    esp_err_t validate_passthrough_links(Link* const* links, std::size_t count) const;
    esp_err_t prepare_link(const Link& link);
    esp_err_t attach_link(Link& link);
    void detach_link(Link& link);
    esp_err_t send_once(std::uint64_t& wait_us);
    static void worker_entry(void* argument);
    void worker_loop();

    Config config_;
    struct Impl;
    Impl* impl_ = nullptr;
    Passthrough* passthrough_owner_ = nullptr;
};
}
