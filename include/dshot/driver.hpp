#pragma once

#include "dshot/config.hpp"
#include "dshot/statistics.hpp"
#include "esp_err.h"

namespace dshot {
class Link;

// Owns a configuration snapshot and one repeating transmission task.
// Serialize start/stop and link begin/end from the application. Outlive all links.
class Driver {
public:
    explicit Driver(const Config& config);
    ~Driver();

    esp_err_t start();
    void stop();
    // Last asynchronous TX error; reset when a new worker starts successfully.
    esp_err_t last_error() const;
    // Coherent snapshot, safe alongside the worker. Serialize with lifecycle calls.
    // Reset on start; retained after stop, faults, and link teardown.
    DriverStatistics statistics() const;

    Driver(const Driver&) = delete;
    Driver& operator=(const Driver&) = delete;
    Driver(Driver&&) = delete;
    Driver& operator=(Driver&&) = delete;

private:
    friend class Link;
    esp_err_t prepare_link(const Link& link);
    esp_err_t attach_link(Link& link);
    void detach_link(Link& link);
    esp_err_t send_once(std::uint64_t& wait_us);
    static void worker_entry(void* argument);
    void worker_loop();

    Config config_;
    struct Impl;
    Impl* impl_ = nullptr;
};
}
