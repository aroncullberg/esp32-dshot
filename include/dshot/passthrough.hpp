#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include "esp_err.h"

namespace dshot {
class Driver;
class Link;

// Releases motor pins for another peripheral.
class Passthrough {
public:
    Passthrough(Driver& driver, Link* const* links, std::size_t count);
    Passthrough(Driver& driver, std::initializer_list<Link*> links);
    ~Passthrough();

    // Closes every link before handing over the motor pins.
    esp_err_t acquire();
    // Call after the other peripheral releases the pins.
    // failiure to restart one link will keep all links closed.
    esp_err_t release(bool resume_zero_frames);
    bool held() const { return held_; }

    // ctx points to this Passthrough object.
    static esp_err_t acquire_callback(void* ctx);
    static esp_err_t release_callback(void* ctx, bool resume_zero_frames);

    Passthrough(const Passthrough&) = delete;
    Passthrough& operator=(const Passthrough&) = delete;
    Passthrough(Passthrough&&) = delete;
    Passthrough& operator=(Passthrough&&) = delete;

private:
    void close_links();
    Driver& driver_;
    std::array<Link*, 4> links_{};
    std::size_t count_;
    bool held_ = false;
};
}
