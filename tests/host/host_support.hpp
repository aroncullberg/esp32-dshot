#pragma once
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "dshot/config.hpp"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

struct HostTask {
    std::mutex mutex;
    std::condition_variable changed;
    std::uint32_t notifications = 0;
    bool deleted = false;
    std::atomic<bool> waiting{false};
    std::thread thread;
};
struct HostTimer {
    std::mutex mutex;
    std::condition_variable changed;
    std::atomic<bool> running{false};
    bool enabled = false;
    unsigned period_us = 0;
    gptimer_alarm_cb_t callback = nullptr;
    void* context = nullptr;
    std::thread thread;
};
namespace host_test {
extern std::atomic<int> channels, encoders;
extern std::atomic<bool> fail_channel, fail_encoder, fail_enable;
extern std::atomic<bool> fail_task, fail_submit, fail_wait;
extern int fail_tx_channel_after;
extern int fail_submit_after;
extern std::atomic<unsigned> max_in_flight, blocking_tx_waits;
extern std::mutex wire_mutex;
extern std::condition_variable wire_changed;
extern bool hold_wait, entered_wait;
extern std::vector<std::uint16_t> wire_packets;
extern std::vector<std::unique_ptr<HostTask>> tasks;
extern std::atomic<int> timers;
extern std::atomic<bool> fail_timer_new, fail_timer_register, fail_timer_alarm;
extern std::atomic<bool> fail_timer_enable, fail_timer_start;
extern std::atomic<bool> fail_timer_enable_partial;
extern bool manual_timer;
extern std::atomic<HostTimer*> active_timer;
extern std::atomic<std::int64_t> command_clock_us;
extern std::atomic<std::int64_t> tick_clock;
extern TickType_t last_rx_wait_ticks;
extern bool silence, bad_reply_crc, rx_overflow, fail_rx_new;
extern bool empty_reply;
extern bool stall_receive;
extern int stall_rx_gpio;
extern bool fail_receive, fail_rx_register, fail_tx_register, fail_rx_enable;
extern std::uint16_t reply_payload;
extern bool fail_startup_level, fail_startup_output;
extern unsigned startup_pulses;
extern TickType_t startup_delay_ticks;

void clean();
dshot::Config config();
void emit_alarm(HostTimer* timer);
void complete_tx(gpio_num_t gpio);

template<class Predicate> void eventually(Predicate predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(predicate());
}
}
