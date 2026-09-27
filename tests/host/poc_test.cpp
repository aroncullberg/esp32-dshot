// Tests the actual codec, configuration, transport and lifecycle sources against
// fake IDF hardware/RTOS operations. This is not an electrical or real-time test.
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "config_validation.hpp"
#include "idf_transport.hpp"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "frame_scheduler.hpp"

using namespace std::chrono_literals;
using namespace dshot;
using namespace dshot::detail;

struct HostSemaphore {
    std::mutex mutex;
    std::condition_variable changed;
    bool token = false;
};
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
struct rmt_channel_t {
    bool enabled = false;
    bool active = false;
    const rmt_symbol_word_t* buffer = nullptr;
    std::array<std::uint32_t, 16> submitted{};
};

namespace {
std::atomic<int> channels{0}, encoders{0};
std::atomic<bool> fail_channel{false}, fail_encoder{false}, fail_enable{false};
std::atomic<bool> fail_task{false}, fail_submit{false}, fail_wait{false};
std::mutex wire_mutex;
std::condition_variable wire_changed;
bool hold_wait = false, entered_wait = false;
std::vector<std::uint16_t> wire_packets;
std::vector<std::unique_ptr<HostTask>> tasks;
thread_local HostTask* current_task = nullptr;
std::atomic<int> timers{0};
std::atomic<bool> fail_timer_new{false}, fail_timer_register{false}, fail_timer_alarm{false};
std::atomic<bool> fail_timer_enable{false}, fail_timer_start{false};
std::atomic<bool> fail_timer_enable_partial{false};
bool manual_timer = false;
std::atomic<HostTimer*> active_timer{nullptr};
const auto clock_start = std::chrono::steady_clock::now();
std::atomic<std::int64_t> command_clock_us{-1};

void join_tasks() {
    for (auto& task : tasks) task->thread.join();
    tasks.clear();
}
void clean() {
    join_tasks();
    assert(channels == 0 && encoders == 0 && timers == 0);
}
Config config() {
    Config result;
    result.protocol = Protocol::DShot600;
    result.signaling = Signaling::Regular;
    result.frame_hz = 1000;
    result.task_core = -1;
    return result;
}
template<class Predicate> void eventually(Predicate predicate) {
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(1ms);
    assert(predicate());
}
}

SemaphoreHandle_t xSemaphoreCreateBinary() { return new HostSemaphore; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout) {
    std::unique_lock lock(semaphore->mutex);
    if (timeout == portMAX_DELAY) semaphore->changed.wait(lock, [&] { return semaphore->token; });
    else if (!semaphore->changed.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return semaphore->token; })) return pdFALSE;
    semaphore->token = false;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore) {
    std::lock_guard lock(semaphore->mutex);
    if (semaphore->token) return pdFALSE;
    semaphore->token = true;
    semaphore->changed.notify_one();
    return pdTRUE;
}
void vSemaphoreDelete(SemaphoreHandle_t semaphore) { delete semaphore; }
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void*), const char*, unsigned, void* arg,
                                  unsigned, TaskHandle_t* handle, int core) {
    assert(core == -1 || (core >= 0 && core < configNUMBER_OF_CORES));
    if (fail_task.exchange(false)) return pdFALSE;
    auto task = std::make_unique<HostTask>();
    if (handle) *handle = task.get();
    task->thread = std::thread([entry,arg,self=task.get()] {
        current_task = self;
        entry(arg);
    });
    tasks.push_back(std::move(task));
    return pdPASS;
}
void vTaskDelete(TaskHandle_t) {
    std::lock_guard lock(current_task->mutex);
    current_task->deleted = true;
}
TaskHandle_t xTaskGetCurrentTaskHandle() { return current_task; }
BaseType_t xTaskNotifyGive(TaskHandle_t handle) {
    auto& task = *static_cast<HostTask*>(handle);
    std::lock_guard lock(task.mutex);
    assert(!task.deleted);
    ++task.notifications;
    task.changed.notify_one();
    return pdPASS;
}
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t* woken) {
    xTaskNotifyGive(handle);
    *woken = pdTRUE;
}
std::uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t timeout) {
    assert(clear == pdTRUE);
    auto& task = *current_task;
    std::unique_lock lock(task.mutex);
    if (timeout == portMAX_DELAY) {
        task.waiting = true;
        task.changed.wait(lock, [&] { return task.notifications != 0; });
        task.waiting = false;
    } else assert(timeout == 0);
    const auto result = task.notifications;
    task.notifications = 0;
    return result;
}
TickType_t xTaskGetTickCount() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - clock_start).count();
}
std::int64_t esp_timer_get_time() {
    if (const auto now = command_clock_us.load(); now >= 0) return now;
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - clock_start).count();
}
BaseType_t xTaskDelayUntil(TickType_t*, TickType_t) {
    std::abort(); // The new worker must never use tick pacing.
}

esp_err_t gptimer_new_timer(const gptimer_config_t* config, gptimer_handle_t* out) {
    assert(config->resolution_hz == 1000000 && config->direction == GPTIMER_COUNT_UP);
    if (fail_timer_new.exchange(false)) return ESP_ERR_NOT_FOUND;
    *out = new HostTimer;
    ++timers;
    return ESP_OK;
}
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t timer,
                                          const gptimer_event_callbacks_t* callbacks, void* context) {
    assert(!timer->enabled);
    if (fail_timer_register.exchange(false)) return ESP_FAIL;
    timer->callback = callbacks->on_alarm;
    timer->context = context;
    return ESP_OK;
}
esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t* alarm) {
    assert(alarm->flags.auto_reload_on_alarm && alarm->reload_count == 0);
    if (fail_timer_alarm.exchange(false)) return ESP_FAIL;
    timer->period_us = alarm->alarm_count;
    return ESP_OK;
}
esp_err_t gptimer_enable(gptimer_handle_t timer) {
    assert(!timer->enabled && timer->callback && timer->period_us);
    if (fail_timer_enable.exchange(false)) return ESP_FAIL;
    timer->enabled = true;
    if (fail_timer_enable_partial.exchange(false)) return ESP_FAIL;
    return ESP_OK;
}
void emit_alarm(HostTimer* timer) {
    const gptimer_alarm_event_data_t event{0,timer->period_us};
    timer->callback(timer, &event, timer->context);
}
esp_err_t gptimer_start(gptimer_handle_t timer) {
    assert(timer->enabled && !timer->running);
    if (fail_timer_start.exchange(false)) return ESP_FAIL;
    timer->running = true;
    active_timer = timer;
    if (!manual_timer) timer->thread = std::thread([timer] {
        std::unique_lock lock(timer->mutex);
        auto next = std::chrono::steady_clock::now() + std::chrono::microseconds(timer->period_us);
        while (timer->running) {
            if (timer->changed.wait_until(lock,next,[&] { return !timer->running; })) break;
            lock.unlock();
            emit_alarm(timer);
            lock.lock();
            next = std::chrono::steady_clock::now() + std::chrono::microseconds(timer->period_us);
        }
    });
    return ESP_OK;
}
esp_err_t gptimer_stop(gptimer_handle_t timer) {
    assert(timer->running);
    {
        std::lock_guard lock(timer->mutex);
        timer->running = false;
    }
    timer->changed.notify_all();
    if (timer->thread.joinable()) timer->thread.join();
    active_timer = nullptr;
    return ESP_OK;
}
esp_err_t gptimer_disable(gptimer_handle_t timer) {
    if (!timer->enabled) return ESP_ERR_INVALID_STATE;
    assert(!timer->running);
    timer->enabled = false;
    return ESP_OK;
}
esp_err_t gptimer_del_timer(gptimer_handle_t timer) {
    assert(!timer->enabled && !timer->running);
    delete timer;
    --timers;
    return ESP_OK;
}

extern "C" {
const char* esp_err_to_name(esp_err_t error) {
    return error == ESP_OK ? "ESP_OK" : error == ESP_ERR_TIMEOUT ? "ESP_ERR_TIMEOUT" : "injected/error";
}
esp_err_t gpio_set_direction(gpio_num_t gpio, gpio_mode_t mode) {
    assert(GPIO_IS_VALID_OUTPUT_GPIO(gpio) && mode == GPIO_MODE_DISABLE);
    return ESP_OK;
}
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t* config, rmt_channel_handle_t* out) {
    assert(config->resolution_hz == 40000000 && config->mem_block_symbols == 48);
    assert(config->trans_queue_depth == 1 && !config->flags.invert_out && !config->flags.with_dma);
    assert(config->flags.init_level == 0);
    if (fail_channel.exchange(false)) return ESP_ERR_NO_MEM;
    *out = new rmt_channel_t;
    ++channels;
    return ESP_OK;
}
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t*, rmt_encoder_handle_t* out) {
    if (fail_encoder.exchange(false)) return ESP_ERR_NO_MEM;
    *out = new rmt_encoder_t;
    ++encoders;
    return ESP_OK;
}
esp_err_t rmt_enable(rmt_channel_handle_t channel) {
    if (fail_enable.exchange(false)) return ESP_FAIL;
    assert(!channel->enabled);
    channel->enabled = true;
    return ESP_OK;
}
esp_err_t rmt_disable(rmt_channel_handle_t channel) {
    assert(channel->enabled);
    channel->active = false;
    channel->buffer = nullptr;
    channel->enabled = false;
    return ESP_OK;
}
esp_err_t rmt_del_channel(rmt_channel_handle_t channel) {
    assert(!channel->enabled && !channel->active);
    delete channel;
    --channels;
    return ESP_OK;
}
esp_err_t rmt_del_encoder(rmt_encoder_handle_t encoder) {
    delete encoder;
    --encoders;
    return ESP_OK;
}
esp_err_t rmt_transmit(rmt_channel_handle_t channel, rmt_encoder_handle_t,
                       const void* payload, size_t size, const rmt_transmit_config_t* config) {
    assert(channel->enabled && !channel->active);
    assert(size == 16 * sizeof(rmt_symbol_word_t));
    assert(config->loop_count == 0 && config->flags.eot_level == 0 && config->flags.queue_nonblocking);
    if (fail_submit.exchange(false)) return ESP_FAIL;
    channel->active = true;
    channel->buffer = static_cast<const rmt_symbol_word_t*>(payload);
    for (unsigned i = 0; i < 16; ++i) channel->submitted[i] = channel->buffer[i].val;
    return ESP_OK;
}
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t channel, int timeout_ms) {
    assert(timeout_ms == 20 && channel->active);
    if (fail_wait.exchange(false)) return ESP_ERR_TIMEOUT; // Remains active until abort.
    std::unique_lock lock(wire_mutex);
    entered_wait = true;
    wire_changed.notify_all();
    wire_changed.wait(lock, [] { return !hold_wait; });
    std::uint16_t packet = 0;
    for (unsigned i = 0; i < 16; ++i) {
        const auto& symbol = channel->buffer[i];
        assert(symbol.val == channel->submitted[i]); // No overwrite during pending TX.
        assert(symbol.level0 == 1 && symbol.level1 == 0);
        assert(symbol.duration0 + symbol.duration1 == 67);
        assert(symbol.duration0 == 25 || symbol.duration0 == 50);
        packet = (packet << 1) | (symbol.duration0 == 50);
    }
    wire_packets.push_back(packet);
    channel->active = false;
    channel->buffer = nullptr;
    wire_changed.notify_all();
    return ESP_OK;
}
}

void test_codec() {
    const struct { std::uint16_t payload; bool request; std::uint16_t packet; } cases[] = {
        {0,false,0}, {48,false,0x0606}, {48,true,0x0617}, {100,false,0x0C84},
        {2047,false,0xFFEE}, {2047,true,0xFFFF}
    };
    for (const auto& test : cases) {
        std::uint16_t packet = 123;
        assert(encode_regular_packet(test.payload,test.request,packet) == ESP_OK);
        assert(packet == test.packet);
    }
    std::uint16_t packet = 0x1234;
    assert(encode_regular_packet(2048,false,packet) == ESP_ERR_INVALID_ARG && packet == 0x1234);
    assert(encode_regular_packet(65535,false,packet) == ESP_ERR_INVALID_ARG && packet == 0x1234);
    constexpr char bits[] = "0000011000000110";
    for (unsigned i = 0; i < 16; ++i) assert(packet_bit(0x0606,i) == (bits[i] == '1'));
    BitTiming timing{};
    for (const auto protocol : {Protocol::DShot150, Protocol::DShot300, Protocol::DShot600}) {
        assert(calculate_bit_timing(protocol,40000000,timing) == ESP_OK);
        const auto expected = protocol == Protocol::DShot150 ? 267u : protocol == Protocol::DShot300 ? 133u : 67u;
        assert(timing.bit_total_ticks == expected);
        assert(timing.bit_0_high_ticks + timing.bit_0_low_ticks == expected);
        assert(timing.bit_1_high_ticks + timing.bit_1_low_ticks == expected);
    }
    const auto before = timing;
    assert(calculate_bit_timing(Protocol::DShot600,1,timing) == ESP_ERR_NOT_SUPPORTED);
    assert(timing.bit_total_ticks == before.bit_total_ticks);
    assert(calculate_bit_timing(Protocol::DShot600,0,timing) == ESP_ERR_INVALID_ARG);
    assert(calculate_bit_timing(static_cast<Protocol>(42),40000000,timing) == ESP_ERR_INVALID_ARG);
    SymbolFrame symbols{};
    assert(encode_regular_symbols(0x0606,timing,symbols) == ESP_OK);
    for (unsigned i = 0; i < 16; ++i) {
        assert(symbols[i].duration0 == (bits[i]=='1' ? 50 : 25));
        assert(symbols[i].duration1 == (bits[i]=='1' ? 17 : 42));
        assert(symbols[i].level0 == 1 && symbols[i].level1 == 0);
    }
    const auto prior = symbols;
    for (unsigned failure = 0; failure < 3; ++failure) {
        auto invalid = timing;
        if (failure == 0) invalid.bit_0_high_ticks = 0;
        if (failure == 1) invalid.bit_1_high_ticks = UINT32_MAX;
        if (failure == 2) invalid.bit_total_ticks = 68;
        assert(encode_regular_symbols(0x0606,invalid,symbols) == ESP_ERR_INVALID_ARG);
        for (unsigned i = 0; i < 16; ++i) assert(symbols[i].val == prior[i].val);
    }
}

void test_config_and_transport() {
    auto c = config();
    assert(validate_config(c) == ESP_OK);
    for (const auto hz : {100u,1000u,2000u,3333u,4000u,8000u}) {
        c.frame_hz = hz; assert(validate_config(c) == ESP_OK);
    }
    c.frame_hz = 8001; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
    c.frame_hz = 0; assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    c = config(); c.task_core = configNUMBER_OF_CORES; assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    c = config(); c.signaling = Signaling::Bidirectional; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
    c.signaling = static_cast<Signaling>(42); assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    for (auto* failure : {&fail_channel,&fail_encoder,&fail_enable}) {
        RmtTransport transport;
        failure->store(true);
        assert(open_transport(transport,GPIO_NUM_2,config()) != ESP_OK);
        assert(channels == 0 && encoders == 0);
        assert(close_transport(transport) == ESP_OK);
        assert(open_transport(transport,GPIO_NUM_2,config()) == ESP_OK);
        assert(open_transport(transport,GPIO_NUM_2,config()) == ESP_ERR_INVALID_STATE);
        fail_wait = true;
        assert(send_packet(transport,0) == ESP_ERR_TIMEOUT);
        assert(!transport.in_flight && !transport.enabled && transport.faulted);
        assert(send_packet(transport,0) == ESP_ERR_INVALID_STATE);
        assert(close_transport(transport) == ESP_OK);
        assert(close_transport(transport) == ESP_OK);
    }
    clean();
}

void test_lifecycle() {
    Driver driver(config());
    assert(!driver.statistics().running && driver.statistics().elapsed_us == 0);
    Link link(driver,GPIO_NUM_2);
    Link duplicate(driver,GPIO_NUM_2);
    assert(driver.start() == ESP_ERR_INVALID_STATE);
    driver.stop(); link.end();
    assert(link.set(48) == ESP_ERR_INVALID_STATE);
    for (unsigned cycle = 0; cycle < 5; ++cycle) {
        assert(link.begin() == ESP_OK);
        assert(link.begin() == ESP_ERR_INVALID_STATE);
        assert(duplicate.begin() == ESP_ERR_INVALID_STATE);
        assert(link.set(48) == ESP_OK);
        assert(link.set(2048) == ESP_ERR_INVALID_ARG);
        fail_task = true;
        assert(driver.start() == ESP_ERR_NO_MEM);
        driver.stop();
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = true; entered_wait = false; wire_packets.clear();
        }
        assert(driver.start() == ESP_OK);
        assert(driver.start() == ESP_ERR_INVALID_STATE);
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        auto pending = driver.statistics();
        assert(pending.running && pending.link_count == 1);
        assert(pending.links[0].gpio == GPIO_NUM_2);
        assert(pending.links[0].completed_frames == 0); // Submission is not completion.
        // Changing the pending request cannot rewrite the active RMT symbol buffer.
        assert(link.set(0) == ESP_OK);
        auto stopping = std::async(std::launch::async,[&] { driver.stop(); });
        assert(stopping.wait_for(5ms) == std::future_status::timeout);
        {
            std::lock_guard lock(wire_mutex); hold_wait = false;
        }
        wire_changed.notify_all();
        assert(stopping.wait_for(2s) == std::future_status::ready);
        stopping.get();
        const auto stopped = driver.statistics();
        assert(!stopped.running && stopped.links[0].completed_frames == 1);
        assert(stopped.links[0].tx_errors == 0 && stopped.elapsed_us >= 5000);
        std::this_thread::sleep_for(1ms);
        assert(driver.statistics().elapsed_us == stopped.elapsed_us); // Frozen after stop.
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        assert(stopped.detailed_timing && stopped.links[0].tx_wait.count == 1);
        assert(stopped.links[0].tx_wait.total_us >= 5000);
        assert(stopped.worker_pass.total_us == stopped.outside_tx_wait.total_us +
               stopped.links[0].tx_wait.total_us);
        assert(stopped.links[0].completion_interval.count == 0);
#else
        assert(!stopped.detailed_timing && stopped.worker_pass.count == 0);
#endif
        {
            std::lock_guard lock(wire_mutex);
            assert(wire_packets.size() == 1 && wire_packets.front() == 0x0606);
        }
        driver.stop();
        // Restart still-open links, then end while running (which joins first).
        assert(driver.start() == ESP_OK);
        link.end(); link.end();
        assert(link.set(0) == ESP_ERR_INVALID_STATE);
        clean();
    }
    assert(link.begin() == ESP_OK);
    fail_submit = true;
    assert(driver.start() == ESP_OK);
    eventually([&] { return driver.last_error() == ESP_FAIL; });
    driver.stop(); link.end(); clean();
    const auto faulted = driver.statistics();
    assert(!faulted.running && faulted.links[0].tx_errors == 1);
    assert(faulted.links[0].completed_frames == 0); // Retained after link.end().
    assert(link.begin() == ESP_OK);
    fail_wait = true;
    assert(driver.start() == ESP_OK);
    eventually([&] { return driver.last_error() == ESP_ERR_TIMEOUT; });
    driver.stop(); link.end(); clean();
    assert(driver.statistics().links[0].tx_errors == 1);
    assert(driver.statistics().links[0].completed_frames == 0);
    assert(link.begin() == ESP_OK);
    assert(driver.start() == ESP_OK);
    assert(driver.last_error() == ESP_OK);
    link.end(); clean();
    // Two distinct links are legal; duplicate GPIO and attachment while running are not.
    Link second(driver,GPIO_NUM_3);
    assert(link.begin() == ESP_OK && second.begin() == ESP_OK);
    {
        std::lock_guard lock(wire_mutex);
        hold_wait = true; entered_wait = false;
    }
    assert(driver.start() == ESP_OK);
    eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
    std::this_thread::sleep_for(10ms); // Force missed slots without catching up.
    {
        std::lock_guard lock(wire_mutex);
        hold_wait = false;
    }
    wire_changed.notify_all();
    eventually([&] {
        const auto snapshot = driver.statistics();
        return snapshot.missed_slots >= 5 && snapshot.links[1].completed_frames >= 2;
    });
    const auto multiple = driver.statistics();
    assert(multiple.link_count == 2 && multiple.links[1].gpio == GPIO_NUM_3);
    assert(multiple.links[0].completed_frames >= multiple.links[1].completed_frames);
    assert(multiple.links[0].completed_frames <= multiple.links[1].completed_frames + 1);
    assert(multiple.links[0].tx_errors == 0 && multiple.links[1].tx_errors == 0);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& stats = multiple.links[i];
        assert(stats.completion_interval.count == stats.completed_frames - 1);
        assert(stats.tx_wait.count == stats.completed_frames);
        assert(stats.tx_wait.min_us <= stats.tx_wait.max_us);
    }
#endif
    Link late(driver,GPIO_NUM_4);
    assert(late.begin() == ESP_ERR_INVALID_STATE);
    second.end(); link.end(); clean();
}

void run_learning_checks();
void run_command_sequence_tests();

void test_command_integration() {
    manual_timer = true;
    command_clock_us = 1000000;
    {
        auto c = config(); c.frame_hz = 4000;
        Driver driver(c);
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        assert(link.set(100) == ESP_OK);
        assert(link.send_command(Command::DirectionNormal) == ESP_ERR_INVALID_STATE);
        assert(link.set(0) == ESP_OK);
        assert(link.send_command(Command::Beep1) == ESP_OK);
        driver.stop(); // Cancels a queued command even before any worker exists.
        assert(link.command_status().state == CommandState::Cancelled);
        assert(driver.start() == ESP_OK);
        eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
        auto step = [&](std::int64_t now) {
            command_clock_us = now;
            const auto before = driver.statistics().worker_iterations;
            emit_alarm(active_timer.load());
            eventually([&] { return driver.statistics().worker_iterations > before && tasks.back()->waiting.load(); });
        };
        {
            std::lock_guard lock(wire_mutex); wire_packets.clear();
        }
        assert(link.send_command(Command::DirectionNormal) == ESP_OK);
        assert(link.set(200) == ESP_ERR_INVALID_STATE);
        step(1000000);
        step(1009999);
        step(1010000);
        assert(link.command_status().completed_repetitions == 1);
        const auto completed = driver.statistics().links[0].completed_frames;
        step(1010250);
        assert(driver.statistics().links[0].completed_frames == completed);
        assert(driver.statistics().links[0].command_pauses == 1);
        for (unsigned i = 1; i < 6; ++i) step(1010000 + i*1000);
        step(1015999);
        assert(link.command_status().state == CommandState::Cooldown);
        step(1016000);
        assert(link.command_status().state == CommandState::Complete);
        {
            std::lock_guard lock(wire_mutex);
            assert(wire_packets.size() == 10);
            for (unsigned i = 0; i < 10; ++i) {
                assert(wire_packets[i] == (i >= 2 && i < 8 ? 0x029B : 0));
            }
        }
        assert(link.send_command(Command::Beep1) == ESP_OK);
        step(1020000);
        {
            std::lock_guard lock(wire_mutex); hold_wait = true; entered_wait = false;
        }
        command_clock_us = 1030000;
        emit_alarm(active_timer.load());
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        assert(link.command_status().completed_repetitions == 0);
        assert(link.set(0) == ESP_OK); // Cancel during RMT's active frame.
        assert(link.send_command(Command::Direction2) == ESP_ERR_INVALID_STATE);
        {
            std::lock_guard lock(wire_mutex); hold_wait = false;
        }
        wire_changed.notify_all();
        eventually([] { return tasks.back()->waiting.load(); });
        assert(link.command_status().state == CommandState::Cancelled);
        assert(link.command_status().completed_repetitions == 1);
        assert(link.set(100) == ESP_ERR_INVALID_STATE); // Already-sent beep retains its cooldown.
        command_clock_us = 1290001;
        assert(link.set(100) == ESP_OK);
        assert(link.set(0) == ESP_OK);
        assert(link.send_command(Command::SaveSettings) == ESP_OK);
        step(1300000);
        fail_wait = true;
        command_clock_us = 1310000;
        emit_alarm(active_timer.load());
        eventually([&] { return driver.last_error() == ESP_ERR_TIMEOUT; });
        driver.stop();
        assert(link.command_status().state == CommandState::Failed);
        assert(link.command_status().completed_repetitions == 0);
        link.end();
        assert(link.command_status().state == CommandState::Failed);
        clean();
        assert(link.begin() == ESP_OK && driver.start() == ESP_OK);
        assert(link.send_command(Command::Direction2) == ESP_OK);
        driver.stop();
        assert(link.command_status().state == CommandState::Cancelled);
        link.end(); clean();
    }
    {
        Driver driver(config());
        Link command_link(driver,GPIO_NUM_2), other(driver,GPIO_NUM_3);
        assert(command_link.begin() == ESP_OK && other.begin() == ESP_OK);
        assert(other.set(100) == ESP_OK);
        assert(command_link.send_command(Command::Direction1) == ESP_OK);
        assert(driver.start() == ESP_OK);
        eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
        auto step = [&](std::int64_t now) {
            command_clock_us = now;
            const auto before = driver.statistics().worker_iterations;
            emit_alarm(active_timer.load());
            eventually([&] { return driver.statistics().worker_iterations > before && tasks.back()->waiting.load(); });
        };
        step(2000000); // Command preamble, other link's ordinary payload.
        step(2010000); // First command frame.
        const auto before = driver.statistics();
        step(2010250); // Command gap must not pause the other link.
        const auto after = driver.statistics();
        assert(after.links[0].completed_frames == before.links[0].completed_frames);
        assert(after.links[0].command_pauses == 1);
        assert(after.links[1].completed_frames == before.links[1].completed_frames + 1);
        assert(after.links[1].command_pauses == 0);
        driver.stop(); command_link.end(); other.end(); clean();
    }
    command_clock_us = -1;
    manual_timer = false;
}

void test_scheduler() {
    // Setup failures release partial timer state, and retry on the same link works.
    for (auto* failure : {&fail_timer_new,&fail_timer_register,&fail_timer_alarm,
                          &fail_timer_enable,&fail_timer_enable_partial}) {
        Driver driver(config());
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        failure->store(true);
        assert(driver.start() != ESP_OK && timers == 0);
        assert(driver.start() == ESP_OK);
        link.end(); clean();
    }
    {
        Driver driver(config());
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        fail_timer_start = true;
        assert(driver.start() == ESP_OK);
        eventually([&] { return driver.last_error() == ESP_FAIL; });
        driver.stop();
        assert(timers == 0 && driver.statistics().links[0].completed_frames == 0);
        assert(driver.start() == ESP_OK);
        link.end(); clean();
    }

    // Drive alarms explicitly: host thread scheduling is not a timing benchmark.
    manual_timer = true;
    for (const auto hz : {1000u,2000u,4000u,8000u,3333u,1u}) {
        auto c = config(); c.frame_hz = hz;
        Driver driver(c);
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK && driver.start() == ESP_OK);
        eventually([] { return active_timer.load() != nullptr; });
        auto* timer = active_timer.load();
        assert(timer->period_us == (1000000u + hz/2) / hz);
        assert(driver.statistics().timer_period_us == timer->period_us);
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = true; entered_wait = false;
        }
        emit_alarm(timer);
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        // Seven alarms while TX is blocked must not produce seven catch-up frames.
        for (unsigned i = 0; i < 7; ++i) emit_alarm(timer);
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = false;
        }
        wire_changed.notify_all();
        eventually([&] { return driver.statistics().missed_slots == 7; });
        std::this_thread::sleep_for(2ms);
        assert(driver.statistics().links[0].completed_frames == 1);
        emit_alarm(timer);
        eventually([&] { return driver.statistics().links[0].completed_frames == 2; });
        // No more alarms are forthcoming. stop must explicitly wake the worker.
        auto stopped = std::async(std::launch::async,[&] { driver.stop(); });
        assert(stopped.wait_for(200ms) == std::future_status::ready);
        stopped.get();
        link.end(); clean();
    }
    manual_timer = false;
}

int main() {
    run_learning_checks();
    test_codec(); std::puts("PASS: packet, bit order, all bitrates, timing and symbol validation");
    test_config_and_transport(); std::puts("PASS: configuration, partial setup rollback, timeout abort and close");
    test_lifecycle(); std::puts("PASS: lifecycle, statistics reset/retention, multi-link counts and missed slots");
    test_scheduler(); std::puts("PASS: GPTimer rollback/restart, 1/2/4/8 kHz periods at 100 Hz RTOS tick, dropped backlog, stop wake-up");
    run_command_sequence_tests();
    test_command_integration();
    std::puts("PASS: command packets, completed repetitions, quiet gaps, cancellation, cooldowns and faults");
    clean();
}
