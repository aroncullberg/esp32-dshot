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

using namespace std::chrono_literals;
using namespace dshot;
using namespace dshot::detail;

struct HostSemaphore {
    std::mutex mutex;
    std::condition_variable changed;
    bool token = false;
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
std::vector<std::unique_ptr<std::thread>> tasks;
const auto clock_start = std::chrono::steady_clock::now();

void join_tasks() {
    for (auto& task : tasks) task->join();
    tasks.clear();
}
void clean() {
    join_tasks();
    assert(channels == 0 && encoders == 0);
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
    auto task = std::make_unique<std::thread>(entry, arg);
    *handle = task.get();
    tasks.push_back(std::move(task));
    return pdPASS;
}
void vTaskDelete(TaskHandle_t) {}
TickType_t xTaskGetTickCount() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - clock_start).count();
}
BaseType_t xTaskDelayUntil(TickType_t* last, TickType_t period) {
    *last += period;
    const auto now = xTaskGetTickCount();
    if (*last <= now) return pdFALSE;
    std::this_thread::sleep_for(std::chrono::milliseconds(*last - now));
    return pdTRUE;
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
    c.frame_hz = 4000; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
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
    assert(link.begin() == ESP_OK);
    assert(driver.start() == ESP_OK);
    assert(driver.last_error() == ESP_OK);
    link.end(); clean();
    // Two distinct links are legal; duplicate GPIO and attachment while running are not.
    Link second(driver,GPIO_NUM_3);
    assert(link.begin() == ESP_OK && second.begin() == ESP_OK);
    assert(driver.start() == ESP_OK);
    Link late(driver,GPIO_NUM_4);
    assert(late.begin() == ESP_ERR_INVALID_STATE);
    second.end(); link.end(); clean();
}

void run_learning_checks();

int main() {
    run_learning_checks();
    test_codec(); std::puts("PASS: packet, bit order, all bitrates, timing and symbol validation");
    test_config_and_transport(); std::puts("PASS: configuration, partial setup rollback, timeout abort and close");
    test_lifecycle(); std::puts("PASS: restart, duplicate GPIO, task/TX failures, buffer lifetime and stop handshake");
    clean();
}
