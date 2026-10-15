#include "host_support.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>
#include "idf_transport.hpp"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "telemetry_fixture.hpp"
#include "soc/gpio_struct.h"

using namespace std::chrono_literals;
using namespace dshot;
using namespace dshot::detail;
using namespace host_test;

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
    bool rx = false;
    int gpio = 0;
    unsigned idle = 0;
    rmt_tx_done_callback_t tx_callback = nullptr;
    rmt_rx_done_callback_t rx_callback = nullptr;
    void* context = nullptr;
    rmt_symbol_word_t* rx_buffer = nullptr;
};

namespace host_test {
std::atomic<int> channels{0}, encoders{0};
std::atomic<bool> fail_channel{false}, fail_encoder{false}, fail_enable{false};
std::atomic<bool> fail_task{false}, fail_submit{false}, fail_wait{false};
int fail_tx_channel_after = -1;
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
std::array<rmt_channel_handle_t,49> receivers{};
bool silence = false, bad_reply_crc = false, rx_overflow = false, fail_rx_new = false;
bool empty_reply = false;
bool stall_receive = false;
bool fail_receive = false, fail_rx_register = false, fail_tx_register = false, fail_rx_enable = false;
std::uint16_t reply_payload = 0xFFF;
bool fail_startup_level = false, fail_startup_output = false;
unsigned startup_pulses = 0;
TickType_t startup_delay_ticks = 0;
int startup_gpio = -1;


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
    result.startup_low_ms = 0;
    return result;
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
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t semaphore, BaseType_t* wake) {
    *wake = pdTRUE;
    return xSemaphoreGive(semaphore);
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
void vTaskDelay(TickType_t ticks) {
    assert(startup_gpio >= 0 && pad_driven(startup_gpio) && !GPIO.rmt_routed[startup_gpio]);
    ++startup_pulses;
    startup_delay_ticks = ticks;
}
TaskHandle_t xTaskGetCurrentTaskHandle() { return current_task; }
#if CONFIG_DSHOT_STAGE_PROFILING
configRUN_TIME_COUNTER_TYPE ulTaskGetIdleRunTimeCounterForCore(BaseType_t core) {
    assert(core >= 0 && core < configNUMBER_OF_CORES);
    return 0; // Arithmetic/blocked-time accounting is exercised by stage_profiler_test.
}
#endif
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
    std::abort();
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
void host_test::emit_alarm(HostTimer* timer) {
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
    assert(GPIO_IS_VALID_OUTPUT_GPIO(gpio));
    if (mode == GPIO_MODE_OUTPUT_OD) {
        assert(startup_gpio == gpio); // Low latch must be set before enabling output.
        if (fail_startup_output) { fail_startup_output = false; return ESP_FAIL; }
        GPIO.rmt_routed[gpio] = false;
        GPIO.peripheral_oe[gpio] = false;
        GPIO.enabled[gpio] = true;
        return ESP_OK;
    }
    assert(mode == GPIO_MODE_DISABLE);
    GPIO.enabled[gpio] = false;
    GPIO.peripheral_oe[gpio] = false;
    return ESP_OK;
}
esp_err_t gpio_set_level(gpio_num_t gpio, unsigned level) {
    assert(GPIO_IS_VALID_OUTPUT_GPIO(gpio) && level == 0);
    if (fail_startup_level) { fail_startup_level = false; return ESP_FAIL; }
    startup_gpio = gpio;
    return ESP_OK;
}
esp_err_t gpio_set_pull_mode(gpio_num_t, gpio_pull_mode_t) { return ESP_OK; }
esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t* config, rmt_channel_handle_t* out) {
    if (fail_rx_new) { fail_rx_new=false; return ESP_ERR_NO_MEM; }
    assert(config->resolution_hz == 40000000 && !config->flags.invert_in);
    *out = new rmt_channel_t;
    (*out)->rx = true; (*out)->gpio = config->gpio_num;
    receivers[config->gpio_num] = *out;
    ++channels;
    return ESP_OK;
}
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t channel, const rmt_rx_event_callbacks_t* cb, void* ctx) {
    if (fail_rx_register) { fail_rx_register=false; return ESP_FAIL; }
    channel->rx_callback=cb->on_recv_done; channel->context=ctx; return ESP_OK;
}
esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t channel, const rmt_tx_event_callbacks_t* cb, void* ctx) {
    if (fail_tx_register) { fail_tx_register=false; return ESP_FAIL; }
    channel->tx_callback=cb->on_trans_done; channel->context=ctx; return ESP_OK;
}
esp_err_t rmt_receive(rmt_channel_handle_t channel, void* buffer, size_t bytes, const rmt_receive_config_t* config) {
    assert(channel->rx && channel->enabled && !channel->active);
    assert(bytes == 32*sizeof(rmt_symbol_word_t) && config->signal_range_max_ns == 40000);
    assert(pad_driven(channel->gpio)); // RX is armed before TX, while idle-high is driven.
    assert(GPIO.rmt_routed[channel->gpio]);
    if (fail_receive) { fail_receive=false; return ESP_FAIL; }
    channel->active=true;
    channel->rx_buffer=static_cast<rmt_symbol_word_t*>(buffer);
    return ESP_OK;
}
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t* config, rmt_channel_handle_t* out) {
    assert(config->resolution_hz == 40000000 && config->mem_block_symbols == 48);
    assert(config->trans_queue_depth == 1 && !config->flags.invert_out && !config->flags.with_dma);
    if (fail_channel.exchange(false)) return ESP_ERR_NO_MEM;
    if (fail_tx_channel_after == 0) {
        fail_tx_channel_after = -1;
        return ESP_ERR_NO_MEM;
    }
    if (fail_tx_channel_after > 0) --fail_tx_channel_after;
    *out = new rmt_channel_t;
    (*out)->gpio=config->gpio_num; (*out)->idle=config->flags.init_level;
    GPIO.enabled[config->gpio_num] = true;
    GPIO.peripheral_oe[config->gpio_num] = true;
    GPIO.rmt_routed[config->gpio_num] = true;
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
    if (channel->rx && fail_rx_enable) { fail_rx_enable=false; return ESP_FAIL; }
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
    if (channel->rx) receivers[channel->gpio]=nullptr;
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
    assert(config->loop_count == 0 && config->flags.eot_level == channel->idle && config->flags.queue_nonblocking);
    if (channel->idle) {
        assert(pad_driven(channel->gpio));
        assert(GPIO.rmt_routed[channel->gpio]);
    }
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
        assert(symbol.level0 == (channel->idle ? 0u : 1u) && symbol.level1 == channel->idle);
        assert(symbol.duration0 + symbol.duration1 == 67);
        assert(symbol.duration0 == 25 || symbol.duration0 == 50);
        packet = (packet << 1) | (symbol.duration0 == 50);
    }
    wire_packets.push_back(packet);
    SymbolFrame frame{};
    std::copy_n(channel->buffer, frame.size(), frame.begin());
    channel->active = false;
    channel->buffer = nullptr;
    wire_changed.notify_all();
    lock.unlock();
    if (channel->tx_callback) {
        rmt_tx_done_event_data_t event{17};
        channel->tx_callback(channel,&event,channel->context);
        auto* rx = receivers[channel->gpio];
        assert(!pad_driven(channel->gpio)); // TX-done must release the pin for the ESC.
        if (rx && rx->active && !stall_receive) {
            const auto capture=empty_reply ? std::vector<rmt_symbol_word_t>{} :
                exchange_capture(frame, silence ? std::vector<rmt_symbol_word_t>{} :
                    esc_reply(reply_payload,600000,bad_reply_crc));
            std::copy(capture.begin(),capture.end(),rx->rx_buffer);
            rx->active=false;
            rmt_rx_done_event_data_t received{};
            received.received_symbols=rx->rx_buffer;
            received.num_symbols=rx_overflow ? 32 : capture.size();
            rx->rx_callback(rx,&received,rx->context);
        }
    }
    return ESP_OK;
}
}
