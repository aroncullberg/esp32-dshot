#include "idf_transport.hpp"
#include "config_validation.hpp"
#include "stage_profiler.hpp"

#include <cstdlib>
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "freertos/task.h"

namespace dshot::detail {
namespace {
constexpr const char* kTag = "dshot_rmt";
constexpr int kTxTimeoutMs = 20;

void require_disabled(esp_err_t error) {
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "RMT cleanup failed: %s", esp_err_to_name(error));
        std::abort();
    }
}

bool IRAM_ATTR received(rmt_channel_handle_t, const rmt_rx_done_event_data_t* event, void* context) {
    auto& t = *static_cast<RmtTransport*>(context);
    BaseType_t wake = pdFALSE;
    portENTER_CRITICAL_ISR(&t.rx_lock);
    if (t.accept_rx) {
        t.rx_count = event->num_symbols;
        t.accept_rx = false;
        xSemaphoreGiveFromISR(t.rx_done, &wake);
    }
    portEXIT_CRITICAL_ISR(&t.rx_lock);
    return wake == pdTRUE;
}

bool IRAM_ATTR transmitted(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void* context) {
    auto& t = *static_cast<RmtTransport*>(context);
    // RMT keeps the pin driven high after transmission(?)
    gpio_ll_output_disable(&GPIO, t.gpio);
    return false;
}

void retire_receive(RmtTransport& t) {
    portENTER_CRITICAL(&t.rx_lock);
    t.accept_rx = false;
    portEXIT_CRITICAL(&t.rx_lock);
    if (t.rx_enabled) {
        require_disabled(rmt_disable(t.rx_channel));
        t.rx_enabled = false;
    }
}
}

esp_err_t open_transport(RmtTransport& transport, gpio_num_t gpio, const Config& config) {
    if (transport.opened || transport.tx_channel || transport.copy_encoder ||
        transport.gpio != GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t error = validate_config(config);
    if (error != ESP_OK) return error;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) return ESP_ERR_INVALID_ARG;

    BitTiming timing{};
    error = calculate_bit_timing(config.protocol, kRMTResolutionHz, timing);
    if (error != ESP_OK) return error;

    if (config.startup_low_ms) {
        // AM32's bootloader can stay in serial mode if the signal starts high.
        // Configure GPIO before allocating RMT: gpio_set_direction() otherwise
        // disconnects the RMT output route in IDF 6.1.
        error = gpio_set_level(gpio, 0);
        if (error == ESP_OK) error = gpio_set_direction(gpio, GPIO_MODE_OUTPUT_OD);
        if (error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(config.startup_low_ms) + 1);
        require_disabled(gpio_set_direction(gpio, GPIO_MODE_DISABLE));
        if (error != ESP_OK) return error;
    }

    rmt_tx_channel_config_t channel_config{};
    channel_config.gpio_num = gpio;
    channel_config.clk_src = RMT_CLK_SRC_XTAL;
    channel_config.resolution_hz = kRMTResolutionHz;
    channel_config.mem_block_symbols = 48;
    channel_config.trans_queue_depth = 1;
    channel_config.intr_priority = 0;
    transport.bidirectional = config.signaling == Signaling::Bidirectional;
    transport.bitrate = static_cast<unsigned>(config.protocol);
    channel_config.flags.init_level = transport.bidirectional ? 1 : 0;
    channel_config.flags.invert_out = false;
    channel_config.flags.with_dma = false;

    error = rmt_new_tx_channel(&channel_config, &transport.tx_channel);
    if (error != ESP_OK) return error;
    transport.gpio = gpio;
    transport.timing = timing;

    rmt_copy_encoder_config_t encoder_config{};
    error = rmt_new_copy_encoder(&encoder_config, &transport.copy_encoder);
    if (error == ESP_OK) error = rmt_enable(transport.tx_channel);
    if (error != ESP_OK) {
        close_transport_or_abort(transport);
        return error;
    }
    transport.enabled = true;
    if (transport.bidirectional) {
        transport.rx_done = xSemaphoreCreateBinary();
        if (!transport.rx_done) error = ESP_ERR_NO_MEM;
        rmt_rx_channel_config_t rx_config{};
        rx_config.gpio_num = gpio;
        rx_config.clk_src = RMT_CLK_SRC_XTAL;
        rx_config.resolution_hz = kRMTResolutionHz;
        rx_config.mem_block_symbols = 48;
        if (error == ESP_OK) error = rmt_new_rx_channel(&rx_config, &transport.rx_channel);
        rmt_rx_event_callbacks_t rx_callbacks{};
        rx_callbacks.on_recv_done = received;
        if (error == ESP_OK) error = rmt_rx_register_event_callbacks(transport.rx_channel, &rx_callbacks, &transport);
        rmt_tx_event_callbacks_t tx_callbacks{};
        tx_callbacks.on_trans_done = transmitted;
        if (error == ESP_OK) error = rmt_tx_register_event_callbacks(transport.tx_channel, &tx_callbacks, &transport);
        if (error == ESP_OK) error = gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
        if (error == ESP_OK) error = rmt_enable(transport.rx_channel);
        if (error != ESP_OK) {
            close_transport_or_abort(transport);
            return error;
        }
        transport.rx_enabled = true;
        // Use GPIO-controlled output enable to release the pin between packets.
        // gpio_output_enable() disconnects the RMT route in IDF 6.1.
        gpio_ll_set_output_enable_ctrl(&GPIO, gpio, false, false);
        transport.receive_config.signal_range_min_ns = 100;
        transport.receive_config.signal_range_max_ns = 40000;
    }
    transport.opened = true;
    transport.faulted = false;
    return ESP_OK;
}

esp_err_t start_packet(RmtTransport& transport, std::uint16_t packet, StageProfiler* profile) {
    profile_stage(profile, ProfileStage::SymbolEncode);
    if (!transport.opened || !transport.enabled || transport.faulted || transport.in_flight) {
        return ESP_ERR_INVALID_STATE;
    }
    transport.rx_wait_us = 0;
    transport.rx_timed_out = false;
    transport.reply = {};
    // Bidirectional DShot uses a complemented checksum.
    if (transport.bidirectional) packet ^= 0xF;
    esp_err_t error = encode_regular_symbols(packet, transport.timing, transport.symbols);
    if (error != ESP_OK) return error;
    if (transport.bidirectional)
        for (auto& symbol : transport.symbols) { symbol.level0 = 0; symbol.level1 = 1; }
    if (transport.bidirectional) {
        profile_stage(profile, ProfileStage::RxSetup);
        if (!transport.rx_enabled) {
            error = rmt_enable(transport.rx_channel);
            if (error != ESP_OK) { transport.faulted = true; return error; }
            transport.rx_enabled = true;
        }
        (void)xSemaphoreTake(transport.rx_done, 0);
        portENTER_CRITICAL(&transport.rx_lock);
        transport.rx_count = 0;
        transport.rx_error = ESP_OK;
        transport.accept_rx = true;
        portEXIT_CRITICAL(&transport.rx_lock);
        gpio_ll_output_enable(&GPIO, transport.gpio);
        // Capture our TX edges as well as the reply. A silent ESC then reaches
        // the hardware idle limit instead of waiting for the task timeout.
        // RX is also ready before the ESC can start its turnaround response.
        transport.rx_started_at = xTaskGetTickCount();
        const auto rx_error = rmt_receive(transport.rx_channel, transport.rx_symbols,
                                          sizeof(transport.rx_symbols), &transport.receive_config);
        if (rx_error != ESP_OK) {
            portENTER_CRITICAL(&transport.rx_lock);
            transport.rx_error = rx_error;
            transport.accept_rx = false;
            portEXIT_CRITICAL(&transport.rx_lock);
            xSemaphoreGive(transport.rx_done);
        }
    }

    profile_stage(profile, ProfileStage::TxSubmit);
    rmt_transmit_config_t transmit_config{};
    transmit_config.loop_count = 0;
    transmit_config.flags.eot_level = transport.bidirectional ? 1 : 0;
    transmit_config.flags.queue_nonblocking = 1;
    error = rmt_transmit(transport.tx_channel, transport.copy_encoder,
                         transport.symbols.data(),
                         transport.symbols.size() * sizeof(rmt_symbol_word_t),
                         &transmit_config);
    if (error != ESP_OK) {
        transport.faulted = true;
        if (transport.bidirectional) {
            retire_receive(transport);
            gpio_ll_output_disable(&GPIO, transport.gpio);
        }
        return error;
    }
    transport.in_flight = true;
    return ESP_OK;
}

esp_err_t finish_packet(RmtTransport& transport, std::uint64_t* wait_us, StageProfiler* profile) {
    if (wait_us) *wait_us = 0;
    if (!transport.in_flight) return ESP_ERR_INVALID_STATE;
    bool done = false;
    esp_err_t rx_error = ESP_OK;
    std::size_t count = 0;
    if (transport.bidirectional) {
        const auto rx_started = esp_timer_get_time();
        profile_stage(profile, ProfileStage::RxWait);
        // All motors share the time since their submissions, rather than each
        // receiving a fresh full timeout when their results are collected.
        const TickType_t timeout = pdMS_TO_TICKS(2) + 1;
        const TickType_t elapsed = xTaskGetTickCount() - transport.rx_started_at;
        done = xSemaphoreTake(transport.rx_done, elapsed >= timeout ? 0 : timeout - elapsed) == pdTRUE;
        profile_stage(profile, ProfileStage::RxFinish);
        transport.rx_wait_us = esp_timer_get_time() - rx_started;
        transport.rx_timed_out = !done;
        portENTER_CRITICAL(&transport.rx_lock);
        transport.accept_rx = false;
        rx_error = transport.rx_error;
        count = transport.rx_count;
        portEXIT_CRITICAL(&transport.rx_lock);
    }

    // RX normally completes after the TX-done ISR has released the pin and
    // queued TX completion. Reap that completion here without a separate sleep
    // for TX. Keep a bounded TX wait for early/failed RX and fault recovery.
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    const auto wait_started = esp_timer_get_time();
#endif
    profile_stage(profile, ProfileStage::TxWait);
    const auto error = rmt_tx_wait_all_done(transport.tx_channel, kTxTimeoutMs);
    profile_stage(profile, ProfileStage::LinkFinish);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    if (wait_us) *wait_us = esp_timer_get_time() - wait_started;
#endif
    if (error != ESP_OK) {
        transport.faulted = true;
        if (transport.bidirectional) retire_receive(transport);
        require_disabled(rmt_disable(transport.tx_channel));
        transport.enabled = false;
    }
    transport.in_flight = false;
    if (transport.bidirectional && error == ESP_OK) {
        if (!done || rx_error != ESP_OK) {
            retire_receive(transport);
            transport.reply.result = done ? ReplyResult::ReceiveError : ReplyResult::NoResponse;
        } else if (count >= 32) {
            transport.reply.result = ReplyResult::Overflow;
        } else {
            profile_stage(profile, ProfileStage::EchoStrip);
            if (!strip_transmit_echo(transport.rx_symbols, transport.rx_count, transport.symbols)) {
                transport.reply.result = ReplyResult::Malformed;
            } else {
                profile_stage(profile, ProfileStage::ReplyDecode);
                transport.reply = decode_reply(transport.rx_symbols,transport.rx_count,kRMTResolutionHz,transport.bitrate);
            }
        }
        profile_stage(profile, ProfileStage::LinkFinish);
    }
    if (transport.bidirectional) gpio_ll_output_disable(&GPIO, transport.gpio);
    return error;
}

esp_err_t send_packet(RmtTransport& transport, std::uint16_t packet,
                      std::uint64_t* wait_us, StageProfiler* profile) {
    if (wait_us) *wait_us = 0;
    const auto error = start_packet(transport, packet, profile);
    return error == ESP_OK ? finish_packet(transport, wait_us, profile) : error;
}

esp_err_t close_transport(RmtTransport& transport) {
    esp_err_t error;
    portENTER_CRITICAL(&transport.rx_lock);
    transport.accept_rx = false;
    portEXIT_CRITICAL(&transport.rx_lock);
    if (transport.enabled) {
        error = rmt_disable(transport.tx_channel);
        if (error != ESP_OK) return error;
        transport.enabled = false;
        transport.in_flight = false;
    }
    if (transport.in_flight) return ESP_ERR_INVALID_STATE;
    if (transport.tx_channel) {
        error = rmt_del_channel(transport.tx_channel);
        if (error != ESP_OK) return error;
        transport.tx_channel = nullptr;
    }
    if (transport.rx_enabled) {
        error = rmt_disable(transport.rx_channel);
        if (error != ESP_OK) return error;
        transport.rx_enabled = false;
    }
    if (transport.rx_channel) {
        error = rmt_del_channel(transport.rx_channel);
        if (error != ESP_OK) return error;
        transport.rx_channel = nullptr;
    }
    if (transport.rx_done) {
        vSemaphoreDelete(transport.rx_done);
        transport.rx_done = nullptr;
    }
    if (transport.copy_encoder) {
        error = rmt_del_encoder(transport.copy_encoder);
        if (error != ESP_OK) return error;
        transport.copy_encoder = nullptr;
    }
    if (transport.gpio != GPIO_NUM_NC) {
        if (transport.bidirectional) {
            error = gpio_set_pull_mode(transport.gpio, GPIO_FLOATING);
            if (error != ESP_OK) return error;
        }
        error = gpio_set_direction(transport.gpio, GPIO_MODE_DISABLE);
        if (error != ESP_OK) return error;
        transport.gpio = GPIO_NUM_NC;
    }
    transport.opened = false;
    transport.faulted = false;
    return ESP_OK;
}

void close_transport_or_abort(RmtTransport& transport) {
    require_disabled(close_transport(transport));
}
}
