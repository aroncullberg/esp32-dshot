#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "frame_codec.hpp"
#include "telemetry_codec.hpp"

namespace dshot::detail {
class StageProfiler;
inline constexpr std::uint32_t kRMTResolutionHz = 40'000'000;

struct RmtTransport {
    RmtTransport() = default;
    RmtTransport(const RmtTransport&) = delete;
    RmtTransport& operator=(const RmtTransport&) = delete;
    RmtTransport(RmtTransport&&) = delete;
    RmtTransport& operator=(RmtTransport&&) = delete;

    rmt_channel_handle_t tx_channel = nullptr;
    rmt_encoder_handle_t copy_encoder = nullptr;
    gpio_num_t gpio = GPIO_NUM_NC;
    bool enabled = false;
    bool opened = false;
    bool in_flight = false;
    bool faulted = false;
    BitTiming timing{};
    SymbolFrame symbols{};
    bool bidirectional = false;
    unsigned bitrate = 0;
    rmt_channel_handle_t rx_channel = nullptr;
    bool rx_enabled = false;
    SemaphoreHandle_t rx_done = nullptr;
    portMUX_TYPE rx_lock = portMUX_INITIALIZER_UNLOCKED;
    // rx_lock protects accept_rx, rx_error, and rx_count.
    bool accept_rx = false;
    esp_err_t rx_error = ESP_OK;
    std::size_t rx_count = 0;
    rmt_symbol_word_t rx_symbols[32]{};
    rmt_receive_config_t receive_config{};
    DecodedReply reply;
    bool rx_timed_out = false;
    TickType_t rx_started_at = 0;
    std::uint64_t rx_wait_us = 0;
};

esp_err_t open_transport(RmtTransport& transport, gpio_num_t gpio, const Config& config);
// The symbol/RX buffers remain owned by this exchange until finish_packet().
esp_err_t start_packet(RmtTransport& transport, std::uint16_t packet,
                       StageProfiler* profile = nullptr);
esp_err_t finish_packet(RmtTransport& transport, std::uint64_t* wait_us = nullptr,
                        StageProfiler* profile = nullptr);
esp_err_t send_packet(RmtTransport& transport, std::uint16_t packet,
                      std::uint64_t* wait_us = nullptr, StageProfiler* profile = nullptr);
esp_err_t close_transport(RmtTransport& transport);
// Failed cleanup may leave callbacks pointing into the transport.
void close_transport_or_abort(RmtTransport& transport);
}
