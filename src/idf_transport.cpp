#include "idf_transport.hpp"
#include "config_validation.hpp"

#include <cstdlib>
#include "esp_log.h"

namespace dshot::detail {
namespace {
constexpr const char* kTag = "dshot_rmt";
constexpr int kTxTimeoutMs = 20;

void require_disabled(esp_err_t error) {
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "RMT cleanup failed: %s", esp_err_to_name(error));
        std::abort(); // Unlike ESP_ERROR_CHECK, this is active with assertions disabled.
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

    rmt_tx_channel_config_t channel_config{};
    channel_config.gpio_num = gpio;
    channel_config.clk_src = RMT_CLK_SRC_XTAL;
    channel_config.resolution_hz = kRMTResolutionHz;
    channel_config.mem_block_symbols = 48;
    channel_config.trans_queue_depth = 1;
    channel_config.intr_priority = 0;
    channel_config.flags.init_level = 0;
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
    transport.opened = true;
    transport.faulted = false;
    return ESP_OK;
}

esp_err_t send_packet(RmtTransport& transport, std::uint16_t packet) {
    if (!transport.opened || !transport.enabled || transport.faulted || transport.in_flight) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t error = encode_regular_symbols(packet, transport.timing, transport.symbols);
    if (error != ESP_OK) return error;

    rmt_transmit_config_t transmit_config{};
    transmit_config.loop_count = 0;
    transmit_config.flags.eot_level = 0;
    transmit_config.flags.queue_nonblocking = 1;
    error = rmt_transmit(transport.tx_channel, transport.copy_encoder,
                         transport.symbols.data(),
                         transport.symbols.size() * sizeof(rmt_symbol_word_t),
                         &transmit_config);
    if (error != ESP_OK) {
        transport.faulted = true;
        return error;
    }
    transport.in_flight = true;
    error = rmt_tx_wait_all_done(transport.tx_channel, kTxTimeoutMs);
    if (error != ESP_OK) {
        transport.faulted = true;
        require_disabled(rmt_disable(transport.tx_channel));
        transport.enabled = false;
    }
    transport.in_flight = false;
    return error;
}

esp_err_t close_transport(RmtTransport& transport) {
    esp_err_t error;
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
    if (transport.copy_encoder) {
        error = rmt_del_encoder(transport.copy_encoder);
        if (error != ESP_OK) return error;
        transport.copy_encoder = nullptr;
    }
    if (transport.gpio != GPIO_NUM_NC) {
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
