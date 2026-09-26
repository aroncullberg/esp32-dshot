#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "frame_codec.hpp"

namespace dshot::detail {
inline constexpr std::uint32_t kRMTResolutionHz = 40'000'000;

// Task-context operations on this state must be serialized. Close before destroy.
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
};

esp_err_t open_transport(RmtTransport& transport, gpio_num_t gpio, const Config& config);
esp_err_t send_packet(RmtTransport& transport, std::uint16_t packet);
esp_err_t close_transport(RmtTransport& transport);
// Never allow storage to be freed if hardware ownership could not be retired.
void close_transport_or_abort(RmtTransport& transport);
}
