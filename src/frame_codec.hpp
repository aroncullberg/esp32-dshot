#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#include "esp_err.h"
#include "driver/rmt_tx.h"

#include "dshot/config.hpp"
#include "hal/rmt_types.h"

namespace dshot::detail {

    esp_err_t encode_regular_packet(
        std::uint16_t payload,
        bool request_telemetry,
        std::uint16_t& packet_out);

    bool packet_bit(std::uint16_t packet, std::size_t wire_index);

    struct BitTiming {
        std::uint32_t bit_total_ticks;
        std::uint32_t bit_0_high_ticks;
        std::uint32_t bit_0_low_ticks;
        std::uint32_t bit_1_high_ticks;
        std::uint32_t bit_1_low_ticks;
    };

    using SymbolFrame = std::array<rmt_symbol_word_t, 16>;

    esp_err_t calculate_bit_timing(Protocol protocol,
                                   std::uint32_t resolution_hz,
                                   BitTiming& timing_out);

    esp_err_t encode_regular_symbols(std::uint16_t packet, const BitTiming& timing, SymbolFrame& symbols_out);
}
