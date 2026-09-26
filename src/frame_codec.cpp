
#include <cstddef>
#include <cstdint>
#include "frame_codec.hpp"
#include "protocol.hpp"
#include "esp_err.h"

namespace dshot::detail {

    // payload is 11 bits
    // request_telemetry is 1 bit
    // body is 12 bits
    // packet_out is 16 bits
    // payload is (i think)  [5 bits zeros][11 bits payload]
    esp_err_t encode_regular_packet(std::uint16_t payload,
                                    bool request_telemetry,
                                    std::uint16_t& packet_out) {
        // 2047 = 0x7FF (11 bits)
        if (payload > 2047) {
            return ESP_ERR_INVALID_ARG;
        }

        // body is (i think) [4 bits zeros][11 bits payload][1 bit telemetry]
        std::uint16_t body = (payload << 1) | (request_telemetry ? 1 : 0);

        std::uint16_t checksum = 0;
        for (int i = 0; i < 16; i=i+4) {
            std::uint16_t mask = 0xF << i;
            checksum ^= (body & mask) >> i;
        }
        // checksum [12 bits zeros][4 bits checksum]

        packet_out = (body << 4) | checksum;

        return ESP_OK;
    }

    bool packet_bit(std::uint16_t packet, std::size_t wire_index) {
        if (wire_index >= 16) {
            return false;
        }
        // STUDY: why cant this be == 1 rather than != 0?
        return (packet & (1u << (15 - wire_index))) != 0;
    }

    esp_err_t calculate_bit_timing(Protocol protocol,
                               std::uint32_t resolution_hz,
                               BitTiming& timing_out) {
        if (!is_supported_protocol(protocol) || resolution_hz == 0) {
            return ESP_ERR_INVALID_ARG;
        }

        const std::uint64_t clock_hz = resolution_hz;
        const std::uint64_t bitrate = static_cast<std::uint64_t>(protocol);

        // what the fuck is this syntax? lambda function? but why the []???
        const auto rounded_divide = [](std::uint64_t numerator, std::uint64_t denominator) {
            return (numerator + denominator / 2) / denominator;
        };

        const auto total_ticks = rounded_divide(clock_hz, bitrate);
        const auto zero_high = rounded_divide(clock_hz * 3, bitrate * 8);
        const auto one_high = rounded_divide(clock_hz * 3, bitrate * 4);

        if (zero_high == 0 || one_high == 0 ||
            zero_high >= total_ticks || one_high >= total_ticks) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        const auto zero_low = total_ticks - zero_high;
        const auto one_low = total_ticks - one_high;

        constexpr std::uint64_t max_duration_ticks = 32767;
        if (zero_high > max_duration_ticks || zero_low > max_duration_ticks ||
            one_high > max_duration_ticks || one_low > max_duration_ticks) {
                return ESP_ERR_NOT_SUPPORTED;
            }

        BitTiming result{};
        result.bit_total_ticks = static_cast<std::uint32_t>(total_ticks);
        result.bit_0_high_ticks = static_cast<std::uint32_t>(zero_high);
        result.bit_0_low_ticks = static_cast<std::uint32_t>(zero_low);
        result.bit_1_high_ticks = static_cast<std::uint32_t>(one_high);
        result.bit_1_low_ticks = static_cast<std::uint32_t>(one_low);

        timing_out = result;
        return ESP_OK;
    }

    esp_err_t encode_regular_symbols(std::uint16_t packet, const BitTiming &timing, SymbolFrame &symbols_out) {
        // Validate before writing anything: invalid input must preserve the array.
        constexpr std::uint32_t max_duration_ticks = 32767;
        if (timing.bit_0_high_ticks == 0 || timing.bit_0_low_ticks == 0 ||
            timing.bit_1_high_ticks == 0 || timing.bit_1_low_ticks == 0 ||
            timing.bit_0_high_ticks > max_duration_ticks ||
            timing.bit_0_low_ticks > max_duration_ticks ||
            timing.bit_1_high_ticks > max_duration_ticks ||
            timing.bit_1_low_ticks > max_duration_ticks ||
            timing.bit_0_high_ticks + timing.bit_0_low_ticks != timing.bit_total_ticks ||
            timing.bit_1_high_ticks + timing.bit_1_low_ticks != timing.bit_total_ticks) {
            return ESP_ERR_INVALID_ARG;
        }
        for (std::size_t i = 0; i < 16; ++i) {
            bool bit = packet_bit(packet, i);
            std::uint32_t high_ticks = bit ? timing.bit_1_high_ticks : timing.bit_0_high_ticks;
            std::uint32_t low_ticks = bit ? timing.bit_1_low_ticks : timing.bit_0_low_ticks;

            symbols_out[i].level0 = 1;
            symbols_out[i].duration0 = high_ticks;
            symbols_out[i].level1 = 0;
            symbols_out[i].duration1 = low_ticks;
        }
        return ESP_OK;
    }
}
