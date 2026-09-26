#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "dshot/config.hpp"
#include "config_validation.hpp"
#include "frame_codec.hpp"
#include "idf_transport.hpp"

namespace {
void check_regular_packets() {
    struct PacketCase {
        std::uint16_t payload;
        bool request_telemetry;
        std::uint16_t expected_packet;
    };

    // Independent expected values from tutorial step 19.
    constexpr PacketCase cases[] = {
        {0, false, 0x0000},
        {48, false, 0x0606},
        {48, true, 0x0617},
        {100, false, 0x0C84},
        {2047, false, 0xFFEE},
    };

    for (const auto& test : cases) {
        std::uint16_t packet = 0x1234;
        const esp_err_t error = dshot::detail::encode_regular_packet(
            test.payload, test.request_telemetry, packet);
        if (error != ESP_OK || packet != test.expected_packet) {
            std::printf("Packet check FAILED: payload=%u request=%u "
                        "error=%d packet=0x%04X expected=0x%04X\n",
                        static_cast<unsigned>(test.payload),
                        static_cast<unsigned>(test.request_telemetry),
                        error, static_cast<unsigned>(packet),
                        static_cast<unsigned>(test.expected_packet));
            std::abort();
        }
    }

    constexpr std::uint16_t invalid_payloads[] = {2048, 65535};
    for (const auto payload : invalid_payloads) {
        std::uint16_t packet = 0x1234;
        const esp_err_t error = dshot::detail::encode_regular_packet(
            payload, false, packet);
        if (error != ESP_ERR_INVALID_ARG || packet != 0x1234) {
            std::printf("Rejection check FAILED: payload=%u "
                        "error=%d expected_error=%d "
                        "packet=0x%04X expected_unchanged=0x1234\n",
                        static_cast<unsigned>(payload), error,
                        ESP_ERR_INVALID_ARG, static_cast<unsigned>(packet));
            std::abort();
        }
    }

    std::puts("Packet checks passed: 5 known packets and 2 rejected inputs.");
}

void check_packet_bit_order() {
    constexpr char expected[] = "0000011000000110";
    char actual[17]{}; // Sixteen bits plus the terminating null character.
    bool matches = true;

    for (std::size_t wire_index = 0; wire_index < 16; ++wire_index) {
        actual[wire_index] = dshot::detail::packet_bit(0x0606, wire_index)
            ? '1' : '0';
        if (actual[wire_index] != expected[wire_index]) {
            matches = false;
        }
    }

    std::printf("Packet 0x0606, MSB first: %s\n", actual);
    if (!matches) {
        std::printf("Bit-order check FAILED: expected %s\n", expected);
        std::fflush(stdout);
        std::abort();
    }
    std::puts("Bit-order check passed: all 16 bits match.");
}

void check_regular_symbols() {
    using namespace dshot::detail;

    BitTiming timing{};
    const esp_err_t timing_error = calculate_bit_timing(
        dshot::Protocol::DShot600, kRMTResolutionHz, timing);
    if (timing_error != ESP_OK || timing.bit_total_ticks != 67 ||
        timing.bit_0_high_ticks != 25 || timing.bit_0_low_ticks != 42 ||
        timing.bit_1_high_ticks != 50 || timing.bit_1_low_ticks != 17) {
        std::printf("Symbol check FAILED: expected D600/40 MHz timing "
                    "67, 25/42, 50/17; got error=%d %u, %u/%u, %u/%u\n",
                    timing_error, static_cast<unsigned>(timing.bit_total_ticks),
                    static_cast<unsigned>(timing.bit_0_high_ticks),
                    static_cast<unsigned>(timing.bit_0_low_ticks),
                    static_cast<unsigned>(timing.bit_1_high_ticks),
                    static_cast<unsigned>(timing.bit_1_low_ticks));
        std::fflush(stdout);
        std::abort();
    }

    SymbolFrame symbols{};
    const esp_err_t error = encode_regular_symbols(0x0606, timing, symbols);
    if (error != ESP_OK || symbols.size() != 16) {
        std::printf("Symbol check FAILED: error=%d, count=%u (expected 16)\n",
                    error, static_cast<unsigned>(symbols.size()));
        std::fflush(stdout);
        std::abort();
    }

    // Independent expected bits: don't reuse packet_bit to check its own output.
    constexpr char expected_bits[] = "0000011000000110";
    unsigned total_ticks = 0;
    for (std::size_t i = 0; i < symbols.size(); ++i) {
        const unsigned expected_high = expected_bits[i] == '1' ? 50 : 25;
        const unsigned expected_low = expected_bits[i] == '1' ? 17 : 42;
        const auto& symbol = symbols[i];
        if (symbol.level0 != 1 || symbol.level1 != 0 ||
            symbol.duration0 != expected_high || symbol.duration1 != expected_low) {
            std::printf("Symbol check FAILED at wire index %u: "
                        "got level/duration %u/%u then %u/%u; "
                        "expected 1/%u then 0/%u\n",
                        static_cast<unsigned>(i),
                        static_cast<unsigned>(symbol.level0),
                        static_cast<unsigned>(symbol.duration0),
                        static_cast<unsigned>(symbol.level1),
                        static_cast<unsigned>(symbol.duration1),
                        expected_high, expected_low);
            std::fflush(stdout);
            std::abort();
        }
        total_ticks += symbol.duration0 + symbol.duration1;
    }
    if (total_ticks != 1072) {
        std::printf("Symbol check FAILED: frame=%u ticks, expected 1072\n", total_ticks);
        std::fflush(stdout);
        std::abort();
    }
    std::puts("Valid symbols passed: 16 high/low pairs, 1072 ticks (26.8 us).");

    // Each invalid input must be rejected before changing any output symbol.
    const char* invalid_names[] = {"zero duration", "duration above 32767",
                                   "inconsistent bit total"};
    bool rejected_all = true;
    for (unsigned test = 0; test < 3; ++test) {
        BitTiming invalid = timing;
        if (test == 0) {
            invalid.bit_0_high_ticks = 0;
            invalid.bit_0_low_ticks = 67; // Keep the total valid; isolate zero duration.
        } else if (test == 1) {
            invalid.bit_total_ticks = 32769;
            invalid.bit_0_high_ticks = 32768;
            invalid.bit_0_low_ticks = 1;
            invalid.bit_1_high_ticks = 16384;
            invalid.bit_1_low_ticks = 16385;
        } else {
            invalid.bit_total_ticks = 68; // Both duration pairs still sum to 67.
        }

        // A recognizable value in every slot reveals even partial output writes.
        SymbolFrame untouched{};
        for (auto& symbol : untouched) {
            symbol.level0 = 0;
            symbol.duration0 = 7;
            symbol.level1 = 1;
            symbol.duration1 = 11;
        }
        const SymbolFrame before = untouched;
        const esp_err_t invalid_error = encode_regular_symbols(0x0606, invalid, untouched);
        bool unchanged = true;
        for (std::size_t i = 0; i < untouched.size(); ++i) {
            if (untouched[i].level0 != before[i].level0 ||
                untouched[i].duration0 != before[i].duration0 ||
                untouched[i].level1 != before[i].level1 ||
                untouched[i].duration1 != before[i].duration1) {
                unchanged = false;
            }
        }
        if (invalid_error != ESP_ERR_INVALID_ARG || !unchanged) {
            std::printf("Invalid-timing check FAILED (%s): error=%d "
                        "expected=%d, output=%s (expected unchanged)\n",
                        invalid_names[test], invalid_error, ESP_ERR_INVALID_ARG,
                        unchanged ? "unchanged" : "modified");
            rejected_all = false;
        }
    }
    if (!rejected_all) {
        std::fflush(stdout);
        std::abort();
    }
    std::puts("Symbol checks passed, including invalid timing and unchanged output.");
}

void dump(const char* label, const dshot::Config& config) {
    std::printf("%s: bitrate=%u signaling=%s frame_hz=%u\n",
                label,
                static_cast<unsigned>(config.protocol),
                static_cast<unsigned>(config.signaling) == 0 ? "Regular" : "Bidirectional",
                config.frame_hz);
}
}

void run_learning_checks() {
    dshot::Config first;
    dshot::Config second;

    first.frame_hz = 500;

    dump("first ", first);   // frame_hz=500
    dump("second", second);  // frame_hz=1000


    std::printf("validation result: %d\n", dshot::validate_config(first));

    std::puts("Configuration values displayed; codec checks follow.");

    check_regular_packets();
    check_packet_bit_order();
    check_regular_symbols();
}
