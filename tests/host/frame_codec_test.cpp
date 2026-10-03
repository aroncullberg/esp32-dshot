#include <cassert>
#include <initializer_list>
#include "dshot/config.hpp"
#include "frame_codec.hpp"

using namespace dshot;
using namespace dshot::detail;

void run_frame_codec_tests() {
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
    unsigned total_ticks = 0;
    for (const auto& symbol : symbols) total_ticks += symbol.duration0 + symbol.duration1;
    assert(total_ticks == 1072);
    for (auto& symbol : symbols) {
        symbol.level0 = 0; symbol.duration0 = 7;
        symbol.level1 = 1; symbol.duration1 = 11;
    }
    const auto prior = symbols;
    for (unsigned failure = 0; failure < 4; ++failure) {
        auto invalid = timing;
        if (failure == 0) {
            invalid.bit_0_high_ticks = 0;
            invalid.bit_0_low_ticks = 67;
        }
        if (failure == 1) {
            invalid.bit_total_ticks = 32769;
            invalid.bit_0_high_ticks = 32768;
            invalid.bit_0_low_ticks = 1;
            invalid.bit_1_high_ticks = 16384;
            invalid.bit_1_low_ticks = 16385;
        }
        if (failure == 2) invalid.bit_1_high_ticks = UINT32_MAX;
        if (failure == 3) invalid.bit_total_ticks = 68;
        assert(encode_regular_symbols(0x0606,invalid,symbols) == ESP_ERR_INVALID_ARG);
        for (unsigned i = 0; i < 16; ++i) assert(symbols[i].val == prior[i].val);
    }
}

