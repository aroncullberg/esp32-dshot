#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include "telemetry_codec.hpp"
#include "telemetry_fixture.hpp"

using namespace dshot::detail;

namespace {
// Keep the original division and alphabet-search decoder as a reference for
// testing the optimized decoder's acceptance boundaries and error results.
DecodedReply reference_gcr(std::uint32_t gcr) {
    constexpr std::uint8_t alphabet[] = {
        0x19,0x1B,0x12,0x13,0x1D,0x15,0x16,0x17,
        0x1A,0x09,0x0A,0x0B,0x1E,0x0D,0x0E,0x0F
    };
    if (gcr >> 20) return {ReplyResult::Malformed,0};
    std::uint16_t word = 0;
    for (int shift = 15; shift >= 0; shift -= 5) {
        const auto code = (gcr >> shift) & 31;
        unsigned nibble = 0;
        while (nibble < 16 && alphabet[nibble] != code) ++nibble;
        if (nibble == 16) return {ReplyResult::GcrError,0};
        word = (word << 4) | nibble;
    }
    if (((word ^ (word >> 4) ^ (word >> 8) ^ (word >> 12)) & 15) != 15)
        return {ReplyResult::ChecksumError,0};
    return {ReplyResult::Valid,static_cast<std::uint16_t>(word >> 4)};
}

DecodedReply reference_reply(const rmt_symbol_word_t* symbols, std::size_t count,
                             std::uint32_t resolution_hz, std::uint32_t bitrate) {
    if (!count) return {};
    if (!symbols || !resolution_hz || (bitrate != 300000 && bitrate != 600000))
        return {ReplyResult::Malformed,0};
    const std::uint64_t rate = static_cast<std::uint64_t>(bitrate) * 5;
    const std::uint64_t scale = static_cast<std::uint64_t>(resolution_hz) * 4;
    std::uint32_t levels = 0;
    unsigned bits = 0;
    bool started = false, ended = false;
    int previous_level = -1;
    for (std::size_t i = 0; i < count * 2; ++i) {
        const auto& symbol = symbols[i / 2];
        const unsigned ticks = i % 2 ? symbol.duration1 : symbol.duration0;
        const unsigned level = i % 2 ? symbol.level1 : symbol.level0;
        if (!ticks) { ended = true; continue; }
        if (ended) return {ReplyResult::Malformed,0};
        if (!started && level == 1) continue;
        started = true;
        if (bits == 21) {
            if (level != 1) return {ReplyResult::Malformed,0};
            ended = true;
            continue;
        }
        if (previous_level == static_cast<int>(level)) return {ReplyResult::Malformed,0};
        previous_level = level;
        const auto measured = ticks * rate;
        unsigned run = (measured + scale / 2) / scale;
        if (level == 1 && measured >= 8 * scale) {
            if (bits < 18) return {ReplyResult::Malformed,0};
            run = 21 - bits;
        } else {
            const auto ideal = run * scale;
            const auto distance = measured > ideal ? measured - ideal : ideal - measured;
            if (!run || run > 3 || distance * 100 > scale * 35 || bits + run > 21)
                return {ReplyResult::Malformed,0};
        }
        levels <<= run;
        if (level) levels |= (1u << run) - 1;
        bits += run;
    }
    if (!started) return {};
    if (ended && previous_level == 0 && bits >= 18 && bits < 21) {
        const auto remaining = 21 - bits;
        levels = (levels << remaining) | ((1u << remaining) - 1);
        bits = 21;
    }
    if (bits != 21) return {ReplyResult::Malformed,0};
    return reference_gcr((levels ^ (levels >> 1)) & 0xFFFFF);
}

void check_equal(DecodedReply actual, DecodedReply expected) {
    assert(actual.result == expected.result && actual.payload == expected.payload);
}

void check_capture(const std::vector<rmt_symbol_word_t>& capture,
                   std::uint32_t resolution, std::uint32_t bitrate) {
    check_equal(decode_reply(capture.data(), capture.size(), resolution, bitrate),
                reference_reply(capture.data(), capture.size(), resolution, bitrate));
}

void set_duration(rmt_symbol_word_t& symbol, bool second, unsigned ticks) {
    if (second) symbol.duration1 = ticks;
    else symbol.duration0 = ticks;
}

std::vector<rmt_symbol_word_t> scaled_reply(unsigned payload, unsigned bitrate,
                                          unsigned resolution, bool zero_end_marker) {
    auto capture = esc_reply(payload, bitrate, false, 0, zero_end_marker, false);
    for (auto& symbol : capture) {
        for (const bool second : {false, true}) {
            const auto ticks = second ? symbol.duration1 : symbol.duration0;
            const auto scaled = (static_cast<std::uint64_t>(ticks) * resolution + 20000000) / 40000000;
            set_duration(symbol, second, std::min<std::uint64_t>(32767, scaled));
        }
    }
    return capture;
}

void test_gcr_exhaustive() {
    // Every possible four-code GCR word, including invalid codes and CRCs.
    for (std::uint32_t gcr = 0; gcr < (1u << 20); ++gcr)
        check_equal(decode_gcr(gcr), reference_gcr(gcr));
    for (unsigned bit = 20; bit < 32; ++bit)
        check_equal(decode_gcr((1u << bit) | 0x9BFB7), reference_gcr((1u << bit) | 0x9BFB7));
}

void test_all_pulse_widths() {
    // This stopped response has 1-, 2- and 3-cell runs, and a final idle tail.
    // Sweep the entire 15-bit RMT duration range at every pulse position.
    for (const auto bitrate : {300000u, 600000u}) {
        for (const auto resolution : {40000000u, 40000001u, 80000000u}) {
            auto capture = scaled_reply(0xFFF, bitrate, resolution, false);
            assert(decode_reply(capture.data(), capture.size(), resolution, bitrate).result == ReplyResult::Valid);
            for (std::size_t half = 0; half < capture.size() * 2; ++half) {
                const auto saved = capture[half / 2];
                for (unsigned ticks = 0; ticks <= 32767; ++ticks) {
                    set_duration(capture[half / 2], half % 2, ticks);
                    check_capture(capture, resolution, bitrate);
                }
                capture[half / 2] = saved;
            }
        }
    }
}

void test_mutated_captures() {
    constexpr unsigned resolutions[] = {
        1, 1000000, 24000000, 40000000, 40000001, 80000000, 160000000,
        std::numeric_limits<unsigned>::max()
    };
    std::uint32_t seed = 0xD500600;
    const auto random = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return seed;
    };
    for (unsigned sample = 0; sample < 50000; ++sample) {
        const auto bitrate = sample % 2 ? 300000u : 600000u;
        const auto resolution = resolutions[(sample / 2) % 8];
        auto capture = scaled_reply(random() & 0xFFF, bitrate, resolution, sample % 3 == 0);
        check_capture(capture, resolution, bitrate);
        auto& symbol = capture[(random() >> 8) % capture.size()];
        switch (sample % 5) {
            case 0: symbol.level0 ^= 1; break;
            case 1: symbol.level1 ^= 1; break;
            case 2: set_duration(symbol, false, 0); break;
            case 3: set_duration(symbol, true, 0); break;
            case 4: symbol.val = random(); break;
        }
        check_capture(capture, resolution, bitrate);
        capture.resize((random() >> 8) % (capture.size() + 1));
        check_capture(capture, resolution, bitrate);
    }
    for (const auto bitrate : {0u, 300000u, 600000u, 1200000u}) {
        for (const auto resolution : {0u, 40000000u}) {
            for (const auto count : {0u, 1u})
                check_equal(decode_reply(nullptr, count, resolution, bitrate),
                            reference_reply(nullptr, count, resolution, bitrate));
            check_capture(esc_reply(0xFFF), resolution, bitrate);
        }
    }
}
}

int main() {
    test_gcr_exhaustive();
    std::puts("PASS: all 1,048,576 GCR words match the original decoder");
    test_all_pulse_widths();
    std::puts("PASS: all 15-bit pulse widths match at every position for D300/D600 and three RMT clocks");
    test_mutated_captures();
    std::puts("PASS: mutated/truncated captures, alternate clocks and invalid arguments match the original decoder");
}
