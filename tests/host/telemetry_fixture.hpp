#pragma once
#include <vector>
#include <cstdint>
#include "hal/rmt_types.h"
#include "frame_codec.hpp"

inline std::vector<rmt_symbol_word_t> esc_reply(std::uint16_t payload, unsigned bitrate = 600000,
                                               bool bad_crc = false, int jitter = 0,
                                               bool zero_end_marker = true,
                                               bool leading_idle = true) {
    const unsigned crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 15;
    const unsigned word = (payload << 4) | (crc ^ (bad_crc ? 1 : 0));
    const unsigned alphabet[16] = {25,27,18,19,29,21,22,23,26,9,10,11,30,13,14,15};
    std::vector<unsigned> bits{0};
    for (int nibble = 3; nibble >= 0; --nibble) {
        const auto code = alphabet[(word >> (nibble*4)) & 15];
        for (int bit = 4; bit >= 0; --bit) bits.push_back(bits.back() ^ ((code >> bit)&1));
    }
    struct Run { unsigned level, ticks; };
    std::vector<Run> runs{{1,1200}}; // Include the idle interval before the reply.
    for (unsigned start = 0; start < bits.size();) {
        unsigned end = start + 1;
        while (end < bits.size() && bits[end] == bits[start]) ++end;
        const auto ticks = ((end-start)*160000000ull + bitrate*5/2) / (bitrate*5);
        runs.push_back({bits[start],static_cast<unsigned>(static_cast<int>(ticks)+jitter)});
        start = end;
    }
    if (runs.back().level == 1) runs.back().ticks += 1600;
    else runs.push_back({1,1600});
    // Model an idle timeout that replaces the final high interval with an end marker.
    if (zero_end_marker) runs.back().ticks = 0;
    if (!leading_idle) runs.erase(runs.begin());
    std::vector<rmt_symbol_word_t> symbols((runs.size()+1)/2);
    for (unsigned i=0; i<runs.size(); ++i) {
        if (i%2) { symbols[i/2].level1=runs[i].level; symbols[i/2].duration1=runs[i].ticks; }
        else { symbols[i/2].level0=runs[i].level; symbols[i/2].duration0=runs[i].ticks; }
    }
    return symbols;
}

inline std::vector<rmt_symbol_word_t> exchange_capture(
    const dshot::detail::SymbolFrame& frame, const std::vector<rmt_symbol_word_t>& reply,
    bool leading_idle = false, bool zero_end_marker = true, int echo_jitter = 0) {
    struct Run { unsigned level, ticks; };
    std::vector<Run> runs;
    if (leading_idle) runs.push_back({1, 200});
    for (const auto& symbol : frame) {
        runs.push_back({0, static_cast<unsigned>(static_cast<int>(symbol.duration0) + echo_jitter)});
        runs.push_back({1, static_cast<unsigned>(static_cast<int>(symbol.duration1) - echo_jitter)});
    }
    if (reply.empty()) {
        runs.back().ticks = zero_end_marker ? 0 : 1600;
    } else {
        // The turnaround gap joins the high half of the final outgoing bit.
        for (const auto& symbol : reply) {
            for (unsigned half = 0; half < 2; ++half) {
                const unsigned level = half ? symbol.level1 : symbol.level0;
                const unsigned ticks = half ? symbol.duration1 : symbol.duration0;
                if (ticks && runs.back().ticks && runs.back().level == level) runs.back().ticks += ticks;
                else runs.push_back({level, ticks});
            }
        }
    }
    std::vector<rmt_symbol_word_t> symbols((runs.size() + 1) / 2);
    for (unsigned i = 0; i < runs.size(); ++i) {
        if (i % 2) { symbols[i / 2].level1 = runs[i].level; symbols[i / 2].duration1 = runs[i].ticks; }
        else { symbols[i / 2].level0 = runs[i].level; symbols[i / 2].duration0 = runs[i].ticks; }
    }
    return symbols;
}
