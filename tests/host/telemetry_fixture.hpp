#pragma once
#include <vector>
#include <cstdint>
#include "hal/rmt_types.h"

inline std::vector<rmt_symbol_word_t> esc_reply(std::uint16_t payload, unsigned bitrate = 600000,
                                               bool bad_crc = false, int jitter = 0,
                                               bool zero_end_marker = true) {
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
    std::vector<rmt_symbol_word_t> symbols((runs.size()+1)/2);
    for (unsigned i=0; i<runs.size(); ++i) {
        if (i%2) { symbols[i/2].level1=runs[i].level; symbols[i/2].duration1=runs[i].ticks; }
        else { symbols[i/2].level0=runs[i].level; symbols[i/2].duration0=runs[i].ticks; }
    }
    return symbols;
}
