#include "telemetry_codec.hpp"

namespace dshot::detail {
DecodedReply decode_gcr(std::uint32_t gcr) {
    // Protocol's 4b/5b alphabet, indexed by decoded nibble.
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
    // Reply checksums are complemented.
    if (((word ^ (word >> 4) ^ (word >> 8) ^ (word >> 12)) & 15) != 15)
        return {ReplyResult::ChecksumError,0};
    return {ReplyResult::Valid,static_cast<std::uint16_t>(word >> 4)};
}

DecodedReply decode_reply(const rmt_symbol_word_t* symbols, std::size_t count,
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
        // Some RMT targets include the idle-high interval preceding the first edge.
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
        // The final high run merges with idle.
        const bool idle_tail = level == 1 && measured >= 8 * scale;
        if (idle_tail) {
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
    // RMT idle timeout can replace the final high run with a zero-duration marker.
    if (ended && previous_level == 0 && bits >= 18 && bits < 21) {
        const auto remaining = 21 - bits;
        levels = (levels << remaining) | ((1u << remaining) - 1);
        bits = 21;
    }
    if (bits != 21) return {ReplyResult::Malformed,0};
    return decode_gcr((levels ^ (levels >> 1)) & 0xFFFFF);
}
}
