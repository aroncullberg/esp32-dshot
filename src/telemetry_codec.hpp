#pragma once
#include <cstddef>
#include <cstdint>
#include "hal/rmt_types.h"
#include "dshot/telemetry.hpp"
#include "frame_codec.hpp"

namespace dshot::detail {
using ReplyResult = ReceiveResult;
struct DecodedReply {
    ReplyResult result = ReplyResult::NoResponse;
    std::uint16_t payload = 0;
};
// Physical (non-inverted) RX levels at 5/4 of the outgoing DShot bitrate.
DecodedReply decode_reply(const rmt_symbol_word_t* symbols, std::size_t count,
                          std::uint32_t resolution_hz, std::uint32_t bitrate);
DecodedReply decode_gcr(std::uint32_t gcr);
// RX starts before TX. Validate and remove our inverted packet in place,
// leaving only the ESC reply (or an empty/idle capture when it is silent).
bool strip_transmit_echo(rmt_symbol_word_t* symbols, std::size_t& count,
                         const SymbolFrame& transmitted);
}
