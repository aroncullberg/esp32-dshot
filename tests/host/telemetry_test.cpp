#include <cassert>
#include <cstdio>
#include "telemetry_codec.hpp"
#include "telemetry_state.hpp"
#include "telemetry_fixture.hpp"
using namespace dshot;
using namespace dshot::detail;

namespace {
void test_telemetry_codec() {
    assert(decode_gcr(0x9BFB7).payload == 0x3F4);
    assert(decode_gcr(0x9BFB7).result == ReplyResult::Valid);
    assert(decode_gcr(0x7BDF9).payload == 0xFFF);
    assert(decode_gcr(0).result == ReplyResult::GcrError);
    assert(decode_gcr(0x100000).result == ReplyResult::Malformed);
    // DShot300 stopped response from the first falling edge.
    // RMT omits the final high cell at idle timeout.
    rmt_symbol_word_t stopped[8]{};
    const unsigned widths[8][2] = {
        {213,107},{107,107},{213,107},{107,107},
        {213,107},{107,107},{107,107},{320,0}
    };
    for (unsigned i = 0; i < 8; ++i) {
        stopped[i].level0 = 0; stopped[i].duration0 = widths[i][0];
        stopped[i].level1 = 1; stopped[i].duration1 = widths[i][1];
    }
    const auto stopped_reply = decode_reply(stopped,8,40000000,300000);
    assert(stopped_reply.result == ReplyResult::Valid && stopped_reply.payload == 0xFFF);
    for (const auto bitrate : {300000u,600000u}) {
        for (unsigned payload=0; payload<4096; ++payload) {
            for (const int jitter : {-5,0,5}) {
                for (const bool zero_end_marker : {false,true}) {
                    const auto capture = esc_reply(payload,bitrate,false,jitter,zero_end_marker);
                    const auto decoded = decode_reply(capture.data(),capture.size(),40000000,bitrate);
                    assert(decoded.result == ReplyResult::Valid && decoded.payload == payload);
                }
            }
        }
    }
    auto bad = esc_reply(0x3F4,600000,true);
    assert(decode_reply(bad.data(),bad.size(),40000000,600000).result == ReplyResult::ChecksumError);
    bad = esc_reply(0x3F4);
    assert(decode_reply(bad.data(),2,40000000,600000).result == ReplyResult::Malformed);
    bad[1].duration0 = 1;
    assert(decode_reply(bad.data(),bad.size(),40000000,600000).result == ReplyResult::Malformed);
    assert(decode_reply(nullptr,0,40000000,600000).result == ReplyResult::NoResponse);

    // Reject data after the end marker.
    bad = esc_reply(0x3F4);
    rmt_symbol_word_t extra{};
    extra.level0 = 0; extra.duration0 = 53;
    bad.push_back(extra);
    assert(decode_reply(bad.data(),bad.size(),40000000,600000).result == ReplyResult::Malformed);
    // Reject an early end marker.
    bad = esc_reply(0xFFF);
    bad.resize(2);
    bad.back().duration1 = 0;
    assert(decode_reply(bad.data(),bad.size(),40000000,600000).result == ReplyResult::Malformed);

    // Test checksum rejection when RMT omits the final high interval.
    bad = esc_reply(0x3F4,600000,true,0,true);
    assert(decode_reply(bad.data(),bad.size(),40000000,600000).result == ReplyResult::ChecksumError);

}

void test_transmit_echo() {
    std::size_t empty_count = 0;
    assert(strip_transmit_echo(nullptr, empty_count, SymbolFrame{}));
    for (const auto bitrate : {300000u, 600000u}) {
        BitTiming timing{};
        assert(calculate_bit_timing(static_cast<Protocol>(bitrate), 40000000, timing) == ESP_OK);
        for (const auto packet : {0x000Fu, 0xFFFFu, 0xA55Au}) {
            SymbolFrame frame{};
            assert(encode_regular_symbols(packet, timing, frame) == ESP_OK);
            for (auto& symbol : frame) { symbol.level0 = 0; symbol.level1 = 1; }
            for (const bool leading_idle : {false, true}) {
                for (const bool zero_end_marker : {false, true}) {
                    auto capture = exchange_capture(frame, {}, leading_idle, zero_end_marker);
                    auto count = capture.size();
                    assert(strip_transmit_echo(capture.data(), count, frame));
                    assert(decode_reply(capture.data(), count, 40000000, bitrate).result == ReplyResult::NoResponse);
                    for (unsigned payload = 0; payload < 4096; ++payload) {
                        const auto reply = esc_reply(payload, bitrate, false, 0, zero_end_marker);
                        capture = exchange_capture(frame, reply, leading_idle, zero_end_marker, payload % 2 ? -2 : 2);
                        assert(capture.size() < 32); // TX + worst-case reply fits the transport buffer.
                        count = capture.size();
                        assert(strip_transmit_echo(capture.data(), count, frame));
                        const auto decoded = decode_reply(capture.data(), count, 40000000, bitrate);
                        assert(decoded.result == ReplyResult::Valid && decoded.payload == payload);
                    }
                }
            }
            auto corrupt = exchange_capture(frame, esc_reply(0xFFF, bitrate));
            corrupt[3].duration0 = 1;
            auto count = corrupt.size();
            assert(!strip_transmit_echo(corrupt.data(), count, frame));
            corrupt = exchange_capture(frame, esc_reply(0xFFF, bitrate));
            corrupt.resize(8); // Truncated TX must not be decoded as a reply.
            count = corrupt.size();
            assert(!strip_transmit_echo(corrupt.data(), count, frame));
        }
    }
}

void test_telemetry_state() {
    Config config;
    config.motor_poles = 14;
    config.erpm_max_age_us = 10000;
    config.edt_max_age_us = 3000000;
    TelemetryState state;
    assert(state.snapshot(0,config).speed.state == ErpmState::NoData);
    assert(state.snapshot(0,config).speed.rpm == 0);
    state.ingest({ReplyResult::Valid,0x3F4},100);
    assert(state.data.speed.erpm == 60000 && state.data.speed.period_us == 1000);
    auto speed = state.snapshot(100,config).speed;
    assert(speed.erpm == 60000 && speed.rpm == 8571 && speed.motor_poles == 14);
    config.motor_poles = 2;
    assert(state.snapshot(100,config).speed.rpm == 60000);
    config.motor_poles = 12;
    assert(state.snapshot(100,config).speed.rpm == 10000);
    config.motor_poles = 14;
    state.ingest({ReplyResult::ChecksumError,0},200);
    assert(state.data.speed.updated_at_us == 100);
    assert(state.snapshot(10101,config).speed.state == ErpmState::Stale);
    assert(state.snapshot(10101,config).speed.rpm == 8571);
    state.ingest({ReplyResult::Valid,0xFFF},10200);
    assert(state.data.speed.state == ErpmState::Stopped && state.data.speed.erpm == 0);
    assert(state.snapshot(10200,config).speed.rpm == 0);
    assert(state.snapshot(20201,config).speed.state == ErpmState::Stale);
    state.request(true,2,1);
    state.deadline_us = 999999;
    state.ingest({ReplyResult::Valid,0xE00},11000);
    assert(state.data.edt.state == EdtState::Enabled && state.extended);
    assert(state.data.edt.acknowledgement.valid && state.data.edt.acknowledgement.value == 0);
    for (const auto payload : {0x200,0x440,0x603,0x801,0xA02,0xC80,0xEEF})
        state.ingest({ReplyResult::Valid,static_cast<std::uint16_t>(payload)},12000);
    assert(state.data.temperature_c.valid && state.data.temperature_c.value == 0);
    assert(state.data.voltage_mv.value == 16000 && state.data.current_ma.value == 3000);
    assert(state.data.debug1.value == 1 && state.data.debug2.value == 2);
    assert(state.data.stress.value == 128 && !state.data.debug3.valid);
    assert(state.data.status.value == 0xEF);
    assert(state.data.alert_events == 1 && state.data.warning_events == 1 && state.data.error_events == 1);
    assert(state.data.speed.updated_at_us == 10200); // EDT never refreshes speed.
    assert(state.snapshot(22000,config).speed.state == ErpmState::Stale);
    assert(state.snapshot(22000,config).temperature_c.fresh);
    assert(!state.snapshot(3012001,config).temperature_c.fresh);
    state.ingest({ReplyResult::Valid,0xE10},13000);
    assert(state.data.unknown.value == 0xE10); // Preserve the full payload when a reserved status bit is set.
    state.request(false,2,2); state.deadline_us = 999999;
    state.ingest({ReplyResult::Valid,0xEFF},14000);
    assert(!state.extended && state.data.edt.state == EdtState::Disabled);
    state.request(true,1,3); state.deadline_us = 999999;
    state.ingest({ReplyResult::Valid,0xE00},15000);
    state.ingest({ReplyResult::Valid,0xC55},16000);
    assert(state.data.debug3.value == 0x55);
    state.request(false,1,4); state.cancel();
    assert(state.data.edt.state == EdtState::Cancelled);
    state.ingest({ReplyResult::Valid,0},17000);
    assert(state.data.speed.updated_at_us == 10200); // A zero period must not replace the last speed reading.
    // Calculate RPM directly from the electrical period.
    config.motor_poles = 4;
    state.ingest({ReplyResult::Valid,0x007},18000);
    assert(state.snapshot(18000,config).speed.erpm == 8571429);
    assert(state.snapshot(18000,config).speed.rpm == 4285714);
    config.motor_poles = 254;
    state.ingest({ReplyResult::Valid,0xFFE},19000); // 510 << 7 = 65280 us.
    assert(state.snapshot(19000,config).speed.rpm == 7);
}
}

void run_telemetry_tests() {
    test_telemetry_codec();
    std::puts("PASS: telemetry codec");
    test_transmit_echo();
    std::puts("PASS: TX echo removal, silent completion, D300/D600 replies, symbol alignment and corrupt/truncated TX rejection");
    test_telemetry_state();
    std::puts("PASS: telemetry state");
}
