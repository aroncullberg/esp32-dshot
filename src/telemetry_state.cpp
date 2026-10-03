#include "telemetry_state.hpp"
#include <initializer_list>

namespace dshot {
const char* erpm_state_name(ErpmState state) {
    switch (state) {
    case ErpmState::NoData: return "no data";
    case ErpmState::Moving: return "moving";
    case ErpmState::Stopped: return "stopped";
    case ErpmState::Stale: return "stale";
    }
    return "unknown";
}
const char* edt_state_name(EdtState state) {
    switch (state) {
    case EdtState::Disabled: return "disabled";
    case EdtState::Enabling: return "enabling";
    case EdtState::Enabled: return "enabled";
    case EdtState::Disabling: return "disabling";
    case EdtState::Timeout: return "timeout";
    case EdtState::Cancelled: return "cancelled";
    case EdtState::Failed: return "failed";
    }
    return "unknown";
}
}
namespace dshot::detail {
namespace {
void update(TelemetryField& field, std::uint32_t value, std::int64_t now) {
    field = {value,now,true,true};
}
}
bool TelemetryState::awaiting() const {
    return data.edt.state == EdtState::Enabling || data.edt.state == EdtState::Disabling;
}
void TelemetryState::request(bool enable, unsigned version, std::uint64_t id) {
    enable_requested = enable;
    data.edt = {};
    data.edt.state = enable ? EdtState::Enabling : EdtState::Disabling;
    data.edt.interpretation_version = version;
    data.edt.attempts = 1;
    command_id = id;
    deadline_us = 0; // The response timeout starts after the first command transmission.
}
void TelemetryState::cancel() {
    if (awaiting()) data.edt.state = EdtState::Cancelled;
}
void TelemetryState::ingest(DecodedReply reply, std::int64_t now) {
    auto& c = data.counters;
    switch (reply.result) {
    case ReplyResult::NoResponse: ++c.no_response; return;
    case ReplyResult::Malformed: ++c.malformed; return;
    case ReplyResult::GcrError: ++c.gcr_errors; return;
    case ReplyResult::ChecksumError: ++c.checksum_errors; return;
    case ReplyResult::Overflow: ++c.overflows; return;
    case ReplyResult::ReceiveError: ++c.receive_errors; return;
    case ReplyResult::Valid: ++c.replies; break;
    }
    const auto value = reply.payload;
    update(data.last_payload,value,now);
    const auto prefix = value >> 8;
    const auto byte = value & 255;
    if (awaiting() && deadline_us && prefix == 14 &&
        ((enable_requested && byte != 255) || (!enable_requested && byte == 255))) {
        update(data.edt.acknowledgement,byte,now);
        extended = enable_requested;
        data.edt.state = extended ? EdtState::Enabled : EdtState::Disabled;
        ++c.edt_frames;
        return;
    }
    if (extended && prefix != 0 && (prefix & 1) == 0) {
        ++c.edt_frames;
        switch (prefix) {
        case 2: update(data.temperature_c,byte,now); break;
        case 4: update(data.voltage_mv,byte * 250,now); break;
        case 6: update(data.current_ma,byte * 1000,now); break;
        case 8: update(data.debug1,byte,now); break;
        case 10: update(data.debug2,byte,now); break;
        case 12:
            update(data.edt.interpretation_version == 2 ? data.stress : data.debug3,byte,now);
            break;
        case 14:
            update(data.status,byte,now);
            if (byte == 255) { extended = false; data.edt.state = EdtState::Disabled; break; }
            if (data.edt.interpretation_version == 2) {
                data.alert_events += (byte >> 7) & 1;
                data.warning_events += (byte >> 6) & 1;
                data.error_events += (byte >> 5) & 1;
                if (byte & 16) update(data.unknown,value,now);
            }
            break;
        default: update(data.unknown,value,now); break;
        }
        return;
    }
    const unsigned period = (value & 511) << (value >> 9);
    if (value != 0xFFF && period == 0) { ++c.malformed; return; }
    ++c.erpm_frames;
    data.speed = {value == 0xFFF ? ErpmState::Stopped : ErpmState::Moving,
        value == 0xFFF ? 0u : (60'000'000u + period / 2) / period,
        value == 0xFFF ? 0u : period, now};
}
Telemetry TelemetryState::snapshot(std::int64_t now, const Config& config) const {
    auto result = data;
    result.speed.motor_poles = config.motor_poles;
    result.speed.rpm = 0;
    if (result.speed.period_us && config.motor_poles >= 2 &&
        config.motor_poles <= 254 && config.motor_poles % 2 == 0) {
        // Convert directly from the electrical period to round only once.
        const auto mechanical_period = static_cast<std::uint64_t>(result.speed.period_us) *
                                       (config.motor_poles / 2);
        result.speed.rpm = (60'000'000ull + mechanical_period / 2) / mechanical_period;
    }
    if (result.speed.state != ErpmState::NoData &&
        now - result.speed.updated_at_us > config.erpm_max_age_us)
        result.speed.state = ErpmState::Stale;
    for (auto* field : {&result.temperature_c,&result.voltage_mv,&result.current_ma,
                       &result.debug1,&result.debug2,&result.debug3,&result.stress,
                       &result.status,&result.last_payload,&result.unknown,&result.edt.acknowledgement})
        field->fresh = field->valid && now - field->updated_at_us <= config.edt_max_age_us;
    return result;
}
}
