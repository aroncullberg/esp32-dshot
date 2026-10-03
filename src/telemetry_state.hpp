#pragma once
#include "dshot/config.hpp"
#include "dshot/telemetry.hpp"
#include "telemetry_codec.hpp"

namespace dshot::detail {
class TelemetryState {
public:
    Telemetry data;
    bool extended = false;
    bool enable_requested = false;
    std::int64_t deadline_us = 0;
    std::uint64_t command_id = 0;
    bool awaiting() const;
    void request(bool enable, unsigned version, std::uint64_t id);
    void cancel();
    void ingest(DecodedReply reply, std::int64_t now);
    Telemetry snapshot(std::int64_t now, const Config& config) const;
};
}
