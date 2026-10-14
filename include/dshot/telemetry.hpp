#pragma once

#include <cstdint>

namespace dshot {
enum class ErpmState { NoData, Moving, Stopped, Stale };
enum class EdtState { Disabled, Enabling, Enabled, Disabling, Timeout, Cancelled, Failed };
enum class ReceiveResult { Valid, NoResponse, Malformed, GcrError, ChecksumError, Overflow, ReceiveError };

// Zero-duration entries mark the end of the capture.
struct ReceiveCapture {
    struct Pulse { std::uint16_t ticks = 0; bool high = false; };
    std::int64_t captured_at_us = 0;
    std::uint32_t resolution_hz = 0, bitrate = 0;
    ReceiveResult result = ReceiveResult::NoResponse;
    unsigned pulse_count = 0;
    Pulse pulses[64]{};
};

struct TelemetryField {
    std::uint32_t value = 0;
    std::int64_t updated_at_us = 0;
    bool valid = false; // Zero is a valid received value.
    bool fresh = false; // Freshness is checked against edt_max_age_us when telemetry is read.
};
struct ErpmSample {
    ErpmState state = ErpmState::NoData;
    std::uint32_t erpm = 0; // e = electrical, so its electrial revolutions per minute (i think, dont actually trust me on this)
    std::uint32_t period_us = 0;
    std::int64_t updated_at_us = 0;
    std::uint32_t rpm = 0;
    unsigned motor_poles = 0;
};
struct EdtStatus {
    EdtState state = EdtState::Disabled;
    unsigned interpretation_version = 2;
    unsigned attempts = 0;
    TelemetryField acknowledgement; // 0xFF acknowledges disabling EDT, other values acknowledge enabling it.
};
struct TelemetryCounters {
    std::uint64_t replies = 0;
    std::uint64_t no_response = 0;
    std::uint64_t malformed = 0;
    std::uint64_t gcr_errors = 0;
    std::uint64_t checksum_errors = 0;
    std::uint64_t overflows = 0;
    std::uint64_t receive_errors = 0;
    std::uint64_t erpm_frames = 0;
    std::uint64_t edt_frames = 0;
    std::uint64_t rx_timeouts = 0; // RX completion did not arrive before the task's deadline.
    std::uint64_t empty_captures = 0; // RX completed with no reply (empty or idle-only capture).
};
struct Telemetry {
    bool available = false; // True only when the link uses bidirectional signaling.
    ErpmSample speed;
    EdtStatus edt;
    TelemetryField temperature_c, voltage_mv, current_ma;
    TelemetryField debug1, debug2, debug3, stress;
    TelemetryField status; // Raw byte, including reserved bits.
    TelemetryField last_payload, unknown;
    // Counts each version 2 reply with its corresponding status bit set.
    std::uint64_t alert_events = 0, warning_events = 0, error_events = 0;
    TelemetryCounters counters;
};
const char* erpm_state_name(ErpmState state);
const char* edt_state_name(EdtState state);
}
