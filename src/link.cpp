#include "dshot/link.hpp"
#include "dshot/driver.hpp"
#include "idf_transport.hpp"
#include "command_sequence.hpp"
#include "telemetry_state.hpp"
#include "stage_profiler.hpp"

#include <new>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

namespace dshot {
struct Link::Impl {
    detail::RmtTransport transport;
    portMUX_TYPE pending_lock = portMUX_INITIALIZER_UNLOCKED;
    std::uint16_t pending_body = 0;
    detail::CommandSequence sequence;
    detail::TelemetryState telemetry;
    ReceiveCapture capture;
};

Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() { end(); }

esp_err_t Link::begin() {
    esp_err_t error = driver_.prepare_link(*this);
    if (error != ESP_OK) return error;
    // ISR callback context and receive buffer must remain in internal SRAM.
    void* storage = heap_caps_malloc(sizeof(Impl), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!storage) return ESP_ERR_NO_MEM;
    impl_ = new (storage) Impl;
    impl_->telemetry.data.available = driver_.config_.signaling == Signaling::Bidirectional;
    impl_->telemetry.data.edt.interpretation_version = driver_.config_.edt_version;
    error = detail::open_transport(impl_->transport, pin_, driver_.config_);
    if (error == ESP_OK) error = driver_.attach_link(*this);
    if (error != ESP_OK) {
        detail::close_transport_or_abort(impl_->transport);
        impl_->~Impl();
        heap_caps_free(impl_);
        impl_ = nullptr;
    }
    return error;
}

void Link::end() {
    if (!impl_) return;
    driver_.stop();
    closed_command_status_ = command_status();
    closed_telemetry_ = telemetry();
    detail::close_transport_or_abort(impl_->transport);
    driver_.detach_link(*this);
    impl_->~Impl();
    heap_caps_free(impl_);
    impl_ = nullptr;
}

esp_err_t Link::set(std::uint16_t payload, bool request_telemetry) {
    if (!impl_ || !impl_->transport.opened) return ESP_ERR_INVALID_STATE;
    if (payload > 2047) return ESP_ERR_INVALID_ARG;
    const std::uint16_t body = (payload << 1) | (request_telemetry ? 1 : 0);
    portENTER_CRITICAL(&impl_->pending_lock);
    if (payload != 0 && impl_->sequence.busy(esp_timer_get_time())) {
        portEXIT_CRITICAL(&impl_->pending_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (payload == 0) {
        impl_->sequence.cancel();
        impl_->telemetry.cancel();
    }
    impl_->pending_body = body;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return ESP_OK;
}

bool Link::ready() const {
    // Called before worker launch or from the worker.
    return impl_ && impl_->transport.opened && impl_->transport.enabled &&
           !impl_->transport.faulted;
}

esp_err_t Link::send_command(Command command) {
    if (!impl_ || !impl_->transport.opened) return ESP_ERR_INVALID_STATE;
    const bool edt = command == Command::ExtendedTelemetryEnable || command == Command::ExtendedTelemetryDisable;
    if (edt && !impl_->transport.bidirectional) return ESP_ERR_NOT_SUPPORTED;
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto error = (impl_->pending_body >> 1) != 0 ? ESP_ERR_INVALID_STATE :
        impl_->sequence.begin(command, esp_timer_get_time());
    if (error == ESP_OK) {
        impl_->telemetry.cancel();
        if (edt) impl_->telemetry.request(command == Command::ExtendedTelemetryEnable,
            driver_.config_.edt_version, impl_->sequence.status().id);
    }
    portEXIT_CRITICAL(&impl_->pending_lock);
    return error;
}

esp_err_t Link::request_edt(bool enabled) {
    return send_command(enabled ? Command::ExtendedTelemetryEnable : Command::ExtendedTelemetryDisable);
}

TelemetryCounters Link::telemetry_counters() const {
    if (!impl_) return closed_telemetry_.counters;
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto counters = impl_->telemetry.data.counters;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return counters;
}

Telemetry Link::telemetry() const {
    detail::TelemetryState snapshot;
    if (!impl_) snapshot.data = closed_telemetry_;
    else {
        portENTER_CRITICAL(&impl_->pending_lock);
        snapshot = impl_->telemetry;
        portEXIT_CRITICAL(&impl_->pending_lock);
    }
    return snapshot.snapshot(esp_timer_get_time(), driver_.config_);
}

ReceiveCapture Link::receive_capture() const {
    if (!impl_) return {};
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto snapshot = impl_->capture;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return snapshot;
}

CommandStatus Link::command_status() const {
    if (!impl_) return closed_command_status_;
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto status = impl_->sequence.status();
    portEXIT_CRITICAL(&impl_->pending_lock);
    return status;
}

void Link::cancel_pending_command() {
    if (!impl_) return;
    portENTER_CRITICAL(&impl_->pending_lock);
    impl_->sequence.cancel();
    impl_->telemetry.cancel();
    portEXIT_CRITICAL(&impl_->pending_lock);
}

esp_err_t Link::transmit_pending(std::uint64_t* wait_us, bool& transmitted, std::uint64_t& rx_wait_us,
                                 detail::StageProfiler* profile) {
    detail::profile_stage(profile, ProfileStage::FramePrepare);
    transmitted = false;
    rx_wait_us = 0;
    if (!ready()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&impl_->pending_lock);
    auto& telemetry = impl_->telemetry;
    if (telemetry.awaiting() && telemetry.deadline_us && esp_timer_get_time() >= telemetry.deadline_us) {
        const auto now = esp_timer_get_time();
        if (!impl_->sequence.busy(now)) {
            if ((impl_->pending_body >> 1) == 0 && telemetry.data.edt.attempts < driver_.config_.edt_attempts) {
                const auto command = telemetry.enable_requested ? Command::ExtendedTelemetryEnable : Command::ExtendedTelemetryDisable;
                if (impl_->sequence.begin(command,now) == ESP_OK) {
                    ++telemetry.data.edt.attempts;
                    telemetry.command_id = impl_->sequence.status().id;
                    telemetry.deadline_us = 0;
                }
            } else telemetry.data.edt.state = EdtState::Timeout;
        }
    }
    const auto frame = impl_->sequence.select(impl_->pending_body,
        impl_->sequence.active() ? esp_timer_get_time() : 0);
    portEXIT_CRITICAL(&impl_->pending_lock);
    if (!frame.send) return ESP_OK;
    const auto body = frame.body;
    std::uint16_t packet;
    auto error = detail::encode_regular_packet(body >> 1, (body & 1) != 0, packet);
    if (error == ESP_OK) error = detail::send_packet(impl_->transport, packet, wait_us, profile);
    detail::profile_stage(profile, ProfileStage::TelemetryUpdate);
    transmitted = error == ESP_OK;
    if (frame.id != 0 || impl_->transport.bidirectional) {
        const auto now = esp_timer_get_time();
        portENTER_CRITICAL(&impl_->pending_lock);
        impl_->sequence.complete(frame, error, now);
        if (telemetry.awaiting() && frame.id == telemetry.command_id) {
            if (error != ESP_OK) telemetry.data.edt.state = EdtState::Failed;
            else if (frame.command && !telemetry.deadline_us)
                telemetry.deadline_us = now + static_cast<std::int64_t>(driver_.config_.edt_response_timeout_ms) * 1000;
        }
        if (transmitted && impl_->transport.bidirectional) {
            const auto& transport = impl_->transport;
            telemetry.ingest(transport.reply,now);
            if (transport.reply.result == detail::ReplyResult::NoResponse) {
                if (transport.rx_timed_out) ++telemetry.data.counters.rx_timeouts;
                else ++telemetry.data.counters.empty_captures;
            }
            // Lifecycle calls serialize changes to opened.
            if (transport.rx_count && transport.reply.result != detail::ReplyResult::NoResponse &&
                transport.reply.result != detail::ReplyResult::ReceiveError) {
                detail::profile_stage(profile, ProfileStage::CaptureCopy);
                auto& capture = impl_->capture;
                capture.captured_at_us = now;
                capture.resolution_hz = detail::kRMTResolutionHz;
                capture.bitrate = transport.bitrate;
                capture.result = transport.reply.result;
                const auto count = transport.rx_count < 32 ? transport.rx_count : 32;
                capture.pulse_count = count * 2;
                for (unsigned i = 0; i < count; ++i) {
                    const auto& symbol = transport.rx_symbols[i];
                    capture.pulses[2*i] = {static_cast<std::uint16_t>(symbol.duration0), symbol.level0 != 0};
                    capture.pulses[2*i+1] = {static_cast<std::uint16_t>(symbol.duration1), symbol.level1 != 0};
                }
            }
        }
        detail::profile_stage(profile, ProfileStage::LinkFinish);
        portEXIT_CRITICAL(&impl_->pending_lock);
    }
    rx_wait_us = impl_->transport.rx_wait_us;
    return error;
}
}
