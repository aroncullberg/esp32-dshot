#include "dshot/link.hpp"
#include "dshot/driver.hpp"
#include "idf_transport.hpp"
#include "command_sequence.hpp"

#include <new>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

namespace dshot {
struct Link::Impl {
    detail::RmtTransport transport;
    portMUX_TYPE pending_lock = portMUX_INITIALIZER_UNLOCKED;
    std::uint16_t pending_body = 0;
    detail::CommandSequence sequence;
};

Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() { end(); }

esp_err_t Link::begin() {
    esp_err_t error = driver_.prepare_link(*this);
    if (error != ESP_OK) return error;
    impl_ = new (std::nothrow) Impl;
    if (!impl_) return ESP_ERR_NO_MEM;
    error = detail::open_transport(impl_->transport, pin_, driver_.config_);
    if (error == ESP_OK) error = driver_.attach_link(*this);
    if (error != ESP_OK) {
        detail::close_transport_or_abort(impl_->transport);
        delete impl_;
        impl_ = nullptr;
    }
    return error;
}

void Link::end() {
    if (!impl_) return;
    driver_.stop();
    closed_command_status_ = command_status();
    detail::close_transport_or_abort(impl_->transport);
    driver_.detach_link(*this);
    delete impl_;
    impl_ = nullptr;
}

esp_err_t Link::set(std::uint16_t payload, bool request_telemetry) {
    // opened is changed only by the serialized application lifecycle, not the worker.
    if (!impl_ || !impl_->transport.opened) return ESP_ERR_INVALID_STATE;
    if (payload > 2047) return ESP_ERR_INVALID_ARG;
    const std::uint16_t body = (payload << 1) | (request_telemetry ? 1 : 0);
    portENTER_CRITICAL(&impl_->pending_lock);
    if (payload != 0 && impl_->sequence.busy(esp_timer_get_time())) {
        portEXIT_CRITICAL(&impl_->pending_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (payload == 0) impl_->sequence.cancel();
    impl_->pending_body = body;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return ESP_OK;
}

bool Link::ready() const {
    // Called before worker launch, or by that worker. Not a concurrent health API.
    return impl_ && impl_->transport.opened && impl_->transport.enabled &&
           !impl_->transport.faulted;
}

esp_err_t Link::send_command(Command command) {
    if (!impl_ || !impl_->transport.opened) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto error = (impl_->pending_body >> 1) != 0 ? ESP_ERR_INVALID_STATE :
        impl_->sequence.begin(command, esp_timer_get_time());
    portEXIT_CRITICAL(&impl_->pending_lock);
    return error;
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
    portEXIT_CRITICAL(&impl_->pending_lock);
}

esp_err_t Link::transmit_pending(std::uint64_t* wait_us, bool& transmitted) {
    transmitted = false;
    if (!ready()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto frame = impl_->sequence.select(impl_->pending_body,
        impl_->sequence.active() ? esp_timer_get_time() : 0);
    portEXIT_CRITICAL(&impl_->pending_lock);
    if (!frame.send) return ESP_OK;
    const auto body = frame.body;
    std::uint16_t packet;
    auto error = detail::encode_regular_packet(body >> 1, (body & 1) != 0, packet);
    if (error == ESP_OK) error = detail::send_packet(impl_->transport, packet, wait_us);
    transmitted = error == ESP_OK;
    if (frame.id != 0) {
        const auto now = esp_timer_get_time();
        portENTER_CRITICAL(&impl_->pending_lock);
        impl_->sequence.complete(frame, error, now);
        portEXIT_CRITICAL(&impl_->pending_lock);
    }
    return error;
}
}
