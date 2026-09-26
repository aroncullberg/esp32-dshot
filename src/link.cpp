#include "dshot/link.hpp"
#include "dshot/driver.hpp"
#include "idf_transport.hpp"

#include <new>
#include "freertos/FreeRTOS.h"

namespace dshot {
struct Link::Impl {
    detail::RmtTransport transport;
    portMUX_TYPE pending_lock = portMUX_INITIALIZER_UNLOCKED;
    std::uint16_t pending_body = 0;
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
    impl_->pending_body = body;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return ESP_OK;
}

bool Link::ready() const {
    // Called before worker launch, or by that worker. Not a concurrent health API.
    return impl_ && impl_->transport.opened && impl_->transport.enabled &&
           !impl_->transport.faulted;
}

std::uint16_t Link::snapshot_pending() {
    portENTER_CRITICAL(&impl_->pending_lock);
    const auto body = impl_->pending_body;
    portEXIT_CRITICAL(&impl_->pending_lock);
    return body;
}

esp_err_t Link::transmit_pending() {
    if (!ready()) return ESP_ERR_INVALID_STATE;
    const auto body = snapshot_pending();
    std::uint16_t packet;
    const auto error = detail::encode_regular_packet(body >> 1, (body & 1) != 0, packet);
    if (error != ESP_OK) return error;
    return detail::send_packet(impl_->transport, packet);
}
}
