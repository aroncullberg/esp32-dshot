#include "dshot/passthrough.hpp"
#include "dshot/driver.hpp"
#include "dshot/link.hpp"

namespace dshot {
Passthrough::Passthrough(Driver& driver, Link* const* links, std::size_t count)
    : driver_(driver), count_(count) {
    if (links && count <= links_.size()) {
        for (std::size_t i = 0; i < count; ++i) links_[i] = links[i];
    }
}

Passthrough::Passthrough(Driver& driver, std::initializer_list<Link*> links)
    : Passthrough(driver, links.begin(), links.size()) {}

Passthrough::~Passthrough() {
    if (held_) release(false);
}

void Passthrough::close_links() {
    driver_.stop();
    for (std::size_t i = 0; i < count_; ++i) links_[i]->end();
}

esp_err_t Passthrough::acquire() {
    if (held_) return ESP_OK;
    if (driver_.passthrough_owner_) return ESP_ERR_INVALID_STATE;
    if (count_ > links_.size()) return ESP_ERR_INVALID_ARG;
    const auto error = driver_.validate_passthrough_links(links_.data(), count_);
    if (error != ESP_OK) return error;
    close_links();
    driver_.passthrough_owner_ = this;
    held_ = true;
    return ESP_OK;
}

esp_err_t Passthrough::release(bool resume_zero_frames) {
    if (!held_) return ESP_OK;
    driver_.passthrough_owner_ = nullptr;
    held_ = false;
    if (!resume_zero_frames) return ESP_OK;
    esp_err_t error = ESP_OK;
    for (std::size_t i = 0; i < count_; ++i) {
        error = links_[i]->begin();
        if (error == ESP_OK) error = links_[i]->set(0);
        if (error != ESP_OK) break;
    }
    if (error == ESP_OK) error = driver_.start();
    if (error != ESP_OK) close_links();
    return error;
}

esp_err_t Passthrough::acquire_callback(void* ctx) {
    return ctx ? static_cast<Passthrough*>(ctx)->acquire() : ESP_ERR_INVALID_ARG;
}

esp_err_t Passthrough::release_callback(void* ctx, bool resume_zero_frames) {
    return ctx ? static_cast<Passthrough*>(ctx)->release(resume_zero_frames) : ESP_ERR_INVALID_ARG;
}
}
