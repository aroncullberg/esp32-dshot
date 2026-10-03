#pragma once
#include <cstdlib>
#include "soc/gpio_struct.h"
inline void gpio_ll_output_enable(gpio_dev_t* gpio, unsigned pin) { gpio->enabled.at(pin) = true; }
inline void gpio_ll_output_disable(gpio_dev_t* gpio, unsigned pin) { gpio->enabled.at(pin) = false; }
inline void gpio_ll_set_output_enable_ctrl(gpio_dev_t* gpio, unsigned pin, bool peripheral, bool invert) {
    if (invert) std::abort();
    gpio->peripheral_oe.at(pin) = peripheral;
}
