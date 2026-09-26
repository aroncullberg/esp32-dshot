#pragma once
#include "hal/gpio_types.h"
#include "esp_err.h"
extern "C" esp_err_t gpio_set_direction(gpio_num_t gpio, gpio_mode_t mode);
