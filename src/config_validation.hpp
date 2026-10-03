#pragma once

#include "dshot/config.hpp"
#include "esp_err.h"

namespace dshot {
    esp_err_t validate_config(const Config& config);
}
