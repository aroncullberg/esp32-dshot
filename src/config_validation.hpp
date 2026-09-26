#pragma once

#include "dshot/config.hpp"
#include "esp_err.h"

namespace dshot {
    // Internal validation for the currently implemented transport/scheduler.
    esp_err_t validate_config(const Config& config);
}
