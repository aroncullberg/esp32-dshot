#pragma once

#include "dshot/config.hpp"

namespace dshot::detail {
    inline bool is_supported_protocol(Protocol protocol) {
        switch (protocol) {
            case Protocol::DShot150:
            case Protocol::DShot300:
            case Protocol::DShot600:
                return true;
            default:
                return false;
        }
    }
}
