#pragma once

// #include <cstdint>

namespace dshot {
    // NOTE: want to use uint32_t but i get inlcude warning when i try to include <cstdint> in this header. so for now using unsigned
    enum class Protocol : unsigned {
        DShot150 = 150000,
        DShot300 = 300000,
        DShot600 = 600000,
        Dshot1200 = 1200000
    };

    enum class Signaling {
        Regular,
        Bidirectional
    };

    class Config {
        public:
            Config(); // STUDY: default constructor(?), how/why/what for?

            Protocol protocol;
            Signaling signaling;
            unsigned frame_hz;
            unsigned task_priority;
            unsigned task_stack_bytes;
            int task_core;
            unsigned erpm_max_age_us;
    };
}
