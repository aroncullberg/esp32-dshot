#pragma once
#include <cstdint>
#include "esp_err.h"
#include "soc/clk_tree_defs.h"
struct HostTimer;
using gptimer_handle_t = HostTimer*;
constexpr int GPTIMER_COUNT_UP = 0;
struct gptimer_config_t { int clk_src; int direction; std::uint32_t resolution_hz; };
struct gptimer_alarm_event_data_t { std::uint64_t count_value, alarm_value; };
using gptimer_alarm_cb_t = bool (*)(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
struct gptimer_event_callbacks_t { gptimer_alarm_cb_t on_alarm; };
struct gptimer_alarm_config_t {
    std::uint64_t alarm_count, reload_count;
    struct { unsigned auto_reload_on_alarm : 1; } flags;
};
esp_err_t gptimer_new_timer(const gptimer_config_t*, gptimer_handle_t*);
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t, const gptimer_event_callbacks_t*, void*);
esp_err_t gptimer_set_alarm_action(gptimer_handle_t, const gptimer_alarm_config_t*);
esp_err_t gptimer_enable(gptimer_handle_t);
esp_err_t gptimer_start(gptimer_handle_t);
esp_err_t gptimer_stop(gptimer_handle_t);
esp_err_t gptimer_disable(gptimer_handle_t);
esp_err_t gptimer_del_timer(gptimer_handle_t);
