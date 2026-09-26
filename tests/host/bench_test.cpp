// Execute the real console parser and worker with scripted commands and time.
#include <cassert>
#include <vector>
#include "../../examples/bench/main/bench_main.cpp"

namespace {
struct Done {};
struct Event {
    std::int64_t at_us;
    bool command;
    Command value;
    bool expected_stream;
    unsigned expected_payload;
};
std::vector<Event> events;
std::size_t next_event = 0;
std::int64_t now_us = 0;
bool stream = false;
unsigned payload = 0, begins = 0, starts = 0, stops = 0, closes = 0;
std::vector<Command> parsed;
}

namespace dshot {
Driver::Driver(const Config& config) : config_(config) {}
Driver::~Driver() {}
esp_err_t Driver::start() { ++starts; stream = true; return ESP_OK; }
void Driver::stop() { ++stops; stream = false; }
esp_err_t Driver::last_error() const { return ESP_OK; }
Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() {}
esp_err_t Link::begin() { ++begins; payload = 0; return ESP_OK; }
void Link::end() { ++closes; }
esp_err_t Link::set(std::uint16_t value, bool) { payload = value; return ESP_OK; }
}

std::int64_t esp_timer_get_time() { return now_us; }
QueueHandle_t xQueueCreate(unsigned, unsigned) { return reinterpret_cast<void*>(1); }
BaseType_t xQueueSend(QueueHandle_t, const void* command, TickType_t) {
    parsed.push_back(*static_cast<const Command*>(command));
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t, void* command, TickType_t) {
    // The previous event must have produced these outputs before the next receive.
    if (next_event != 0) {
        const auto& previous = events[next_event - 1];
        assert(stream == previous.expected_stream);
        assert(payload == previous.expected_payload);
    }
    if (next_event == events.size()) throw Done{};
    const auto& event = events[next_event++];
    now_us = event.at_us;
    *static_cast<Command*>(command) = event.value;
    return event.command ? pdTRUE : pdFALSE;
}
void vQueueDelete(QueueHandle_t) {}
BaseType_t xTaskCreate(void (*)(void*), const char*, unsigned, void*, unsigned, TaskHandle_t*) { return pdPASS; }
esp_err_t esp_console_cmd_register(const esp_console_cmd_t*) { return ESP_OK; }
esp_err_t esp_console_new_repl_stdio(const esp_console_repl_config_t*, esp_console_repl_t**) { return ESP_OK; }
esp_err_t esp_console_start_repl(esp_console_repl_t*) { return ESP_OK; }
extern "C" const char* esp_err_to_name(esp_err_t) { return "test error"; }
extern "C" void _esp_error_check_failed(esp_err_t, const char*, int, const char*, const char*) { std::abort(); }

int main() {
    char run[] = "run", a[] = "200", b[] = "300", invalid[] = "2048", junk[] = "4x";
    char* first[] = {run,a};
    char* second[] = {run,b};
    char* bad[] = {run,invalid};
    char* bad_text[] = {run,junk};
    assert(run_command(2,first) == 0 && parsed.back().payload == 200);
    assert(run_command(2,second) == 0 && parsed.back().payload == 300);
    const auto count = parsed.size();
    assert(run_command(2,bad) != 0 && run_command(2,bad_text) != 0);
    assert(parsed.size() == count);
    assert(arm_command(1,nullptr) == 0 && parsed.back().action == Action::Arm);
    assert(stop_command(1,nullptr) == 0 && parsed.back().action == Action::Stop);

    constexpr std::int64_t hold = static_cast<std::int64_t>(CONFIG_DSHOT_BENCH_ARM_HOLD_MS) * 1000;
    events = {
        {0, true, {Action::Run,200}, false, 0},
        {1, true, {Action::Arm}, true, 0},
        {2, true, {Action::Run,200}, true, 0},
        {hold+2, false, {}, true, 200},
        {hold+100000000, false, {}, true, 200}, // No timeout, even after 100 seconds.
        {hold+100000001, true, {Action::Run,300}, true, 300},
        {hold+100000002, true, {Action::Stop}, true, 0},
        {hold+100000003, false, {}, true, 0}, // Stop preserves the frame stream.
        {hold+100000004, true, {Action::Arm}, true, 0},
        {hold+100000005, true, {Action::Run,400}, true, 0},
        {hold+100000006, true, {Action::Stop}, true, 0}, // Cancels queued run.
        {2*hold+100000010, false, {}, true, 0},
    };
    try { bench_worker(nullptr); } catch (const Done&) {}
    assert(next_event == events.size());
    assert(begins == 1 && starts == 1 && stops == 0 && closes == 0);
    std::puts("PASS: arm keeps streaming, run updates persist, stop sends zero, no run timeout");
}
