#include <cassert>
#include <vector>
#include <string>
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
unsigned snapshots = 0;
std::vector<Command> parsed;
bool command_busy = false;
unsigned accepted_commands = 0;
dshot::CommandStatus command_progress;
}

namespace dshot {
Driver::Driver(const Config& config) : config_(config) {}
Driver::~Driver() {}
esp_err_t Driver::start() { ++starts; stream = true; return ESP_OK; }
void Driver::stop() { ++stops; stream = false; }
esp_err_t Driver::last_error() const { return ESP_OK; }
DriverStatistics Driver::statistics() const {
    ++snapshots;
    DriverStatistics result;
    result.running = stream;
    result.elapsed_us = now_us;
    result.link_count = 1;
    result.links[0].gpio = CONFIG_DSHOT_BENCH_GPIO;
    result.links[0].completed_frames = now_us / 1000;
    return result;
}
Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() {}
Telemetry Link::telemetry() const { return {}; }
ReceiveCapture Link::receive_capture() const { return {}; }
esp_err_t Link::request_edt(bool enabled) {
    return send_command(enabled ? dshot::Command::ExtendedTelemetryEnable : dshot::Command::ExtendedTelemetryDisable);
}
esp_err_t Link::begin() { ++begins; payload = 0; return ESP_OK; }
void Link::end() { ++closes; }
esp_err_t Link::set(std::uint16_t value, bool) {
    if (value != 0 && command_busy) return ESP_ERR_INVALID_STATE;
    if (value == 0 && command_busy) {
        command_busy = false;
        command_progress.state = CommandState::Cancelled;
    }
    payload = value;
    return ESP_OK;
}
esp_err_t Link::send_command(dshot::Command command) {
    if (payload != 0 || command_busy) return ESP_ERR_INVALID_STATE;
    command_busy = true;
    command_progress.id = ++accepted_commands;
    command_progress.command = command;
    command_progress.state = CommandState::Pending;
    command_progress.total_repetitions = command_info(command)->repetitions;
    return ESP_OK;
}
CommandStatus Link::command_status() const { return command_progress; }
}

std::int64_t esp_timer_get_time() { return now_us; }
QueueHandle_t xQueueCreate(unsigned, unsigned) { return reinterpret_cast<void*>(1); }
BaseType_t xQueueSend(QueueHandle_t, const void* command, TickType_t) {
    parsed.push_back(*static_cast<const Command*>(command));
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t, void* command, TickType_t) {
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
    assert(stats_command(1,nullptr) == 0 && parsed.back().action == Action::Stats);
    assert(stats_command(2,nullptr) != 0);
    auto parse = [](esp_console_cmd_func_t callback, std::initializer_list<const char*> args) {
        std::vector<std::string> storage(args.begin(),args.end());
        std::vector<char*> argv;
        for (auto& arg : storage) argv.push_back(arg.data());
        return callback(argv.size(),argv.data());
    };
    assert(parse(beep_command,{"beep"}) == 0 && parsed.back().payload == 1);
    assert(parse(beep_command,{"beep","5"}) == 0 && parsed.back().payload == 5);
    assert(parse(beep_command,{"beep","6"}) != 0);
    assert(parse(direction_command,{"direction","reversed"}) == 0 && parsed.back().payload == 21);
    assert(parse(direction_command,{"direction","2"}) == 0 && parsed.back().payload == 8);
    assert(parse(direction_command,{"direction","reverse"}) != 0);
    assert(parse(save_command,{"save"}) == 0 && parsed.back().payload == 12);
    assert(parse(esc_command,{"command","edt-enable"}) == 0 && parsed.back().payload == 13);
    assert(parse(esc_command,{"command","20"}) == 0 && parsed.back().payload == 20);
    assert(parse(esc_command,{"command","15"}) != 0);
    assert(parse(esc_command,{"command","status"}) == 0 && parsed.back().action == Action::CommandStatus);
    assert(parse(telemetry_command,{"telemetry"}) == 0 && parsed.back().action == Action::Telemetry);
    assert(parse(telemetry_command,{"telemetry","raw"}) == 0 && parsed.back().action == Action::TelemetryRaw);
    assert(parse(telemetry_command,{"telemetry","invalid"}) == 1);
    assert(parse(edt_command,{"edt","on"}) == 0 && parsed.back().action == Action::Edt && parsed.back().payload == 1);
    assert(parse(edt_command,{"edt","off"}) == 0 && parsed.back().payload == 0);
    assert(parse(edt_command,{"edt","status"}) == 0 && parsed.back().action == Action::Telemetry);
    assert(parse(edt_command,{"edt","bad"}) != 0);

    constexpr std::int64_t hold = static_cast<std::int64_t>(CONFIG_DSHOT_BENCH_ARM_HOLD_MS) * 1000;
    events = {
        {0, true, {Action::Run,200}, false, 0},
        {1, true, {Action::Arm}, true, 0},
        {2, true, {Action::Run,200}, true, 0},
        {3, true, {Action::EscCommand,1}, true, 0},
        {hold+2, false, {}, true, 200},
        {hold+100000000, false, {}, true, 200}, // No timeout after 100 seconds.
        {hold+100000000, true, {Action::Stats}, true, 200},
        {hold+100000000, true, {Action::Stats}, true, 200}, // Zero-length interval.
        {hold+100000001, true, {Action::Run,300}, true, 300},
        {hold+100000002, true, {Action::Stop}, true, 0},
        {hold+100000003, false, {}, true, 0},
        {hold+100000004, true, {Action::Arm}, true, 0},
        {hold+100000005, true, {Action::Run,400}, true, 0},
        {hold+100000006, true, {Action::Stop}, true, 0}, // Cancels queued run.
        {2*hold+100000010, false, {}, true, 0},
        {2*hold+100000011, true, {Action::EscCommand,1}, true, 0},
        {2*hold+100000012, true, {Action::Run,200}, true, 0}, // Reject a run request while an ESC command is pending.
        {2*hold+100000013, true, {Action::EscCommand,20}, true, 0},
        {2*hold+100000014, true, {Action::CommandStatus}, true, 0},
        {2*hold+100000015, true, {Action::Stop}, true, 0},
        {2*hold+100000016, true, {Action::Run,200}, true, 200},
        {2*hold+100000017, true, {Action::EscCommand,20}, true, 200}, // Reject an ESC command while the motor is running.
        {2*hold+100000018, true, {Action::Stop}, true, 0},
        {2*hold+100000019, true, {Action::EscCommand,8}, true, 0},
    };
    try { bench_worker(nullptr); } catch (const Done&) {}
    assert(next_event == events.size());
    assert(begins == 1 && starts == 1 && stops == 0 && closes == 0);
    assert(snapshots == 2);
    assert(accepted_commands == 2 && command_progress.command == dshot::Command::Direction2);
    std::puts("PASS: arm keeps streaming, run updates persist, stop sends zero, no run timeout");
}
