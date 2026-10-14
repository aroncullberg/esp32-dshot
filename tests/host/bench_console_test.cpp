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
    int expected_other_payload = -1;
    unsigned expected_open_motors = all_motors;
};
std::vector<Event> events;
std::size_t next_event = 0;
std::int64_t now_us = 0;
bool stream = false;
unsigned payload = 0, begins = 0, starts = 0, stops = 0, closes = 0;
unsigned snapshots = 0;
std::vector<Command> parsed;
std::array<unsigned, motor_pins.size()> motor_payloads{};
std::array<bool, motor_pins.size()> command_busy{};
std::array<dshot::CommandStatus, motor_pins.size()> command_statuses{};
unsigned open_motors = 0;
int fail_begin_motor = -1;
unsigned accepted_commands = 0;
dshot::CommandStatus command_progress;
std::size_t motor_index(gpio_num_t pin) {
    for (std::size_t i = 0; i < motor_pins.size(); ++i) if (motor_pins[i] == pin) return i;
    std::abort();
}
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
    for (std::size_t i = 0; i < motor_pins.size(); ++i) {
        if (!(open_motors & (1u << i))) continue;
        auto& link = result.links[result.link_count++];
        link.gpio = motor_pins[i];
        link.completed_frames = now_us / 1000;
    }
    return result;
}
Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() {}
Telemetry Link::telemetry() const { return {}; }
ReceiveCapture Link::receive_capture() const { return {}; }
esp_err_t Link::request_edt(bool enabled) {
    return send_command(enabled ? dshot::Command::ExtendedTelemetryEnable : dshot::Command::ExtendedTelemetryDisable);
}
esp_err_t Link::begin() {
    const auto i = motor_index(pin_);
    if (static_cast<int>(i) == fail_begin_motor) return ESP_FAIL;
    ++begins;
    motor_payloads[i] = 0;
    if (i == 0) payload = 0;
    open_motors |= 1u << i;
    return ESP_OK;
}
void Link::end() {
    const auto i = motor_index(pin_);
    if (open_motors & (1u << i)) { ++closes; open_motors &= ~(1u << i); }
}
esp_err_t Link::set(std::uint16_t value, bool) {
    const auto i = motor_index(pin_);
    assert(open_motors & (1u << i));
    if (value != 0 && command_busy[i]) return ESP_ERR_INVALID_STATE;
    if (value == 0 && command_busy[i]) {
        command_busy[i] = false;
        command_statuses[i].state = CommandState::Cancelled;
        if (i == 0) command_progress = command_statuses[i];
    }
    motor_payloads[i] = value;
    if (i == 0) payload = value;
    return ESP_OK;
}
esp_err_t Link::send_command(dshot::Command command) {
    const auto i = motor_index(pin_);
    if (motor_payloads[i] != 0 || command_busy[i]) return ESP_ERR_INVALID_STATE;
    command_busy[i] = true;
    auto& status = command_statuses[i];
    status.id = ++accepted_commands;
    status.command = command;
    status.state = CommandState::Pending;
    status.total_repetitions = command_info(command)->repetitions;
    if (i == 0) command_progress = status;
    return ESP_OK;
}
CommandStatus Link::command_status() const { return command_statuses[motor_index(pin_)]; }
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
        assert(open_motors == (stream ? previous.expected_open_motors : 0));
        for (std::size_t i = 1; i < motor_pins.size(); ++i)
            assert(motor_payloads[i] == (previous.expected_other_payload < 0 ?
                previous.expected_payload : static_cast<unsigned>(previous.expected_other_payload)));
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
#if DSHOT_BENCH_FOUR_MOTORS
    assert(parse(motor_command,{"motor","all"}) == 0 && parsed.back().payload == all_motors);
    for (unsigned i = 1; i <= 4; ++i) {
        const auto number = std::to_string(i);
        assert(parse(motor_command,{"motor",number.c_str()}) == 0 && parsed.back().payload == (1u << (i - 1)));
    }
    for (const auto* invalid_motor : {"0","5","-1","1x","11"})
        assert(parse(motor_command,{"motor",invalid_motor}) != 0);
#endif

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
    assert(begins == motor_pins.size() && starts == 1 && stops == 0 && closes == 0);
    assert(snapshots == 2);
    assert(accepted_commands == 2 * motor_pins.size() && command_progress.command == dshot::Command::Direction2);
    std::puts("PASS: arm keeps streaming, run updates persist, stop sends zero, no run timeout");

#if DSHOT_BENCH_FOUR_MOTORS
    // Fresh worker, with four distinct link states.
    stream = false;
    payload = begins = starts = stops = closes = accepted_commands = open_motors = 0;
    motor_payloads.fill(0);
    command_busy.fill(false);
    command_statuses.fill({});
    next_event = 0;
    events = {
        {0, true, {Action::Arm}, true, 0},
        {hold + 1, false, {}, true, 0},
        {hold + 2, true, {Action::Run,200}, true, 200},
        {hold + 3, true, {Action::Motor,1}, true, 200}, // Selection alone preserves other throttles.
        {hold + 4, true, {Action::Run,300}, true, 300, 200},
        {hold + 5, true, {Action::Stop}, true, 0}, // Stop all, even with only motor 1 selected.
        {hold + 6, true, {Action::EscCommand,1}, true, 0}, // Only motor 1 gets the command.
        {hold + 7, true, {Action::Motor,all_motors}, true, 0},
        {hold + 8, true, {Action::Run,400}, true, 0}, // Busy motor blocks the entire throttle update.
        {hold + 9, true, {Action::Stop}, true, 0},
        {hold + 10, true, {Action::Motor,1}, true, 0},
        {hold + 11, true, {Action::Arm}, true, 0, 0, 1}, // Restart with only one open output.
        {hold + 12, true, {Action::Run,500}, true, 0, 0, 1},
        {hold + 13, true, {Action::Stop}, true, 0, 0, 1}, // Cancel a queued run during arming.
        {2*hold + 12, false, {}, true, 0, 0, 1},
        {2*hold + 13, true, {Action::Motor,all_motors}, true, 0, 0, 1},
        {2*hold + 14, true, {Action::Run,600}, true, 0, 0, 1}, // Reject a run to unopened outputs.
        {2*hold + 15, true, {Action::Arm}, true, 0}, // Restore all four.
        {3*hold + 16, false, {}, true, 0},
    };
    try { bench_worker(nullptr); } catch (const Done&) {}
    assert(begins == 9 && starts == 3 && closes == 5);
    assert(accepted_commands == 1 && !command_busy[0]);
    std::puts("PASS: motor selection, independent throttle, stop-all, group busy check and one/four-output isolation");

    // A failed allocation halfway through arming must release the already-open links.
    stream = false;
    payload = begins = starts = stops = closes = open_motors = 0;
    next_event = 0;
    fail_begin_motor = 2;
    events = {{0, true, {Action::Arm}, false, 0}};
    try { bench_worker(nullptr); } catch (const Done&) {}
    assert(begins == 2 && closes == 2 && starts == 0 && open_motors == 0);
    std::puts("PASS: four-motor bench rolls back partial arming failure");
#endif
}
