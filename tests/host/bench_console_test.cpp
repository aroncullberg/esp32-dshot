#include <cassert>
#include <vector>
#include <string>
#include <limits>
#include <unistd.h>
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
bool stage_profile_requested = false;
#if DSHOT_BENCH_FOUR_MOTORS && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
std::array<configRUN_TIME_COUNTER_TYPE, configNUMBER_OF_CORES> idle_counters{};
std::array<configRUN_TIME_COUNTER_TYPE, configNUMBER_OF_CORES> idle_increments{};
unsigned cpu_delays = 0;

#endif
#if DSHOT_BENCH_FOUR_MOTORS
template<typename Function>
std::string capture_output(Function function) {
    std::fflush(stdout);
    FILE* output = std::tmpfile();
    assert(output);
    const int saved = dup(fileno(stdout));
    assert(saved >= 0 && dup2(fileno(output), fileno(stdout)) >= 0);
    function();
    std::fflush(stdout);
    assert(dup2(saved, fileno(stdout)) >= 0);
    close(saved);
    std::rewind(output);
    std::string result;
    char buffer[256];
    while (std::fgets(buffer, sizeof(buffer), output)) result += buffer;
    std::fclose(output);
    return result;
}
#endif
std::size_t motor_index(gpio_num_t pin) {
    for (std::size_t i = 0; i < motor_pins.size(); ++i) if (motor_pins[i] == pin) return i;
    std::abort();
}
}

namespace dshot {
Driver::Driver(const Config& config) : config_(config) {}
Driver::~Driver() {}
esp_err_t Driver::set_stage_profiling(bool enabled) {
    stage_profile_requested = enabled && CONFIG_DSHOT_STAGE_PROFILING;
    return enabled && !CONFIG_DSHOT_STAGE_PROFILING ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}
esp_err_t Driver::start() { ++starts; stream = true; return ESP_OK; }
void Driver::stop() { ++stops; stream = false; }
esp_err_t Driver::last_error() const { return ESP_OK; }
DriverStatistics Driver::statistics() const {
    ++snapshots;
    DriverStatistics result;
    result.running = stream;
    result.elapsed_us = now_us;
    result.worker_iterations = stream ? now_us / 1000 : 0;
    result.missed_slots = stream ? now_us / 1000000 : 0;
    result.stage_profile.available = CONFIG_DSHOT_STAGE_PROFILING;
    result.stage_profile.sample_interval = 16;
    result.stage_profile.sampled_passes = stage_profile_requested && stream ? now_us / 16000 : 0;
    auto& tx_stage = result.stage_profile.stages[static_cast<std::size_t>(ProfileStage::TxWait)];
    tx_stage.count = result.stage_profile.sampled_passes * 4;
    tx_stage.elapsed_us = tx_stage.count * 40;
    tx_stage.busy_us = tx_stage.count * 4;
    for (std::size_t i = 0; i < motor_pins.size(); ++i) {
        if (!(open_motors & (1u << i))) continue;
        auto& link = result.links[result.link_count++];
        link.gpio = motor_pins[i];
        link.completed_frames = now_us / 1000;
        link.tx_errors = now_us / 1000000;
    }
    return result;
}
void Driver::statistics(DriverStatistics& result) const { result = statistics(); }
Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() {}
TelemetryCounters Link::telemetry_counters() const { return telemetry().counters; }
Telemetry Link::telemetry() const {
    Telemetry result;
    result.available = Config{}.signaling == Signaling::Bidirectional;
    // Deliberately nonzero lifetime totals: the profile must report window deltas.
    result.counters.replies = now_us / 2000;
    result.counters.no_response = now_us / 4000;
    result.counters.rx_timeouts = now_us / 8000;
    result.counters.empty_captures = now_us / 8000;
    result.counters.checksum_errors = now_us / 1000000;
    return result;
}
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
void vTaskDelete(TaskHandle_t) {}
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) { return 4096; }
BaseType_t xTaskCreate(void (*)(void*), const char*, unsigned, void*, unsigned, TaskHandle_t*) { return pdPASS; }
#if DSHOT_BENCH_FOUR_MOTORS && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
configRUN_TIME_COUNTER_TYPE xPortGetRunTimeCounterValue() { return now_us; }
configRUN_TIME_COUNTER_TYPE ulTaskGetIdleRunTimeCounterForCore(BaseType_t core) {
    return idle_counters.at(core);
}
void vTaskDelay(TickType_t ticks) {
    assert(ticks == pdMS_TO_TICKS(1000));
    ++cpu_delays;
    now_us += 1000000;
    for (std::size_t core = 0; core < idle_counters.size(); ++core)
        idle_counters[core] += idle_increments[core];
}
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
bool fail_task_snapshot = false;
bool churn_tasks = false;
unsigned task_snapshots = 0;
TaskHandle_t xTaskGetIdleTaskHandleForCore(BaseType_t core) {
    return reinterpret_cast<TaskHandle_t>(static_cast<std::uintptr_t>(core + 1));
}
UBaseType_t uxTaskGetSystemState(TaskStatus_t* tasks, UBaseType_t capacity,
                               configRUN_TIME_COUNTER_TYPE* total) {
    ++task_snapshots;
    if (fail_task_snapshot) return 0;
    assert(capacity >= 4);
    *total = now_us;
    auto set = [&](unsigned index, const char* name, unsigned percent_tenths, unsigned number) {
        tasks[index] = {reinterpret_cast<TaskHandle_t>(static_cast<std::uintptr_t>(index + 1)),
                        name, number, static_cast<configRUN_TIME_COUNTER_TYPE>(now_us * percent_tenths / 1000)};
    };
    set(0, "IDLE0", 998, 1);
    set(1, "IDLE1", stream ? 610 : 1000, 2);
    set(2, "dshot_bench", 2, 3);
    if (stream) set(3, churn_tasks && task_snapshots % 2 == 0 ? "new_task" : "dshot_tx", 390,
                    churn_tasks && task_snapshots % 2 == 0 ? 5 : 4);
    if (stream && churn_tasks && task_snapshots % 2 == 0) tasks[3].ulRunTimeCounter = 1000;
    return stream ? 4 : 3;
}
#endif
#endif
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
    assert(parse(profile_command,{"profile"}) == 0 && parsed.back().action == Action::Profile && parsed.back().payload == 3);
    assert(parse(profile_command,{"profile","1"}) == 0 && parsed.back().payload == 1);
    assert(parse(profile_command,{"profile","30"}) == 0 && parsed.back().payload == 30);
    assert(parse(profile_command,{"profile","basic"}) == 0 && parsed.back().payload == (3 | 0x8000));
    assert(parse(profile_command,{"profile","5","basic"}) == 0 && parsed.back().payload == (5 | 0x8000));
    for (const auto* invalid_seconds : {"0", "31", "-1", "1x", "", "999999999999999999999999"})
        assert(parse(profile_command,{"profile",invalid_seconds}) != 0);
    assert(parse(profile_command,{"profile","1","2"}) != 0);
    const auto cpu_commands = parsed.size();
    assert(cpu_command(2, nullptr) != 0);
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    // Different per-core loads, accumulated idle counters, and timer/counter rollover.
    now_us = std::numeric_limits<std::uint32_t>::max() - 500000;
    idle_counters.fill(std::numeric_limits<configRUN_TIME_COUNTER_TYPE>::max() - 500000);
    idle_increments = {750000, 250000};
    const auto cpu_output = capture_output([] { assert(cpu_command(1, nullptr) == 0); });
    assert(cpu_output.find("CPU0: 25.0% busy") != std::string::npos);
    assert(cpu_output.find("CPU1: 75.0% busy") != std::string::npos);
    assert(cpu_output.find("CPU average: 50.0% busy") != std::string::npos);
    idle_increments = {1001000, 0}; // Idle accounting skew must not report negative usage.
    const auto extremes = capture_output([] { assert(cpu_command(1, nullptr) == 0); });
    assert(extremes.find("CPU0: 0.0% busy") != std::string::npos);
    assert(extremes.find("CPU1: 100.0% busy") != std::string::npos);
    assert(extremes.find("CPU average: 50.0% busy") != std::string::npos);
    assert(cpu_delays == 2);
    now_us = 0;
#else
    assert(cpu_command(1, nullptr) != 0);
#endif
    assert(parsed.size() == cpu_commands);
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

    auto reset_profile_bench = [&]() {
        stream = false;
        payload = begins = starts = stops = closes = open_motors = 0;
        fail_begin_motor = -1;
        motor_payloads.fill(0);
        command_busy.fill(false);
        command_statuses.fill({});
        next_event = 0;
    };
    auto run_profile_events = [&]() {
        return capture_output([] { try { bench_worker(nullptr); } catch (const Done&) {} });
    };
    reset_profile_bench();
    events = {{100000000, true, {Action::Profile,3}, false, 0},
              {101000000, true, {Action::Profile,3}, false, 0},
              {103000000, false, {}, false, 0}};
    const auto unarmed_profile = run_profile_events();
#if DSHOT_BENCH_RUNTIME_PROFILE_ENABLED
    assert(unarmed_profile.find("profile: 3.000 s") != std::string::npos);
    assert(unarmed_profile.find("active outputs: none") != std::string::npos);
    assert(unarmed_profile.find("CPU0: 0.2% busy") != std::string::npos);
    assert(unarmed_profile.find("CPU1: 0.0% busy") != std::string::npos);
    assert(unarmed_profile.find("sample already running") != std::string::npos);
    assert(unarmed_profile.find("passes 0 (0.0/s)") != std::string::npos);
    assert(unarmed_profile.find("stack free B") != std::string::npos);
    assert(unarmed_profile.find("Bench stack minimum free after reporting: 4096 bytes") != std::string::npos);

    auto armed_profile_events = [&](bool basic = false) {
        reset_profile_bench();
        task_snapshots = 0;
        events = {{0, true, {Action::Arm}, true, 0},
                  {1, true, {Action::Profile,3}, true, 0}, // Arming must finish first.
                  {hold+1, false, {}, true, 0},
                  {hold+2, true, {Action::Profile,static_cast<std::uint16_t>(basic ? 0x8003 : 3)}, true, 0},
                  {hold+3000002, false, {}, true, 0}};
        return run_profile_events();
    };
    const auto armed_profile = armed_profile_events();
    assert(armed_profile.find("wait until armed") != std::string::npos);
    assert(armed_profile.find("profile: 3.000 s") != std::string::npos);
    assert(armed_profile.find("CPU1: 39.0% busy") != std::string::npos);
    assert(armed_profile.find("dshot_tx") != std::string::npos);
    assert(armed_profile.find("39.00%") != std::string::npos);
    assert(armed_profile.find("frames 3000 (1000.0/s) | TX errors 3") != std::string::npos);
    assert(armed_profile.find("passes 3000 (1000.0/s) | missed slots 3") != std::string::npos);
    if (dshot::Config{}.signaling == dshot::Signaling::Bidirectional) {
        assert(armed_profile.find("replies 1500 | no response 750 (timeouts 375, empty/idle 375)") != std::string::npos);
        assert(armed_profile.find("CRC 3") != std::string::npos);
    }
    assert(task_snapshots == 2);
    assert(!stage_profile_requested);
#if CONFIG_DSHOT_STAGE_PROFILING
    assert(armed_profile.find("TX wait") != std::string::npos);
    assert(armed_profile.find("40.00") != std::string::npos);
    assert(armed_profile.find("4.00") != std::string::npos);
    assert(armed_profile.find("1.60%") != std::string::npos);
#else
    assert(armed_profile.find("Stage profile: off") != std::string::npos);
#endif
    const auto basic_profile = armed_profile_events(true);
    assert(basic_profile.find("Stage profile: off") != std::string::npos);
    assert(basic_profile.find("TX wait") == std::string::npos);
    assert(basic_profile.find("dshot_tx") != std::string::npos);
    assert(!stage_profile_requested);
    assert(cpu_delays == 2); // Profiling never blocks the bench task with vTaskDelay.

    churn_tasks = true;
    const auto churn_profile = armed_profile_events();
    assert(churn_profile.find("new_task") != std::string::npos);
    assert(churn_profile.find("(new)") != std::string::npos);
    assert(churn_profile.find("exited; final runtime unavailable") != std::string::npos);
    churn_tasks = false;

    fail_task_snapshot = true;
    const auto failed_profile = armed_profile_events();
    assert(failed_profile.find("task snapshot failed") != std::string::npos);
    assert(failed_profile.find("profile: 3.000 s") == std::string::npos);
    fail_task_snapshot = false;

    reset_profile_bench();
    events = {{0, true, {Action::Arm}, true, 0},
              {hold+1, false, {}, true, 0},
              {hold+2, true, {Action::Run,100}, true, 100},
              {hold+3, true, {Action::Profile,3}, true, 100},
              {hold+10003, true, {Action::Stop}, true, 0},
              {hold+3000003, false, {}, true, 0}};
    const auto cancelled_profile = run_profile_events();
    assert(cancelled_profile.find("profile cancelled: workload changed") != std::string::npos);
    assert(cancelled_profile.find("stopped") != std::string::npos);
    assert(cancelled_profile.find("profile: 3.000 s") == std::string::npos);
    assert(cpu_delays == 2);
    assert(!stage_profile_requested);
    std::puts("PASS: shared-window profile deltas, task accounting, unarmed/armed states, task churn, snapshot failure and responsive stop");
#else
    assert(unarmed_profile.find("profile: enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS") != std::string::npos);
#endif
#endif
}
