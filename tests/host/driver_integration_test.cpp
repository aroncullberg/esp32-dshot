#include <cassert>
#include <cstdio>
#include <future>
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "dshot/passthrough.hpp"
#include "config_validation.hpp"
#include "idf_transport.hpp"
#include "host_support.hpp"
#include "telemetry_fixture.hpp"
#include "soc/gpio_struct.h"

using namespace std::chrono_literals;
using namespace dshot;
using namespace dshot::detail;
using namespace host_test;

void test_config_and_transport() {
    auto c = config();
    assert(validate_config(c) == ESP_OK);
    c.startup_low_ms = 1001;
    assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    c.startup_low_ms = 0;
    assert(c.motor_poles == CONFIG_DSHOT_DEFAULT_MOTOR_POLES);
    for (const auto poles : {0u,1u,3u,13u,255u,256u}) {
        c.motor_poles = poles;
        assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    }
    for (const auto poles : {2u,12u,14u,254u}) {
        c.motor_poles = poles;
        assert(validate_config(c) == ESP_OK);
    }
    c = config();
    for (const auto hz : {100u,1000u,2000u,3333u,4000u,8000u}) {
        c.frame_hz = hz; assert(validate_config(c) == ESP_OK);
    }
    c.frame_hz = 8001; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
    c.frame_hz = 0; assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    c = config(); c.task_core = configNUMBER_OF_CORES; assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    c = config(); c.signaling = Signaling::Bidirectional; assert(validate_config(c) == ESP_OK);
    c.frame_hz = 8000; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
    c.frame_hz = 4000; assert(validate_config(c) == ESP_OK);
    c.protocol = Protocol::DShot150; assert(validate_config(c) == ESP_ERR_NOT_SUPPORTED);
    c.protocol = Protocol::DShot600;
    c.signaling = static_cast<Signaling>(42); assert(validate_config(c) == ESP_ERR_INVALID_ARG);
    for (auto* failure : {&fail_channel,&fail_encoder,&fail_enable}) {
        RmtTransport transport;
        failure->store(true);
        assert(open_transport(transport,GPIO_NUM_2,config()) != ESP_OK);
        assert(channels == 0 && encoders == 0);
        assert(close_transport(transport) == ESP_OK);
        assert(open_transport(transport,GPIO_NUM_2,config()) == ESP_OK);
        assert(open_transport(transport,GPIO_NUM_2,config()) == ESP_ERR_INVALID_STATE);
        fail_wait = true;
        assert(send_packet(transport,0) == ESP_ERR_TIMEOUT);
        assert(!transport.in_flight && !transport.enabled && transport.faulted);
        assert(send_packet(transport,0) == ESP_ERR_INVALID_STATE);
        assert(close_transport(transport) == ESP_OK);
        assert(close_transport(transport) == ESP_OK);
    }
    clean();
}

void test_startup_low_hold() {
    auto c = config();
    c.signaling = Signaling::Bidirectional;
    c.startup_low_ms = 300;
    const auto before = startup_pulses;
    RmtTransport t;
    assert(open_transport(t, GPIO_NUM_2, c) == ESP_OK);
    assert(startup_pulses == before + 1 && startup_delay_ticks == pdMS_TO_TICKS(300) + 1);
    assert(GPIO.rmt_routed[GPIO_NUM_2]); // GPIO setup must precede RMT routing.
    assert(send_packet(t, 0) == ESP_OK && t.reply.result == ReplyResult::Valid);
    close_transport_or_abort(t);
    c.startup_low_ms = 0;
    assert(open_transport(t, GPIO_NUM_2, c) == ESP_OK && startup_pulses == before + 1);
    close_transport_or_abort(t);
    c.startup_low_ms = 300;
    for (auto* failure : {&fail_startup_level, &fail_startup_output}) {
        *failure = true;
        assert(open_transport(t, GPIO_NUM_2, c) == ESP_FAIL);
        assert(!pad_driven(GPIO_NUM_2) && !t.opened && channels == 0);
    }
    assert(startup_pulses == before + 1); // Failed setup must not wait or allocate RMT.
    clean();
    std::puts("PASS: optional startup low hold, RMT route preservation, disabled hold and GPIO setup rollback");
}

void test_lifecycle() {
    Driver driver(config());
    assert(!driver.statistics().running && driver.statistics().elapsed_us == 0);
    Link link(driver,GPIO_NUM_2);
    Link duplicate(driver,GPIO_NUM_2);
    assert(driver.start() == ESP_ERR_INVALID_STATE);
    driver.stop(); link.end();
    assert(link.set(48) == ESP_ERR_INVALID_STATE);
    for (unsigned cycle = 0; cycle < 5; ++cycle) {
        assert(link.begin() == ESP_OK);
        assert(link.begin() == ESP_ERR_INVALID_STATE);
        assert(duplicate.begin() == ESP_ERR_INVALID_STATE);
        assert(link.set(48) == ESP_OK);
        assert(link.set(2048) == ESP_ERR_INVALID_ARG);
        fail_task = true;
        assert(driver.start() == ESP_ERR_NO_MEM);
        driver.stop();
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = true; entered_wait = false; wire_packets.clear();
        }
        assert(driver.start() == ESP_OK);
        assert(driver.start() == ESP_ERR_INVALID_STATE);
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        auto pending = driver.statistics();
        assert(pending.running && pending.link_count == 1);
        assert(pending.links[0].gpio == GPIO_NUM_2);
        assert(pending.links[0].completed_frames == 0); // Submission is not completion.
        // Pending requests must leave the active RMT buffer unchanged.
        assert(link.set(0) == ESP_OK);
        auto stopping = std::async(std::launch::async,[&] { driver.stop(); });
        assert(stopping.wait_for(5ms) == std::future_status::timeout);
        {
            std::lock_guard lock(wire_mutex); hold_wait = false;
        }
        wire_changed.notify_all();
        assert(stopping.wait_for(2s) == std::future_status::ready);
        stopping.get();
        const auto stopped = driver.statistics();
        assert(!stopped.running && stopped.links[0].completed_frames == 1);
        assert(stopped.links[0].tx_errors == 0 && stopped.elapsed_us >= 5000);
        std::this_thread::sleep_for(1ms);
        assert(driver.statistics().elapsed_us == stopped.elapsed_us);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
        assert(stopped.detailed_timing && stopped.links[0].tx_wait.count == 1);
        assert(stopped.links[0].tx_wait.total_us >= 5000);
        assert(stopped.worker_pass.total_us == stopped.outside_tx_wait.total_us +
               stopped.links[0].tx_wait.total_us);
        assert(stopped.links[0].completion_interval.count == 0);
#else
        assert(!stopped.detailed_timing && stopped.worker_pass.count == 0);
#endif
        {
            std::lock_guard lock(wire_mutex);
            assert(wire_packets.size() == 1 && wire_packets.front() == 0x0606);
        }
        driver.stop();
        assert(driver.start() == ESP_OK);
        link.end(); link.end();
        assert(link.set(0) == ESP_ERR_INVALID_STATE);
        clean();
    }
    assert(link.begin() == ESP_OK);
    fail_submit = true;
    assert(driver.start() == ESP_OK);
    eventually([&] { return driver.last_error() == ESP_FAIL; });
    driver.stop(); link.end(); clean();
    const auto faulted = driver.statistics();
    assert(!faulted.running && faulted.links[0].tx_errors == 1);
    assert(faulted.links[0].completed_frames == 0);
    assert(link.begin() == ESP_OK);
    fail_wait = true;
    assert(driver.start() == ESP_OK);
    eventually([&] { return driver.last_error() == ESP_ERR_TIMEOUT; });
    driver.stop(); link.end(); clean();
    assert(driver.statistics().links[0].tx_errors == 1);
    assert(driver.statistics().links[0].completed_frames == 0);
    assert(link.begin() == ESP_OK);
    assert(driver.start() == ESP_OK);
    assert(driver.last_error() == ESP_OK);
    link.end(); clean();
    Link second(driver,GPIO_NUM_3);
    assert(link.begin() == ESP_OK && second.begin() == ESP_OK);
    {
        std::lock_guard lock(wire_mutex);
        hold_wait = true; entered_wait = false;
    }
    assert(driver.start() == ESP_OK);
    eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
    std::this_thread::sleep_for(10ms); // Keep TX blocked long enough to miss several frame slots.
    {
        std::lock_guard lock(wire_mutex);
        hold_wait = false;
    }
    wire_changed.notify_all();
    eventually([&] {
        const auto snapshot = driver.statistics();
        return snapshot.missed_slots >= 5 && snapshot.links[1].completed_frames >= 2;
    });
    const auto multiple = driver.statistics();
    assert(multiple.link_count == 2 && multiple.links[1].gpio == GPIO_NUM_3);
    assert(multiple.links[0].completed_frames >= multiple.links[1].completed_frames);
    assert(multiple.links[0].completed_frames <= multiple.links[1].completed_frames + 1);
    assert(multiple.links[0].tx_errors == 0 && multiple.links[1].tx_errors == 0);
#if CONFIG_DSHOT_TIMING_DIAGNOSTICS
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& stats = multiple.links[i];
        assert(stats.completion_interval.count == stats.completed_frames - 1);
        assert(stats.tx_wait.count == stats.completed_frames);
        assert(stats.tx_wait.min_us <= stats.tx_wait.max_us);
    }
#endif
    Link late(driver,GPIO_NUM_4);
    assert(late.begin() == ESP_ERR_INVALID_STATE);
    second.end(); link.end(); clean();
}

void test_command_integration() {
    manual_timer = true;
    command_clock_us = 1000000;
    {
        auto c = config(); c.frame_hz = 4000;
        Driver driver(c);
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        assert(link.set(100) == ESP_OK);
        assert(link.send_command(Command::DirectionNormal) == ESP_ERR_INVALID_STATE);
        assert(link.set(0) == ESP_OK);
        assert(link.send_command(Command::Beep1) == ESP_OK);
        driver.stop();
        assert(link.command_status().state == CommandState::Cancelled);
        assert(driver.start() == ESP_OK);
        eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
        auto step = [&](std::int64_t now) {
            command_clock_us = now;
            const auto before = driver.statistics().worker_iterations;
            emit_alarm(active_timer.load());
            eventually([&] { return driver.statistics().worker_iterations > before && tasks.back()->waiting.load(); });
        };
        {
            std::lock_guard lock(wire_mutex); wire_packets.clear();
        }
        assert(link.send_command(Command::DirectionNormal) == ESP_OK);
        assert(link.set(200) == ESP_ERR_INVALID_STATE);
        step(1000000);
        step(1009999);
        step(1010000);
        assert(link.command_status().completed_repetitions == 1);
        const auto completed = driver.statistics().links[0].completed_frames;
        step(1010250);
        assert(driver.statistics().links[0].completed_frames == completed);
        assert(driver.statistics().links[0].command_pauses == 1);
        for (unsigned i = 1; i < 10; ++i) step(1010000 + i*1000);
        step(1019999);
        assert(link.command_status().state == CommandState::Cooldown);
        step(1020000);
        assert(link.command_status().state == CommandState::Complete);
        {
            std::lock_guard lock(wire_mutex);
            assert(wire_packets.size() == 14);
            for (unsigned i = 0; i < 14; ++i) {
                assert(wire_packets[i] == (i >= 2 && i < 12 ? 0x029B : 0));
            }
        }
        assert(link.send_command(Command::Beep1) == ESP_OK);
        step(1020000);
        {
            std::lock_guard lock(wire_mutex); hold_wait = true; entered_wait = false;
        }
        command_clock_us = 1030000;
        emit_alarm(active_timer.load());
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        assert(link.command_status().completed_repetitions == 0);
        assert(link.set(0) == ESP_OK); // Cancel during RMT's active frame.
        assert(link.send_command(Command::Direction2) == ESP_ERR_INVALID_STATE);
        {
            std::lock_guard lock(wire_mutex); hold_wait = false;
        }
        wire_changed.notify_all();
        eventually([] { return tasks.back()->waiting.load(); });
        assert(link.command_status().state == CommandState::Cancelled);
        assert(link.command_status().completed_repetitions == 1);
        assert(link.set(100) == ESP_ERR_INVALID_STATE); // Already-sent beep retains its cooldown.
        command_clock_us = 1290001;
        assert(link.set(100) == ESP_OK);
        assert(link.set(0) == ESP_OK);
        assert(link.send_command(Command::SaveSettings) == ESP_OK);
        step(1300000);
        fail_wait = true;
        command_clock_us = 1310000;
        emit_alarm(active_timer.load());
        eventually([&] { return driver.last_error() == ESP_ERR_TIMEOUT; });
        driver.stop();
        assert(link.command_status().state == CommandState::Failed);
        assert(link.command_status().completed_repetitions == 0);
        link.end();
        assert(link.command_status().state == CommandState::Failed);
        clean();
        assert(link.begin() == ESP_OK && driver.start() == ESP_OK);
        assert(link.send_command(Command::Direction2) == ESP_OK);
        driver.stop();
        assert(link.command_status().state == CommandState::Cancelled);
        link.end(); clean();
    }
    {
        Driver driver(config());
        Link command_link(driver,GPIO_NUM_2), other(driver,GPIO_NUM_3);
        assert(command_link.begin() == ESP_OK && other.begin() == ESP_OK);
        assert(other.set(100) == ESP_OK);
        assert(command_link.send_command(Command::Direction1) == ESP_OK);
        assert(driver.start() == ESP_OK);
        eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
        auto step = [&](std::int64_t now) {
            command_clock_us = now;
            const auto before = driver.statistics().worker_iterations;
            emit_alarm(active_timer.load());
            eventually([&] { return driver.statistics().worker_iterations > before && tasks.back()->waiting.load(); });
        };
        step(2000000);
        step(2010000); // First command frame.
        const auto before = driver.statistics();
        step(2010250); // Command gap must not pause the other link.
        const auto after = driver.statistics();
        assert(after.links[0].completed_frames == before.links[0].completed_frames);
        assert(after.links[0].command_pauses == 1);
        assert(after.links[1].completed_frames == before.links[1].completed_frames + 1);
        assert(after.links[1].command_pauses == 0);
        driver.stop(); command_link.end(); other.end(); clean();
    }
    command_clock_us = -1;
    manual_timer = false;
}

void test_scheduler() {
    for (auto* failure : {&fail_timer_new,&fail_timer_register,&fail_timer_alarm,
                          &fail_timer_enable,&fail_timer_enable_partial}) {
        Driver driver(config());
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        failure->store(true);
        assert(driver.start() != ESP_OK && timers == 0);
        assert(driver.start() == ESP_OK);
        link.end(); clean();
    }
    {
        Driver driver(config());
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK);
        fail_timer_start = true;
        assert(driver.start() == ESP_OK);
        eventually([&] { return driver.last_error() == ESP_FAIL; });
        driver.stop();
        assert(timers == 0 && driver.statistics().links[0].completed_frames == 0);
        assert(driver.start() == ESP_OK);
        link.end(); clean();
    }

    // Trigger alarms explicitly to avoid host scheduling delays.
    manual_timer = true;
    for (const auto hz : {1000u,2000u,4000u,8000u,3333u,1u}) {
        auto c = config(); c.frame_hz = hz;
        Driver driver(c);
        Link link(driver,GPIO_NUM_2);
        assert(link.begin() == ESP_OK && driver.start() == ESP_OK);
        eventually([] { return active_timer.load() != nullptr; });
        auto* timer = active_timer.load();
        assert(timer->period_us == (1000000u + hz/2) / hz);
        assert(driver.statistics().timer_period_us == timer->period_us);
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = true; entered_wait = false;
        }
        emit_alarm(timer);
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        for (unsigned i = 0; i < 7; ++i) emit_alarm(timer);
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = false;
        }
        wire_changed.notify_all();
        eventually([&] { return driver.statistics().missed_slots == 7; });
        std::this_thread::sleep_for(2ms);
        assert(driver.statistics().links[0].completed_frames == 1);
        emit_alarm(timer);
        eventually([&] { return driver.statistics().links[0].completed_frames == 2; });
        // Stop must wake the worker without another alarm.
        auto stopped = std::async(std::launch::async,[&] { driver.stop(); });
        assert(stopped.wait_for(200ms) == std::future_status::ready);
        stopped.get();
        link.end(); clean();
    }
    manual_timer = false;
}

void test_passthrough_callbacks() {
    esp_err_t (*acquire)(void*) = Passthrough::acquire_callback;
    esp_err_t (*release)(void*, bool) = Passthrough::release_callback;
    assert(acquire(nullptr) == ESP_ERR_INVALID_ARG);
    assert(release(nullptr, true) == ESP_ERR_INVALID_ARG);
    manual_timer = true;
#if CONFIG_IDF_TARGET_ESP32S3
    constexpr std::size_t count = 4;
#else
    constexpr std::size_t count = 2;
#endif
    for (auto signaling : {Signaling::Regular, Signaling::Bidirectional}) {
        auto settings = config(); settings.signaling = signaling;
        Driver driver(settings);
        Link first(driver, GPIO_NUM_2), second(driver, GPIO_NUM_3);
        Link third(driver, GPIO_NUM_4), fourth(driver, GPIO_NUM_5);
        Link* links[] = {&first, &second, &third, &fourth};
        Passthrough handoff(driver, links, count);
        Passthrough competing(driver, links, count);
        for (std::size_t i = 0; i < count; ++i) {
            assert(links[i]->begin() == ESP_OK);
            assert(links[i]->set(100 + i) == ESP_OK);
        }
        assert(driver.start() == ESP_OK);
        eventually([] { return active_timer.load() != nullptr; });
        {
            std::lock_guard lock(wire_mutex);
            hold_wait = true; entered_wait = false; wire_packets.clear();
        }
        emit_alarm(active_timer.load());
        eventually([] { std::lock_guard lock(wire_mutex); return entered_wait; });
        auto acquiring = std::async(std::launch::async, [&] { return acquire(&handoff); });
        assert(acquiring.wait_for(5ms) == std::future_status::timeout);
        assert(channels > 0); // Passthrough must wait for the active transmission to finish.
        { std::lock_guard lock(wire_mutex); hold_wait = false; }
        wire_changed.notify_all();
        assert(acquiring.get() == ESP_OK && handoff.held());
        clean();
        assert(driver.start() == ESP_ERR_INVALID_STATE);
        assert(first.begin() == ESP_ERR_INVALID_STATE);
        assert(first.set(0) == ESP_ERR_INVALID_STATE);
        assert(first.send_command(Command::Beep1) == ESP_ERR_INVALID_STATE);
        assert(acquire(&competing) == ESP_ERR_INVALID_STATE);
        assert(acquire(&handoff) == ESP_OK && channels == 0);
        assert(!pad_driven(GPIO_NUM_2) && !pad_driven(GPIO_NUM_3));
        { std::lock_guard lock(wire_mutex); wire_packets.clear(); }
        assert(release(&handoff, true) == ESP_OK && !handoff.held());
        assert(release(&handoff, true) == ESP_OK);
        eventually([] { return active_timer.load() != nullptr; });
        emit_alarm(active_timer.load());
        eventually([&] { return driver.statistics().links[count - 1].completed_frames == 1; });
        {
            std::lock_guard lock(wire_mutex);
            assert(wire_packets.size() == count);
            for (auto packet : wire_packets) assert(packet == (signaling == Signaling::Regular ? 0 : 0xF));
        }
        assert(first.send_command(Command::Beep1) == ESP_OK);
        if (signaling == Signaling::Bidirectional) assert(second.request_edt(true) == ESP_OK);
        assert(acquire(&handoff) == ESP_OK);
        assert(first.command_status().state == CommandState::Cancelled);
        if (signaling == Signaling::Bidirectional) assert(second.telemetry().edt.state == EdtState::Cancelled);
        assert(release(&handoff, false) == ESP_OK); // An interrupted passthrough session leaves the driver stopped.
        assert(!driver.statistics().running && channels == 0 && first.set(50) == ESP_ERR_INVALID_STATE);
        assert(release(&handoff, true) == ESP_OK && channels == 0); // A later release must not restart an interrupted session.
        assert(acquire(&handoff) == ESP_OK && release(&handoff, true) == ESP_OK);
        assert(acquire(&handoff) == ESP_OK);
        fail_tx_channel_after = 1; // Second link fails after the first has reopened.
        assert(release(&handoff, true) == ESP_ERR_NO_MEM);
        assert(!handoff.held() && !driver.statistics().running);
        clean();
        assert(first.set(1) == ESP_ERR_INVALID_STATE && second.set(1) == ESP_ERR_INVALID_STATE);
        assert(acquire(&handoff) == ESP_OK);
        fail_task = true;
        assert(release(&handoff, true) == ESP_ERR_NO_MEM);
        clean();
        assert(acquire(&handoff) == ESP_OK && release(&handoff, true) == ESP_OK);
        assert(acquire(&handoff) == ESP_OK);
        assert(release(&handoff, false) == ESP_OK);
        clean();
    }
    {
        Driver driver(config()), other(config());
        Link first(driver, GPIO_NUM_2), second(driver, GPIO_NUM_3), foreign(other, GPIO_NUM_4);
        assert(first.begin() == ESP_OK && second.begin() == ESP_OK);
        Passthrough incomplete(driver, {&first});
        assert(incomplete.acquire() == ESP_ERR_INVALID_STATE && channels == 2);
        Passthrough duplicate(driver, {&first, &first});
        assert(duplicate.acquire() == ESP_ERR_INVALID_ARG && channels == 2);
        Passthrough wrong_driver(driver, {&first, &foreign});
        assert(wrong_driver.acquire() == ESP_ERR_INVALID_ARG && channels == 2);
        Passthrough empty(driver, {});
        assert(empty.acquire() == ESP_ERR_INVALID_ARG);
        Passthrough null_link(driver, {nullptr});
        assert(null_link.acquire() == ESP_ERR_INVALID_ARG);
        {
            Passthrough held(driver, {&first, &second});
            assert(held.acquire() == ESP_OK);
        } // Destroying a held passthrough allows the links to reopen.
        assert(first.begin() == ESP_OK);
        first.end(); clean();
    }
    manual_timer = false;
    std::puts("PASS: passthrough callbacks, full-group TX/RX handoff, zero restoration, request blocking and all-link rollback");
}

void test_bidirectional_transport() {
    auto c=config(); c.signaling=Signaling::Bidirectional; c.frame_hz=4000;
    for (auto* failure : {&fail_rx_new,&fail_rx_register,&fail_tx_register,&fail_rx_enable}) {
        RmtTransport t; *failure=true;
        assert(open_transport(t,GPIO_NUM_2,c) != ESP_OK);
        assert(channels == 0 && encoders == 0);
    }
    RmtTransport t;
    assert(open_transport(t,GPIO_NUM_2,c) == ESP_OK && channels == 2);
    reply_payload=0x3F4;
    assert(send_packet(t,0x0606) == ESP_OK);
    assert(wire_packets.back() == 0x0609); // Bidirectional DShot complements the checksum.
    assert(t.reply.result == ReplyResult::Valid && t.reply.payload == 0x3F4);
    assert(!pad_driven(GPIO_NUM_2));
    silence=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::NoResponse);
    assert(t.rx_enabled && !t.faulted && !t.rx_timed_out);
    assert(t.rx_count == 0); // Echo alone cannot become telemetry.
    stall_receive=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::NoResponse);
    assert(!t.rx_enabled && !t.faulted && t.rx_timed_out);
    stall_receive=false;
    silence=false; empty_reply=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::NoResponse);
    assert(t.rx_enabled && !t.rx_timed_out);
    empty_reply=false;
    silence=false; bad_reply_crc=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::ChecksumError);
    bad_reply_crc=false; rx_overflow=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::Overflow);
    rx_overflow=false; fail_receive=true;
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::ReceiveError);
    assert(send_packet(t,0) == ESP_OK && t.reply.result == ReplyResult::Valid);
    fail_wait=true;
    assert(send_packet(t,0) == ESP_ERR_TIMEOUT && !t.enabled && !t.rx_enabled);
    assert(!pad_driven(GPIO_NUM_2));
    close_transport_or_abort(t); clean();
    reply_payload=0xFFF;
    std::puts("PASS: bidirectional polarity, RX before TX, echo-only idle completion, stalled/corrupt/overflow RX recovery and TX abort");
}

void test_edt_link() {
    manual_timer=true; command_clock_us=1000000;
    auto c=config(); c.signaling=Signaling::Bidirectional; c.frame_hz=4000;
    c.edt_response_timeout_ms=50; c.edt_attempts=2;
    Driver driver(c); Link link(driver,GPIO_NUM_2);
    assert(link.begin() == ESP_OK && driver.start() == ESP_OK);
    eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
    auto step = [&](std::int64_t now, std::uint16_t reply) {
        command_clock_us=now; reply_payload=reply;
        const auto before=driver.statistics().worker_iterations;
        emit_alarm(active_timer.load());
        eventually([&] { return driver.statistics().worker_iterations > before && tasks.back()->waiting.load(); });
    };
    step(1000000,0xFFF);
    assert(link.telemetry().speed.state == ErpmState::Stopped);
    const auto capture = link.receive_capture();
    const auto expected = esc_reply(0xFFF,600000,false,0,true,false);
    assert(capture.result == ReceiveResult::Valid && capture.captured_at_us == 1000000);
    assert(capture.resolution_hz == 40000000 && capture.bitrate == 600000);
    assert(capture.pulse_count == expected.size()*2);
    for (unsigned i = 0; i < expected.size(); ++i) {
        assert(capture.pulses[2*i].ticks == expected[i].duration0);
        assert(capture.pulses[2*i].high == (expected[i].level0 != 0));
        assert(capture.pulses[2*i+1].ticks == expected[i].duration1);
        assert(capture.pulses[2*i+1].high == (expected[i].level1 != 0));
    }
    // Preserve the last good eRPM after a bad reply.
    bad_reply_crc = true;
    step(1000000,0xFFF);
    assert(link.receive_capture().result == ReceiveResult::ChecksumError);
    assert(link.telemetry().speed.state == ErpmState::Stopped);
    bad_reply_crc = false;
    silence = true;
    step(1000000,0xFFF);
    assert(link.receive_capture().result == ReceiveResult::ChecksumError);
    assert(link.telemetry().counters.rx_timeouts == 0 && link.telemetry().counters.empty_captures == 1);
    silence = false;
    empty_reply = true;
    step(1000000,0xFFF);
    assert(link.receive_capture().result == ReceiveResult::ChecksumError);
    assert(link.telemetry().counters.no_response == 2 && link.telemetry().counters.rx_timeouts == 0 &&
           link.telemetry().counters.empty_captures == 2);
    empty_reply = false;
    assert(link.request_edt(true) == ESP_OK);
    step(1000001,0xFFF); // Preamble.
    for (unsigned i=0; i<10; ++i) step(1010001+i*1200, i==6 ? 0xE00 : 0xFFF);
    assert(link.telemetry().edt.state == EdtState::Enabled);
    assert(link.command_status().completed_repetitions == 10);
    step(1023000,0x228);
    assert(link.command_status().state == CommandState::Complete);
    assert(link.telemetry().temperature_c.value == 40);
    assert(link.set(100) == ESP_OK);
    step(1024000,0x3F4);
    assert(link.telemetry().speed.erpm == 60000);
    assert(link.request_edt(false) == ESP_ERR_INVALID_STATE);
    assert(link.set(0) == ESP_OK);
    assert(link.request_edt(false) == ESP_OK);
    step(1025000,0xFFF);
    for (unsigned i=0; i<10; ++i) step(1035000+i*1200, i==6 ? 0xEFF : 0xFFF);
    step(1048000,0xFFF);
    assert(link.telemetry().edt.state == EdtState::Disabled);
    // Without an enable acknowledgement, stop after two attempts.
    assert(link.request_edt(true) == ESP_OK);
    step(1050000,0xFFF);
    for (unsigned i=0; i<10; ++i) step(1060000+i*1200,0xFFF);
    step(1073000,0xFFF);
    step(1110000,0xFFF); // Retry begins.
    assert(link.telemetry().edt.attempts == 2);
    for (unsigned i=0; i<10; ++i) step(1120000+i*1200,0xFFF);
    step(1133000,0xFFF);
    step(1170000,0xFFF);
    assert(link.telemetry().edt.state == EdtState::Timeout);
    // A run request prevents another EDT attempt.
    assert(link.request_edt(true) == ESP_OK);
    step(1180000,0xFFF);
    for (unsigned i=0; i<10; ++i) step(1190000+i*1200,0xFFF);
    step(1203000,0xFFF);
    assert(link.set(100) == ESP_OK);
    step(1240000,0x3F4);
    assert(link.telemetry().edt.state == EdtState::Timeout && link.telemetry().edt.attempts == 1);
    { std::lock_guard lock(wire_mutex); assert(wire_packets.back() == (0x0C84 ^ 0xF)); }
    assert(link.set(0) == ESP_OK);
    assert(link.request_edt(true) == ESP_OK);
    assert(link.set(0) == ESP_OK);
    assert(link.telemetry().edt.state == EdtState::Cancelled);
    assert(link.request_edt(true) == ESP_OK);
    driver.stop(); link.end();
    assert(link.telemetry().edt.state == EdtState::Cancelled);
    clean(); manual_timer=false; command_clock_us=-1; reply_payload=0xFFF;
    std::puts("PASS: link eRPM publication, EDT enable/disable acknowledgement, retries, timeout, cancellation and teardown");
}

void test_silent_bidirectional_group() {
    manual_timer = true;
    silence = true;
    auto c = config();
    c.signaling = Signaling::Bidirectional;
    Driver driver(c);
    Link first(driver,GPIO_NUM_2), second(driver,GPIO_NUM_3);
#if CONFIG_IDF_TARGET_ESP32S3
    Link third(driver,GPIO_NUM_4), fourth(driver,GPIO_NUM_5);
    const std::array<Link*,4> links{&first,&second,&third,&fourth};
#else
    const std::array<Link*,2> links{&first,&second};
#endif
    for (auto* link : links) assert(link->begin() == ESP_OK);
    assert(driver.start() == ESP_OK);
    eventually([] { return active_timer.load() && tasks.back()->waiting.load(); });
    for (unsigned pass = 1; pass <= 3; ++pass) {
        emit_alarm(active_timer.load());
        eventually([&] { return driver.statistics().worker_iterations == pass && tasks.back()->waiting.load(); });
        for (auto* link : links) {
            const auto telemetry = link->telemetry();
            assert(telemetry.counters.no_response == pass);
            assert(telemetry.counters.empty_captures == pass && telemetry.counters.rx_timeouts == 0);
            assert(telemetry.counters.replies == 0);
            assert(telemetry.speed.state == ErpmState::NoData);
        }
    }
    // Replies can begin on the next pass without restarting any output.
    silence = false;
    emit_alarm(active_timer.load());
    eventually([&] { return driver.statistics().worker_iterations == 4 && tasks.back()->waiting.load(); });
    for (auto* link : links) assert(link->telemetry().counters.replies == 1);
    const auto stats = driver.statistics();
    for (std::size_t i = 0; i < stats.link_count; ++i)
        assert(stats.links[i].completed_frames == 4 && stats.links[i].tx_errors == 0);
    driver.stop();
    for (auto* link : links) link->end();
    clean();
    manual_timer = false;
    std::puts("PASS: all silent outputs complete without RX timeouts and accept replies on the next pass");
}

int main() {
    test_config_and_transport(); std::puts("PASS: configuration, partial setup rollback, timeout abort and close");
    test_startup_low_hold();
    test_lifecycle(); std::puts("PASS: lifecycle, statistics reset/retention, multi-link counts and missed slots");
    test_passthrough_callbacks();
    test_scheduler(); std::puts("PASS: GPTimer rollback/restart, 1/2/4/8 kHz periods at 100 Hz RTOS tick, dropped backlog, stop wake-up");
    test_command_integration();
    test_bidirectional_transport();
    test_silent_bidirectional_group();
    test_edt_link();
    std::puts("PASS: command packets, completed repetitions, quiet gaps, cancellation, cooldowns and faults");
    clean();
}
