#include <cassert>
#include <initializer_list>
#include "command_sequence.hpp"
using namespace dshot;
using namespace dshot::detail;

void run_command_sequence_tests() {
    for (const auto id : {0,11,15,30,31,36,48,255}) {
        assert(command_info(static_cast<Command>(id)) == nullptr);
    }
    for (const auto command : {Command::DirectionNormal, Command::DirectionReversed,
                               Command::SaveSettings, Command::ExtendedTelemetryEnable,
                               Command::ExtendedTelemetryDisable}) {
        CommandSequence sequence;
        assert(sequence.begin(command, 0) == ESP_OK);
        const auto preamble = sequence.select(0, 0);
        assert(preamble.send && preamble.body == 0);
        assert(sequence.status().state == CommandState::Pending);
        sequence.complete(preamble, ESP_OK, 50);
        assert(sequence.select(0,10049).body == 0);
        std::int64_t now = 10050;
        for (unsigned i = 0; i < 6; ++i) {
            const auto frame = sequence.select(0,now);
            assert(frame.send && frame.command);
            assert(frame.body == ((static_cast<unsigned>(command) << 1) | 1u));
            assert(sequence.status().completed_repetitions == i);
            sequence.complete(frame, ESP_OK, now + 50);
            assert(sequence.status().completed_repetitions == i + 1);
            if (i != 5) assert(!sequence.select(0,now + 1049).send);
            now += 1050;
        }
        const auto status = sequence.status();
        assert(status.state == CommandState::Cooldown && status.total_repetitions == 6);
        const auto end = status.last_completed_us + (command == Command::SaveSettings ? 35000 : 1000);
        assert(sequence.select(0,end-1).body == 0);
        assert(sequence.begin(Command::Beep1,end-1) == ESP_ERR_INVALID_STATE);
        sequence.select(0,end);
        assert(sequence.status().state == CommandState::Complete);
        assert(!sequence.busy(end));
    }

    CommandSequence beep;
    assert(beep.begin(Command::Beep3,0) == ESP_OK);
    auto frame = beep.select(0,0);
    beep.complete(frame,ESP_OK,100);
    frame = beep.select(0,10100);
    assert(frame.command && frame.body == 7);
    beep.complete(frame,ESP_OK,10200);
    assert(beep.status().completed_repetitions == 1);
    beep.select(0,270199);
    assert(beep.status().state == CommandState::Cooldown);
    beep.select(0,270200);
    assert(beep.status().state == CommandState::Complete);

    CommandSequence cancelled;
    cancelled.begin(Command::Beep1,0);
    frame = cancelled.select(0,0); cancelled.complete(frame,ESP_OK,0);
    frame = cancelled.select(0,10000);
    cancelled.cancel();
    assert(cancelled.begin(Command::Direction1,10001) == ESP_ERR_INVALID_STATE);
    cancelled.complete(frame,ESP_OK,10050);
    assert(cancelled.status().state == CommandState::Cancelled);
    assert(cancelled.status().completed_repetitions == 1); // The in-flight frame did finish.
    assert(cancelled.busy(270049) && !cancelled.busy(270050));
    assert(cancelled.begin(Command::Direction1,270050) == ESP_OK);
    const auto new_id = cancelled.status().id;
    cancelled.complete(frame,ESP_OK,270100); // Stale completion cannot advance the new request.
    assert(cancelled.status().id == new_id && cancelled.status().completed_repetitions == 0);

    for (const bool fail_preamble : {false,true}) {
        CommandSequence failed;
        failed.begin(Command::SaveSettings,0);
        frame = failed.select(0,0);
        if (!fail_preamble) {
            failed.complete(frame,ESP_OK,0);
            frame = failed.select(0,10000);
        }
        failed.complete(frame,ESP_ERR_TIMEOUT,10100);
        assert(failed.status().state == CommandState::Failed);
        assert(failed.status().error == ESP_ERR_TIMEOUT && failed.status().completed_repetitions == 0);
    }
    CommandSequence info;
    info.begin(Command::EscInfo,0);
    frame = info.select(0,0); info.complete(frame,ESP_OK,0);
    frame = info.select(0,10000); info.complete(frame,ESP_OK,10050);
    assert(!info.select(0,22049).send);
    assert(info.select(0,22050).command);
}
