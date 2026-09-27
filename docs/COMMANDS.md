# ESC commands

Each link now sequences named DShot commands. The bench uses the same API;
command timing does not depend on the console task or RTOS tick.

## Bench usage

After `arm` has finished, return to zero with `stop`, then try `beep 1`.
`beep` without a number selects tone 1. Tones 1–5 are available.
Use `command status` to inspect progress. Wait for `complete` before another
command or a nonzero run value.

`direction normal` selects command 20; `direction reversed` selects 21.
After completion, use your usual `run <value>` to check direction.
The additional forms `direction 1` and `direction 2` select commands 7 and 8.

On AM32, 1/2 change the stored direction setting in RAM, while normal/reversed
select direction relative to that setting. To persist a direction change there,
use `direction 1` or `direction 2`, wait for completion, then issue `save`.
See [AM32 command handling](https://github.com/am32-firmware/AM32/blob/main/Src/dshot.c).

`command list` (or just `command`) lists all supported transmit names and IDs.
`command <name>` and `command <id>` use the same sequencer as the short commands.
Examples include `3d-on`, `3d-off`, `save`, and `esc-info`. Reserved/undefined IDs
are rejected. Payload zero remains `stop` / `run 0`.

The bench prints `stop first` if a nonzero raw payload is pending, and `command
busy` if another sequence or its cooldown is pending. It keeps the stream running.
`stop` cancels remaining repetitions and resumes zero frames. After completion,
zero frames continue until the next `run`.

## Component API

```cpp
// With an opened link and an already-armed ESC:
link.set(0);
esp_err_t result = link.send_command(dshot::Command::DirectionReversed);
// Inspect later from the application task:
dshot::CommandStatus progress = link.command_status();
```

`send_command()` accepts one sequence per link and requires its pending raw payload
be zero. It returns `ESP_ERR_INVALID_STATE` for an unopened link, nonzero pending
payload, or busy sequence/cooldown. Unsupported IDs return `ESP_ERR_INVALID_ARG`
when the link is otherwise ready for a command. A sequence can be queued before
`Driver::start()`; the application still owns ESC arming. Submission does not
transmit synchronously.

`set(0)` cancels a sequence. Nonzero `set()` returns `ESP_ERR_INVALID_STATE` until
the sequence and required cooldown finish. Raw `set()` retains its 0..2047 range;
use `send_command()` for managed repetitions/timing. An earlier nonzero payload
is not restored automatically.

`command_status()` reports a per-open-link request ID, state, completed/target
repetitions, completion timestamps, and an error. States are pending, preparing,
transmitting, cooldown, complete, cancelled, and failed. Complete means the local
sequence was transmitted and its cooldown elapsed; it is not an ESC acknowledgement.
The current regular transport has no receive path.

## Sequence behavior

1. Send zero, then continue zero for at least 10 ms from that first zero frame's
   successful completion.
2. Send the command with its telemetry-request bit set. Count the repetition only
   after RMT reports completion.
3. Leave the wire idle between repetitions. Do not insert zero/throttle frames:
   AM32 resets its repeat counter on zero. Resume on an eligible scheduler slot.
4. After the final repetition, send zero throughout the cooldown and afterward.

Most repeat intervals are 1 ms. Direction, 3D, save, and EDT enable/disable use
six repetitions. Beeps use one with a 260 ms cooldown; save has a 35 ms cooldown.
ESC info uses six requests for AM32 compatibility, spaced at least 12 ms apart,
and a 12 ms cooldown. Other ESCs may reply to each info request. See the
[DShot command reference](https://betaflight.com/docs/development/API/Dshot).

`src/command.cpp` owns these protocol parameters; they are not board configuration.
An ESC may implement only some listed commands. Telemetry commands, including EDT
enable/disable, can be transmitted now, but response capture, EDT negotiation,
and telemetry decoding remain later milestones. Firmware-specific programming
commands are not included.

Cancellation stops future repetitions. A frame already selected by the worker finishes and
is counted for the cancelled request; its required cooldown is retained. A new
command cannot replace an in-flight command frame. `Driver::stop()` cancels
pending sequences even before worker launch. TX faults fail the affected active
sequence and stop the driver; other pending sequences are cancelled. Final status
survives `Link::end()` and resets on a fresh successful `begin()`.

## Measurements and tests

`stats` includes **command pauses**, the scheduler slots intentionally left
without a transmission between repetitions. They are separate from missed timer
alarms and TX errors. Frame rate temporarily drops during those gaps; other links
continue to be serviced normally.

Offline tests cover packet values/telemetry bits, repetition counts, minimum
delays, failed completions, cancellation during a frame, stale request completions,
and console dispatch. Hardware command acceptance is the next bench check.
No command tests run at boot.
