# Measuring the DShot baseline

The bench's `stats` command reads the driver's counters while transmission continues.
It never starts or stops a link. Nothing is printed automatically for profiling.

```text
arm
stats
run 100
stats
stop
stats
```

Use a raw run value appropriate for your ESC setup. You can also collect the entire
baseline while sending zero frames.

## Reading the output

- **Frames:** successful RMT transmission completions, accumulated per GPIO.
  Submitting a packet or changing the pending payload does not increment this.
- **TX errors:** failed transmission attempts, including a completion timeout.
  A failed attempt is not also counted as a completed frame.
- **Command pauses:** scheduler slots intentionally left idle between repetitions
  of a command. They are not TX errors or missed timer alarms. Completion rate
  temporarily drops during these gaps; other links continue normally.
- **Frames/s:** completions since the previous `stats` command divided by elapsed
  time over that interval. The first reading uses the time since driver start.
- **Total/interval seconds:** elapsed driver-session time and the interval used
  for the displayed rate. Session time freezes when the worker exits. A second
  reading after it has stopped has no new interval and prints `rate --`.
- **Timer:** configured hardware alarm period in microseconds. A requested
  1000/2000/4000/8000 Hz produces 1000/500/250/125 us respectively. Other rates
  round to the nearest microsecond (for example, 3333 Hz uses 300 us).
- **Missed slots:** delivered timer alarms the worker skipped. If several alarms
  accumulate before it wakes, it sends one frame and counts the others. Alarms
  accumulated during a transmission pass are discarded before waiting for a
  fresh alarm. This is a driver-wide counter, not one counter per GPIO. Timer
  interrupts coalesced while interrupt service is blocked cannot be counted
  individually, so this is not a count of every possible physical missed period.

Counters reset when `Driver::start()` attempts a new worker launch. They are
retained after worker exit, a TX fault, and link teardown, so a failed session
can still be inspected. An ordinary bench `stop` keeps sending zero, so it does
not reset or freeze the counters. Repeating `arm` on an already running stream
does not reset them either.

The driver currently transmits links sequentially. A snapshot may therefore
show one more completion on an earlier link than on a later one.

## Detailed timing

Enable **Component config → DShot → Compile detailed timing instrumentation**
(`CONFIG_DSHOT_TIMING_DIAGNOSTICS`) and rebuild to add these measurements:

| Measurement | Interval measured |
|---|---|
| Per-link outside TX wait | Snapshot/encoding/submission/return handling, excluding the time inside `rmt_tx_wait_all_done()` |
| Per-link TX wait | Elapsed time inside `rmt_tx_wait_all_done()`; a failure before reaching the wait contributes a zero sample |
| Per-link completion interval | Time between the worker's observations of consecutive successful completions |
| Worker pass | One pass through the attached links, including TX waits and per-link statistics updates |
| Worker outside TX waits | Worker-pass elapsed time minus the sum of that pass's measured TX waits |

For each measurement, `stats` prints minimum, average, maximum, and sample count
since the current session started. The timing summaries are cumulative even
though the displayed frames/s uses the interval since the previous reading.
The first successful completion has no preceding interval and contributes no
completion-interval sample. Worker-pass samples include a pass interrupted by
stop or an error.

These are elapsed microseconds from `esp_timer_get_time()`, including time when
another task or interrupt preempts the worker. They are not CPU utilization.
Completion observations are not the actual first or last GPIO edges. Use the
scope to measure waveform cadence and pulse widths; these statistics explain
what the software was doing around those transmissions.

Detailed timing adds timestamp reads and accounting. Compare enabled/disabled
builds when measuring its overhead. With it disabled, basic counters remain
available and no per-frame microsecond timestamps are collected. Time is still
read at session start/end and when requesting a statistics snapshot.

## Component API

`Driver::statistics()` returns a `dshot::DriverStatistics` value. Its `links`
array contains `link_count` entries, identified by GPIO. Unused entries have
GPIO -1. `detailed_timing` indicates whether timing collection was compiled in;
an empty timing measurement has `count == 0`.

The snapshot uses a short critical section shared with the worker, so its
64-bit values can be read coherently on both supported 32-bit chips. As with
the other public APIs, serialize calls with application lifecycle operations.
There is no allocation or printing in the measurement path.

## Collecting a baseline

1. Record the board, ESC firmware, GPIO, protocol, requested frame rate, RTOS
   tick rate, task settings, and whether detailed timing is enabled.
2. Start the stream and let arming finish. Enter `stats` to establish an interval
   baseline, wait about ten seconds, then enter `stats` again.
3. Repeat while running the motor and while applying a repeatable background
   workload. Record that workload with the results.
4. Compare completion rate and missed slots. Use the scope for actual frame
   spacing, and detailed timings to investigate waits and software delays.
5. Repeat the same setup after changing the scheduler.

## Testing the GPTimer scheduler

The earlier tick-based hardware baseline was 1000 completed frames/s with zero
TX errors and one missed slot over approximately 27 seconds on the user's C3
bench. The missed-slot definition now follows delivered timer alarms rather than
RTOS tick deadlines, so compare achieved rate and workload as well as this counter.

1. Rebuild at 1000 frames/s and confirm the same motor behavior and completion rate.
2. In menuconfig, set **Component config → DShot → Default frames per second** to
   2000, rebuild/flash, and compare two `stats` readings several seconds apart.
3. Repeat at 4000. Investigate 8000 only after collecting those results. The worker
   still services links sequentially, so extra links consume more of each period.
4. To verify tick independence on hardware, set **Component config → FreeRTOS →
   Kernel → configTICK_RATE_HZ** to 100, keeping DShot at 4000, then rebuild/flash.
   The timer period remains 250 us. Menu placement can vary; searching for
   `FREERTOS_HZ` locates the setting.

The host tests exercise counter/reset/fault behavior, timer setup rollback,
notifications at a simulated 100 Hz RTOS tick, discarded alarm backlogs, prompt
stop without another alarm, both instrumentation builds, and bench command
dispatch. They do not measure ESP performance.
