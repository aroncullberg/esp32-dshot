# Four-motor DShot bench

An ESP32-S3 serial console for a 4-in-1 ESC. Defaults: GPIOs 18, 39, 38, 40; bidirectional DShot600 at 1000 Hz; 300 ms startup low hold.

## Build and run

With ESP-IDF 6.1 activated, run from this directory:

```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Set the pins under `DShot four-motor bench` and protocol settings under `Component config → DShot`. Restart the ESC after changing signaling mode.

## Console

All four motors are selected initially.

- `motor <1..4|all>`: select command targets.
- `arm`: open only the selected outputs and send zeros; wait for `armed`.
- `run <0..2047>`: hold the selected motors' raw DShot value.
- `stop`: set every open motor to zero and cancel queued commands.
- `telemetry`: show selected motors' speed and telemetry.
- `stats`: show per-output frame rates and errors.
- `cpu`: sample CPU usage for one second, then show each core's busy percentage and the average across cores. DShot continues running during the sample.
- `profile [seconds] [basic]`: collect a shared-window report (default 3 seconds, range 1–30). Shows core usage, each task's runtime, worker passes, missed slots, per-output frame rates/TX errors, bidirectional RX/error counters, and sampled stage timings. `basic` disables stage sampling for an overhead comparison.

CPU measurement uses FreeRTOS idle-task run-time counters, enabled in `sdkconfig.defaults` with the ESP Timer clock and 64-bit counters. If you already have an `sdkconfig`, enable `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`, `CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER`, and `CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64` in `menuconfig` and rebuild.

Other commands are the same as the [single-motor bench](../bench/README.md).

## Profiling comparisons

Bench4 defaults to performance optimization (`CONFIG_COMPILER_OPTIMIZATION_PERF`,
`-O2`). For an existing `sdkconfig`, select performance optimization in
`menuconfig` before comparing results. See [measured improvements](PROFILING_RESULTS.md).

After flashing, use `profile` before arming for the idle baseline. Then:

```text
motor all
arm
```

Wait for `armed`, then run `profile` (or `profile 5` for a five-second sample).

Wait for the report before entering another command. Sampling runs in the background and `stop` remains responsive. `arm`, `run`, `stop`, ESC commands, and EDT changes cancel an active sample. Profiling during arming is rejected. `stop` keeps sending zero frames; reboot to return to the unarmed baseline.

For one output, use `motor 1`, then `arm` and wait for `armed` before profiling. Selection alone does not close outputs. To compare regular/bidirectional mode or frame rates, change the DShot settings in `menuconfig`, rebuild, flash, and repeat. The report includes signaling mode, target rate, worker core, active outputs/raw values, and whether detailed timing instrumentation is enabled.

All rates, task runtimes, and error counts use differences between the same two snapshot boundaries, rather than lifetime totals. The actual interval and the time spent collecting each boundary are printed; task, driver, and RX snapshots are collected sequentially, so boundaries have that small skew. Task runtime is expressed as microseconds and as a percentage where 100% means one full core. Idle tasks are included in the table; core busy percentages are calculated from their runtime deltas. FreeRTOS attributes interrupt time to the interrupted task, so this report does not separate ISR cost.

`stack free B` is each task's minimum free stack since creation, measured in bytes by ESP-IDF. The report also prints the bench task's watermark after its main tables have been formatted. Bench state and profile snapshots live on the heap, and the build rejects individual bench functions with stack frames larger than 2 KiB to preserve headroom in the 8 KiB bench task stack.

Task names are copied at each boundary and tasks are matched by handle and task number. New/exited tasks are marked; final runtime of exited tasks is unavailable. A failed snapshot (including more than 32 tasks) aborts the sample. Profiling requires the ESP Timer run-time clock; the default 64-bit counters avoid the short rollover interval of 32-bit counters.

### Sampled stages

`CONFIG_DSHOT_STAGE_PROFILING` enables the stage profiler, independently of `CONFIG_DSHOT_TIMING_DIAGNOSTICS`. It is enabled in bench4 defaults and requires a pinned worker on multicore targets (bench4 defaults to core 1). For an existing `sdkconfig`, enable it under `Component config → DShot`; leave the older timing diagnostics disabled for these comparisons. `CONFIG_DSHOT_STAGE_PROFILE_INTERVAL` defaults to 16, sampling one complete four-motor update in 16. Stage timestamps are collected only when requested by `profile`; `profile basic` keeps them off. The equivalent C++ API is `Driver::set_stage_profiling(bool)`.

The stage table separates frame preparation, symbol encoding, RX setup, TX submission, TX wait, RX wait, RX completion handling, echo removal, reply decoding, telemetry updates, raw-capture copying, link cleanup, and bookkeeping. Rows aggregate all active motors. Counts and totals are differenced over the report window, just like frame/error counters.

- `visits`: measured entries into the stage; bookkeeping/cleanup can be entered several times per update.
- `elapsed us/v`: average elapsed microseconds per visit, including blocking and preemption.
- `busy us/v`: elapsed time minus the worker core's idle-task runtime delta. Blocking that runs idle is excluded, but other tasks can contribute. Interrupt accounting follows the RTOS policy; this is not exclusive DShot task or ISR CPU time.
- `est core %`: extrapolates the sampled busy time across the window's worker updates. Sampling bias and instrumentation overhead affect this estimate; it is not an exact reconciliation with the task runtime table.

The empty probe row measures the timestamp-pair overhead; compare small stage timings against it. Probe overhead is reported rather than subtracted. Scheduler work before/after the sampled pass and the cost of merging samples are outside the stage table. `idle clamps` counts boundaries where idle accounting slightly exceeds elapsed time; their busy time is clamped to zero. Compare `profile 5 basic` with `profile 5` at the same workload to assess added measurement overhead.
