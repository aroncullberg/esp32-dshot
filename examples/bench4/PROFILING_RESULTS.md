# Four-motor profiling results

Measurements supplied on 2026-10-15: ESP32-S3 at 240 MHz, DShot600 bidirectional,
four outputs at 1,000 Hz, worker pinned to core 1, motors armed with raw value 0.
These are individual hardware samples, not repeated statistical benchmarks.

## Measured improvements before transport batching

| Configuration | Worker CPU, basic report | Worker CPU, sampled stages enabled |
| --- | ---: | ---: |
| Debug optimization (`-Og`), old timing diagnostics off | 38.80% | 39.91% |
| Performance optimization (`-O2`), stack fixes | 33.88% | 35.42% |
| `-O2`, integer pulse ranges and direct GCR lookup | 28.34% | 30.31% |

100% means one fully occupied core. The comparable basic reports show:

- `-Og` to `-O2`: 4.92 percentage points saved, or 12.7% less worker CPU.
- Decoder optimization at `-O2`: another 5.54 percentage points saved, or 16.4% less worker CPU.
- Combined: 10.46 percentage points saved, or 27.0% less worker CPU.
- At 1,000 updates/s, the combined reduction is approximately 105 microseconds
  of accounted worker CPU per four-output update.

Sampled reply decoding fell from 21.73 to 12.27 microseconds per motor reply:
43.5% less elapsed/busy time in that stage. The latest basic report transmitted
5,009 frames per motor in 5.010 seconds, with a valid reply for every frame,
zero TX/decode errors, and one missed slot. The sampled run also missed one slot.
The earlier `-O2` basic run missed zero slots and its sampled run missed two.

The original reports with older detailed timing enabled measured 41.87–41.99%
worker CPU in bidirectional mode. A separate run with that instrumentation
disabled measured 38.45%. Regular DShot measured 17.20% with it and 15.42%
without it. These configuration differences are separate from the decoder gain.

## Changes included

- Asynchronous `cpu` and shared-window `profile [seconds] [basic]` commands.
  Core idle time, task runtime, frame rates, missed slots, and RX/error counters
  use the same sample window; stop remains responsive.
- Optional stage sampling, normally one complete update in 16, independent of
  the older timing diagnostics. `basic` disables stage sampling.
- Heap-backed bench state and profile snapshots, statistics output into existing
  storage, and a counters-only telemetry getter. These avoid large stack
  temporaries that overflowed the bench task under `-O2`.
- Task stack watermarks and a 2 KiB function-frame build guard in both benches.
  The latest report has 5,828 bytes minimum bench stack free and 2,552 bytes
  minimum worker stack free.
- Compile-time D300/D600 pulse acceptance ranges for the 40 MHz RMT clock,
  preserving the 35% tolerance and generic-clock behavior. Received pulses no
  longer require individual 64-bit divisions. A direct 32-entry GCR lookup
  replaces alphabet searches.
- Exhaustive comparison against the original decoder for all 1,048,576 GCR
  words and all 15-bit pulse widths at every position of a representative reply,
  plus mutated/truncated captures and alternate clocks. Host suites use ASan/UBSan.

## Interpretation and remaining work

Use basic task runtime to compare CPU consumption. Stage busy time subtracts
core idle from elapsed time and can include other tasks and interrupts. Stage
percentages extrapolate sampled updates, including measurement overhead, so
their sum is not a breakdown of the basic task-runtime percentage. Elapsed wait
time also includes periods when hardware operates while the core is idle.

Next, remove the separate blocking TX wait during bidirectional exchanges and
start all active motor exchanges before collecting results. Measure their CPU
and update-latency effects on hardware; no savings for those changes are claimed
here. Test under competing load as well as the otherwise idle bench.

## Source reports

- [Regular, old diagnostics enabled](../../d600-reg-1khz.txt)
- [Bidirectional, old diagnostics enabled](../../d600-bidir-1khz.txt)
- [Regular, old diagnostics disabled](../../d600-reg-1khz-withoutdiagnostics.txt)
- [Bidirectional, old diagnostics disabled](../../d600-bidir-1khz-withoutdiagnostics.txt.txt)
- [Debug optimization and stage sampling](../../d600-bidir-1khz-detailed.txt)
- [Performance optimization and stack fixes](../../d600-bidir-1khz-detailed-02.txt)
- [Optimized reply decoder](../../d600-bidir-1khz-detailed-o2-replydecodingoptimized.txt)
