# Four-motor profiling results

Measurements supplied on 2026-10-15: ESP32-S3 at 240 MHz, DShot600 bidirectional,
four outputs at 1,000 Hz, worker pinned to core 1, motors armed with raw value 0.
These are individual hardware samples, not repeated statistical benchmarks.

## Worker CPU across optimization stages

| Configuration | Worker CPU, basic report | Worker CPU, sampled stages enabled |
| --- | ---: | ---: |
| Debug optimization (`-Og`), old timing diagnostics off | 38.80% | 39.91% |
| Performance optimization (`-O2`), stack fixes | 33.88% | 35.42% |
| `-O2`, integer pulse ranges and direct GCR lookup | 28.34% | 30.31% |
| `-O2`, optimized decoder, RX-first completion and batched transport | 21.03% | 22.48% |

100% means one fully occupied core. The comparable basic reports show:

- `-Og` to `-O2`: 4.92 percentage points saved, or 12.7% less worker CPU.
- Decoder optimization at `-O2`: another 5.54 percentage points saved, or 16.4% less worker CPU.
- RX-first completion and batched transport: another 7.31 percentage points
  saved, or 25.8% less worker CPU than the optimized decoder with sequential transport.
- Combined: 38.80% -> 21.03%, saving 17.77 percentage points, or 45.8% less worker CPU.
- At 1,000 updates/s, accounted worker CPU falls from approximately 388 to 210
  microseconds per four-output update, saving approximately 178 microseconds.

Sampled reply decoding fell from 21.73 to 12.27 microseconds per motor reply:
43.5% less elapsed/busy time in that stage before transport batching. That
optimized-decoder basic run and its sampled run each missed one slot. The
earlier `-O2` basic run missed zero slots and its sampled run missed two.

The original reports with older detailed timing enabled measured 41.87–41.99%
worker CPU in bidirectional mode. A separate run with that instrumentation
disabled measured 38.45%. Regular DShot measured 17.20% with it and 15.42%
without it. These configuration differences are separate from the decoder gain.

## Measured transport improvements

The final report adds RX-first completion and overlapping motor exchanges:

| Workload and report | Worker CPU | Missed slots | TX/RX/decode errors |
| --- | ---: | ---: | ---: |
| Four armed outputs, raw 0, basic | 21.03% | 1 | 0 |
| Four armed outputs, raw 0, sampled stages | 22.48% | 0 | 0 |
| Four armed outputs, raw 100, sampled stages | 22.28% | 0 | 0 |

The final basic sample transmitted 5,009 frames per motor in 5.010 seconds
(999.8 updates/s), with 5,009 valid replies per motor, no missing replies, no
TX/RX/decode errors, and one missed slot. Core 1 was 78.96% idle; core 0 was
approximately 0.2% busy. Stage profiling was disabled during this sample.

The two sampled-stage five-second windows ran at approximately 1,000 updates/s
with zero missed slots and no errors. Comparing the same raw-0 workload with
stage profiling enabled:

- Optimized decoder, sequential transport: 30.31% -> batched transport: 22.48%.
  This saves 7.83 percentage points, or 25.8% of the earlier worker CPU.
- Initial `-Og` sampled run: 39.91% -> final sampled run: 22.48%.
  This saves 17.43 percentage points, or 43.7% of the earlier worker CPU.
- Sampled TX wait elapsed time fell from 42.60 to 6.43 microseconds per visit;
  RX wait fell from 86.33 to 13.82 microseconds per visit. RX exchanges overlap,
  so shorter collection waits do not imply a faster physical ESC response.

The final armed basic sample now provides a comparison with the earlier
28.34% basic result: 28.34% -> 21.03%, a 25.8% reduction from the transport
changes. The 22.48% sampled-stage result is 1.45 percentage points higher than
the final basic result in these individual runs; keep basic and sampled-stage
comparisons separate.
Small differences between per-motor frame/reply deltas can occur because the
driver and telemetry snapshots are collected sequentially at each boundary.

Minimum reported worker stack free was 2,488 bytes in the final basic sample;
the bench remained at 5,828 bytes. Host tests pass for both target configurations with
ASan/UBSan, and ESP32-S3 bench4 and ESP32-C3 bench builds pass. New regression
checks cover simultaneous in-flight buffers, out-of-order callbacks, stop,
partial submission/TX failure, one stalled receiver, command gaps, and RX
deadlines including tick rollover.

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
  The latest report has 5,828 bytes minimum bench stack free and 2,488 bytes
  minimum worker stack free.
- Compile-time D300/D600 pulse acceptance ranges for the 40 MHz RMT clock,
  preserving the 35% tolerance and generic-clock behavior. Received pulses no
  longer require individual 64-bit divisions. A direct 32-entry GCR lookup
  replaces alphabet searches.
- Exhaustive comparison against the original decoder for all 1,048,576 GCR
  words and all 15-bit pulse widths at every position of a representative reply,
  plus mutated/truncated captures and alternate clocks. Host suites use ASan/UBSan.
- Split link and transport submission from completion so all active motors'
  hardware exchanges overlap before the worker collects their results. Per-link
  TX/RX buffers remain owned by their exchanges until completion.
- Bidirectional RX completion is collected before TX completion, avoiding a
  separate TX sleep during normal exchanges. The TX ISR still releases each
  pin independently; bounded TX waits remain for early RX and fault recovery.
- RX timeout budgets start at submission. Stop, partial submission failures,
  and TX errors still drain every exchange already submitted. A command gap
  skips only that motor, and per-link RX failures can recover on the next update.
- Detailed RX wait statistics exclude decoding, and overlapping per-link
  elapsed intervals are documented separately from total worker-pass latency.

## Interpretation and remaining work

Use basic task runtime to compare CPU consumption. Stage busy time subtracts
core idle from elapsed time and can include other tasks and interrupts. Stage
percentages extrapolate sampled updates, including measurement overhead, so
their sum is not a breakdown of the basic task-runtime percentage. Elapsed wait
time also includes periods when hardware operates while the core is idle.

The transport now starts all active motor exchanges before collecting
results and collects bidirectional RX before TX completion. It preserves per-link
buffers, pin release in the TX ISR, bounded failure handling, and RX deadlines
counted from submission. The final otherwise idle bench results support pausing
optimization here. The next validation should be integration under the intended
application's competing load, checking missed
slots, error counters, and worst-case update latency as well as average CPU.

## Source reports

- [Regular, old diagnostics enabled](../../d600-reg-1khz.txt)
- [Bidirectional, old diagnostics enabled](../../d600-bidir-1khz.txt)
- [Regular, old diagnostics disabled](../../d600-reg-1khz-withoutdiagnostics.txt)
- [Bidirectional, old diagnostics disabled](../../d600-bidir-1khz-withoutdiagnostics.txt.txt)
- [Debug optimization and stage sampling](../../d600-bidir-1khz-detailed.txt)
- [Performance optimization and stack fixes](../../d600-bidir-1khz-detailed-02.txt)
- [Optimized reply decoder](../../d600-bidir-1khz-detailed-o2-replydecodingoptimized.txt)
- [RX-first completion and batched transport, raw 0 and raw 100](../../d600-bidir-1khz-detailed-o2-final.txt)
- [Final armed basic sample, raw 0](../../d600-bidir-1khz-basic-o2-final.txt)
