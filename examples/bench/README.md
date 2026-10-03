# DShot bench

Build from this directory with ESP-IDF 6.1:

```sh
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Set the signal GPIO in `DShot bench`, and bitrate/signaling/frame rate in
`Component config → DShot`. Regular D150/D300/D600 and bidirectional D300/D600
are supported. For a new configuration, select `esp32c3` or `esp32s3` with
`idf.py set-target` first.

- `arm`: start zero frames, wait the configured interval, then keep zero frames running.
- `run <0..2047>`: hold the raw value until another run or stop.
- `stop`: return to zero frames without stopping transmission.
- `stats`: show per-GPIO completed frames/s, frame/error totals, and missed slots.
- `beep [1..5]`: issue one beep, defaulting to tone 1.
- `direction <normal|reversed|1|2>`: issue a direction command.
- `save`: save ESC settings.
- `command <name|id|list|status>`: list/send other named commands or inspect progress.
- `telemetry`: show eRPM, motor RPM, received EDT fields, their ages, and RX counters.
- `telemetry raw`: show the latest nonempty RX capture, decode result, age and
  pulse durations in clock ticks. Useful when malformed/GCR/CRC counters grow.
- `edt <on|off|status>`: request EDT enable/disable or inspect telemetry.

Set **Component config → DShot → Default motor magnetic pole count** to your
motor's total magnetic poles (default 14). This option appears with bidirectional
signaling; C++ can override it through `Config::motor_poles`.

For the first receive test select **Bidirectional (inverted)**, D600, and 1000 Hz.
Restart the ESC after changing signaling. `arm`, wait for arming, then `telemetry`
should show a fresh stopped sample. Try `run 100`, `telemetry`, then `stop` and
`edt on`. Read `telemetry` again after about a second. See
[telemetry setup and troubleshooting](../../docs/TELEMETRY.md).

After arming, use `stop` before sending an ESC command. A command leaves the link
sending zero; use `run` again after completion. `stop` cancels remaining repetitions.
See [command usage and API](../../docs/COMMANDS.md), including the distinction
between AM32's direction 1/2 and normal/reversed commands.

The first `stats` rate covers the session so far; subsequent readings use the
interval since the previous reading. It leaves the stream and payload unchanged.
Enable `DSHOT_TIMING_DIAGNOSTICS` under `Component config → DShot` for detailed
elapsed timings. See [statistics and baseline measurement](../../docs/STATISTICS.md).

To increase cadence, change **Component config → DShot → Default frames per
second** to 2000 or 4000, rebuild, and flash. GPTimer supplies the wake-up period;
the FreeRTOS tick rate can remain unchanged. `stats` also prints the configured
timer period (500 us at 2 kHz, 250 us at 4 kHz). All links are still sent sequentially.
