# DShot bench

Build from this directory with ESP-IDF 6.1:

```sh
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Set the signal GPIO in `DShot bench`, and bitrate/signaling/frame rate in `Component config → DShot`. Use regular D150/D300/D600. For a new configuration, first select `esp32c3` or `esp32s3` with `idf.py set-target`.

- `arm`: start zero frames, wait the configured interval, then keep zero frames running.
- `run <0..2047>`: hold the raw value until another run or stop.
- `stop`: return to zero frames without stopping transmission.
- `stats`: show per-GPIO completed frames/s, frame/error totals, and missed slots.
- `beep [1..5]`: issue one beep, defaulting to tone 1.
- `direction <normal|reversed|1|2>`: issue a direction command.
- `save`: save ESC settings.
- `command <name|id|list|status>`: list/send other named commands or inspect progress.

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
