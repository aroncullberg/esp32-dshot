# DShot bench

A serial console example for running a motor, sending ESC commands, and reading telemetry.

## Build and run

With ESP-IDF 6.1 activated, run from this directory:

```sh
idf.py set-target esp32c3
idf.py menuconfig
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Use `esp32s3` instead for an S3 board. Skip `set-target` if the project is already configured for your chip.

In `menuconfig`, set the signal GPIO under `DShot bench` and the bitrate, signaling mode, and frame rate under `Component config → DShot`. For motor RPM readings, set the motor's magnetic pole count too.

## Console commands

- `arm`: send zero frames through the arming interval, then keep sending.
- `run <0..2047>`: hold a raw DShot value until changed; there is no automatic stop.
- `stop`: return to zero frames and cancel any command sequence.
- `beep [1..5]`: play a tone.
- `direction <normal|reversed|1|2>`: set motor direction.
- `save`: save ESC settings.
- `command <name|id|list|status>`: send other commands, list them, or check progress.
- `telemetry`: show speed and Extended DShot Telemetry (EDT).
- `telemetry raw`: show the latest captured reply for debugging.
- `edt <on|off|status>`: enable, disable, or check EDT.
- `stats`: show frame rates and error counters.

Start with `arm` and wait for arming to finish before using `run`. Use `stop` before sending ESC commands.

Telemetry needs bidirectional signaling. Restart the ESC after changing signaling mode.

