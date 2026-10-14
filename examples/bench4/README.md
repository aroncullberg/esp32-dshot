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

Other commands are the same as the [single-motor bench](../bench/README.md).
