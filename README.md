# esp32-dshot

A C++ DShot component for ESP-IDF 6.1, made for the ESP32-C3 and ESP32-S3.

It's intended to be used alongside [esp32-esc-passthrough](https://github.com/aroncullberg/esp32-esc-passthrough), but works on its own too.

## Features

- Regular DShot150, DShot300, and DShot600 using RMT.
- Bidirectional DShot300 and DShot600 with eRPM, motor RPM, and Extended DShot Telemetry (EDT).
- ESC commands, including beeps, motor direction, and saving settings.
- Multiple ESC outputs, configurable frame rate, and per-output statistics.
- Helpers to release and reclaim the ESC signal pins for passthrough.

## Compatibility

Only tested on ESP32-C3 and ESP32-S3. Other ESP32 chips should work, but you'll need to try and verify them yourself. The component manifest currently lists only C3 and S3.

It should work with both Bluejay and AM32 ESC firmware, but only Bluejay has been tested so far.

## Example

The [bench example](examples/bench/README.md) includes build instructions and a serial console for running a motor, sending ESC commands, and reading telemetry and statistics.

To use it in your own ESP-IDF project, place this repo in `components/dshot` or add it to `EXTRA_COMPONENT_DIRS`.

## misc

If you're interested in the scope captures used to develop, debug, and test this, send a message and i'll get around to uploading them.
