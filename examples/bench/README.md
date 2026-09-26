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

TODO
