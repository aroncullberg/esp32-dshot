# Moving the bench to ESP-IDF 6.1

This repository now targets the 6.1 release series: `>=6.1,<6.2`. The component manifest enforces that range, while the component and bench require CMake 3.22 or newer. ESP32-C3 and ESP32-S3 remain supported build targets. The driver implementation files remain empty.

## Activate the new installation

Install the tools and Python environment belonging to 6.1 after downloading the source checkout. For a conventional Git installation, run `./install.sh esp32c3,esp32s3` from the checkout once; an installation managed by EIM should use EIM's tool installation workflow instead. The official [6.1 release page](https://github.com/espressif/esp-idf/releases/tag/v6.1) describes both approaches.

Open a fresh shell so the old compiler and Python environment are not already selected. Assuming the new directory is `~/esp/esp-idf-v6.1`:

```sh
. "$HOME/esp/esp-idf-v6.1/export.sh"
idf.py --version
printf '%s\n' "$IDF_PATH"
cmake --version
```

Substitute the actual checkout path if named differently. The SDK version should be 6.1 or a 6.1.x patch, and IDF_PATH should identify the new checkout. A source clone without matching tools is not yet a usable installation.

## Reconfigure an existing bench build

From `~/code/dshot/examples/bench`, optionally save a copy of the existing `sdkconfig` somewhere outside the repository, then run:

```sh
idf.py fullclean
idf.py reconfigure
idf.py menuconfig
idf.py build
```

`fullclean` removes generated build outputs, not your source or saved `sdkconfig`. Reconfiguration migrates the saved settings to the new SDK. Inspect DShot defaults, bench GPIO, console backend, and the transport-ready switch afterward. Keep that switch off while the component is unimplemented.

Do not manually edit the SDK version text or `CONFIG_IDF_INIT_VERSION` in `sdkconfig`: it is metadata, not the dependency pin. The authoritative supported SDK range is in `idf_component.yml`. Do not delete your pin configuration just to remove an old version string.

If the old build directory is incomplete and IDF refuses to clean it, rename it to `build-idf5-backup` and configure again. Use a different backup name if that directory already exists. If your build lives at a custom path, pass that same `-B` path to each command; otherwise the instructions and editor assume `examples/bench/build`.

For a completely new configuration, use `idf.py set-target esp32c3` or `idf.py set-target esp32s3` before menuconfig and build. Changing the target resets configuration, so it is unnecessary when upgrading an existing build for the same chip.

## Console setup in 6.1

Select the primary console under `Component config → ESP-STDIO → Channel for console output`. The bench now calls `esp_console_new_repl_stdio()`, which uses that selected backend. UART remains the default; native USB Serial/JTAG is useful when the board exposes it. A secondary output console is not the interactive input port.

The original `CONFIG_ESP_CONSOLE_UART_DEFAULT` and `CONFIG_ESP_CONSOLE_SECONDARY_NONE` names remain valid, so the defaults file does not need renamed symbols. The console menu has moved. Use the primary console's serial device for `idf.py -p YOUR_SERIAL_PORT flash monitor`.

## Changes that affect the tutorial and future driver

| Area | 6.1 guidance |
|---|---|
| Shared TX/RX GPIO | Assign the same GPIO to both objects. `io_loop_back` is removed. |
| Output handover | Still explicitly disable output before the ESC reply and wait for the driven interval to end before re-enabling. Shared routing does not perform this handover. |
| Initial output level | Set TX `flags.init_level` separately from transaction `flags.eot_level`; low for regular, high for bidirectional with `invert_out` disabled. |
| Open-drain | The RMT `io_od_mode` field is removed. Use GPIO APIs if a design needs open-drain. |
| Build dependencies | Use explicit `esp_driver_*` dependencies, already present in the scaffold. |
| FreeRTOS declarations | Include the FreeRTOS headers used by each implementation file explicitly. |
| Configuration syntax | Component Kconfig uses supported bool/int/choice syntax and uppercase symbols. GPIO limits follow the selected C3/S3 target. |
| C library | 6.x defaults to Picolibc. Keep standard I/O through public APIs; do not depend on Newlib internals or per-task stream replacement. |

The POC's copy encoder, one-shot `rmt_transmit()`, millisecond completion timeout, and task-based scheduler still use the same basic API pattern. There is no transport implementation to migrate yet; the documentation now teaches the 6.1 interfaces directly.

## Editor indexing

Regenerate `compile_commands.json` by configuring/building with 6.1 before restarting the language server. The repository's Zed configuration points to `examples/bench/build` and allows clangd to query both C3's RISC-V compiler and S3's Xtensa compiler. It retains the existing installed clangd executable path; if that executable is replaced during your tool installation, update that path to the installed binary. Never point the editor at an old build database from a different SDK or target.

## Verification scope

The migrated scaffold was built successfully for both ESP32-C3 and ESP32-S3 on 2026-09-26 using the local 6.1 checkout. The driver-enabled bench branch was also compile-checked on C3 against declarations matching the tutorial; the empty driver was not linked or implemented. No hardware was flashed or exercised during this migration.

A successful default bench build verifies the application scaffolding against the new SDK. It does not verify the empty driver or physical motor behavior. When implementing the tutorial, enable the transport-ready switch only after the documented interface exists and then rebuild. Verify the waveform before using a motor.

## References

- [6.0 peripheral migration](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/migration-guides/release-6.x/6.0/peripherals.html): changes inherited when upgrading from 5.x.
- [6.1 RMT TX interface](https://github.com/espressif/esp-idf/blob/v6.1/components/esp_driver_rmt/include/driver/rmt_tx.h).
- [6.1 console interface](https://github.com/espressif/esp-idf/blob/v6.1/components/console/esp_console.h).
- [6.1 standard-I/O configuration](https://github.com/espressif/esp-idf/blob/v6.1/components/esp_stdio/Kconfig).
