#!/usr/bin/env python3
"""Run real POC sources against host fake hardware/RTOS, using configured IDF headers.
Usage: python3 tests/host/run.py [path/to/idf/build]
Requires clang++ and an IDF compile_commands.json from a configured C3/S3 bench.
"""
from pathlib import Path
import json
import os
import shlex
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
build = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else root / "examples/bench/build"
database = json.loads((build / "compile_commands.json").read_text())
entry = next(item for item in database if item["file"].endswith("/src/frame_codec.cpp"))
args = shlex.split(entry["command"])
# Avoid target libc/FreeRTOS headers; the harness replaces only OS and GPIO operations.
components = ("esp_common", "esp_driver_rmt", "esp_hal_rmt", "esp_hal_gpio", "soc", "esp_rom", "esp_hw_support")
includes = [arg for arg in args if arg.startswith("-I") and any(f"/{name}/" in arg for name in components)]
sources = [root / "src" / name for name in ("config.cpp", "frame_codec.cpp", "idf_transport.cpp", "driver.cpp", "link.cpp", "frame_scheduler.cpp", "command.cpp", "command_sequence.cpp")]
for detailed in (0, 1):
    with tempfile.TemporaryDirectory(prefix="dshot-host-") as temporary:
        binary = Path(temporary) / "poc_test"
        # Test both builds regardless of the configuration used to obtain IDF headers.
        (Path(temporary) / "sdkconfig.h").write_text(
            '#include "' + str(build / "config/sdkconfig.h") + '"\n'
            '#undef CONFIG_DSHOT_TIMING_DIAGNOSTICS\n'
            '#undef CONFIG_FREERTOS_HZ\n#define CONFIG_FREERTOS_HZ 100\n'
            f'#define CONFIG_DSHOT_TIMING_DIAGNOSTICS {detailed}\n')
        command = [os.environ.get("CXX", "clang++"), "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Werror",
                   "-Wno-extern-c-compat", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                   "-I" + temporary, "-I" + str(root / "tests/host/fakes"), "-I" + str(build / "config"),
                   "-I" + str(root / "include"), "-I" + str(root / "src"), *includes,
                   str(root / "tests/host/poc_test.cpp"), str(root / "tests/host/learning_checks.cpp"), str(root / "tests/host/command_test.cpp"), *(str(source) for source in sources), "-o", str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True, timeout=20)

# Exercise the actual bench command parser/worker separately from the driver mocks above.
with tempfile.TemporaryDirectory(prefix="dshot-bench-test-") as temporary:
    binary = Path(temporary) / "bench_test"
    command = [os.environ.get("CXX", "clang++"), "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Werror",
               "-Wno-extern-c-compat", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-I" + str(root / "tests/host/fakes"), "-I" + str(build / "config"),
               "-I" + str(root / "include"), "-I" + str(root / "src"), *includes,
               str(root / "tests/host/bench_test.cpp"), str(root / "src/config.cpp"), str(root / "src/command.cpp"), "-o", str(binary)]
    subprocess.run(command, check=True)
    subprocess.run([str(binary)], check=True, timeout=20)
