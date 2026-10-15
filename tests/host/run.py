#!/usr/bin/env python3
"""Run host tests using headers from a configured ESP-IDF bench build."""
from pathlib import Path
import argparse
import json
import os
import shlex
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "tests/host"
COMPONENTS = {"esp_common", "esp_driver_rmt", "esp_hal_rmt", "esp_hal_gpio", "soc", "esp_rom", "esp_hw_support"}
UNIT_SOURCES = [
    HOST / "unit_test.cpp", HOST / "frame_codec_test.cpp",
    HOST / "command_sequence_test.cpp", HOST / "telemetry_test.cpp",
    *(ROOT / "src" / name for name in (
        "config.cpp", "frame_codec.cpp", "command.cpp", "command_sequence.cpp",
        "telemetry_codec.cpp", "telemetry_state.cpp")),
]
INTEGRATION_SOURCES = [
    HOST / "driver_integration_test.cpp", HOST / "host_support.cpp",
    *(ROOT / "src" / name for name in (
        "config.cpp", "frame_codec.cpp", "idf_transport.cpp", "driver.cpp",
        "link.cpp", "passthrough.cpp", "frame_scheduler.cpp", "command.cpp",
        "command_sequence.cpp", "telemetry_codec.cpp", "telemetry_state.cpp")),
]
BENCH_SOURCES = [
    HOST / "bench_console_test.cpp",
    *(ROOT / "src" / name for name in ("config.cpp", "command.cpp", "telemetry_state.cpp")),
]


def idf_includes(build):
    database_path = build / "compile_commands.json"
    config_path = build / "config/sdkconfig.h"
    for path in (database_path, config_path):
        if not path.is_file():
            raise ValueError(f"Missing {path}. Configure the bench with ESP-IDF first; see tests/README.md.")
    database = json.loads(database_path.read_text())
    entry = next((item for item in database if item["file"].endswith("/src/frame_codec.cpp")), None)
    if entry is None:
        raise ValueError(f"No DShot frame codec entry in {database_path}. Reconfigure the bench build.")
    args = entry["arguments"] if "arguments" in entry else shlex.split(entry["command"])
    includes = [arg for arg in args if arg.startswith("-I") and COMPONENTS.intersection(Path(arg[2:]).parts)]
    if not any((Path(arg[2:]) / "esp_err.h").is_file() for arg in includes):
        raise ValueError("ESP-IDF headers are unavailable at the recorded paths. Reconfigure the bench build.")
    return includes


def run_suite(name, sources, compiler, build, includes, diagnostics=0, defines=()):
    print(f"Running {name}", flush=True)
    with tempfile.TemporaryDirectory(prefix="dshot-host-") as temporary:
        binary = Path(temporary) / name
        (Path(temporary) / "sdkconfig.h").write_text(
            f'#include "{build / "config/sdkconfig.h"}"\n'
            '#undef CONFIG_DSHOT_TIMING_DIAGNOSTICS\n'
            '#undef CONFIG_DSHOT_STAGE_PROFILING\n#define CONFIG_DSHOT_STAGE_PROFILING 0\n'
            '#undef CONFIG_FREERTOS_HZ\n#define CONFIG_FREERTOS_HZ 100\n'
            '#ifndef CONFIG_DSHOT_DEFAULT_STARTUP_LOW_MS\n#define CONFIG_DSHOT_DEFAULT_STARTUP_LOW_MS 0\n#endif\n'
            f'#define CONFIG_DSHOT_TIMING_DIAGNOSTICS {diagnostics}\n' +
            ''.join(f'#undef {name}\n#define {name} {value}\n'
                    for name, value in (define.split('=', 1) for define in defines)))
        command = [
            *compiler, "-std=c++17", "-pthread", "-Wall", "-Wextra", "-Werror",
            "-Wno-extern-c-compat", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I" + temporary, "-I" + str(HOST / "fakes"), "-I" + str(build / "config"),
            "-I" + str(ROOT / "include"), "-I" + str(ROOT / "src"), *includes,
            *(str(source) for source in sources), "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True, timeout=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", nargs="?", type=Path, default=ROOT / "examples/bench/build",
                        help="configured ESP-IDF bench build directory")
    args = parser.parse_args()
    try:
        build = args.build.resolve()
        includes = idf_includes(build)
        compiler = shlex.split(os.environ.get("CXX", "clang++"))
        if not compiler or shutil.which(compiler[0]) is None:
            raise ValueError("Host C++ compiler not found. Install clang++ or set CXX to a host compiler with ASan/UBSan support.")
        run_suite("telemetry_codec_test", [HOST / "telemetry_codec_test.cpp", ROOT / "src/telemetry_codec.cpp"],
                  compiler, build, includes)
        run_suite("unit_test", UNIT_SOURCES, compiler, build, includes)
        for diagnostics in (0, 1):
            run_suite(f"driver_integration_test_{diagnostics}", INTEGRATION_SOURCES,
                      compiler, build, includes, diagnostics)
        stage_defines = ("CONFIG_DSHOT_STAGE_PROFILING=1", "CONFIG_DSHOT_STAGE_PROFILE_INTERVAL=16",
                         "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=1",
                         "CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER=1",
                         "CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64=1")
        run_suite("driver_stage_profile_test", INTEGRATION_SOURCES, compiler, build, includes,
                  defines=stage_defines)
        run_suite("bench_console_test", BENCH_SOURCES, compiler, build, includes)
        run_suite("bench4_console_test", BENCH_SOURCES, compiler, build, includes,
                  defines=("DSHOT_BENCH_FOUR_MOTORS=1", "CONFIG_DSHOT_BENCH_GPIO_2=3",
                           "CONFIG_DSHOT_BENCH_GPIO_3=4", "CONFIG_DSHOT_BENCH_GPIO_4=5",
                           "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=0"))
        for counter64 in (0, 1):
            stage_test_defines = (*stage_defines[:-1], f"CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64={counter64}")
            run_suite(f"stage_profiler_test_u{64 if counter64 else 32}", [HOST / "stage_profiler_test.cpp"],
                      compiler, build, includes, defines=stage_test_defines)
            run_suite(f"bench4_cpu_test_u{64 if counter64 else 32}", BENCH_SOURCES,
                      compiler, build, includes,
                      defines=("DSHOT_BENCH_FOUR_MOTORS=1", "CONFIG_DSHOT_BENCH_GPIO_2=3",
                               "CONFIG_DSHOT_BENCH_GPIO_3=4", "CONFIG_DSHOT_BENCH_GPIO_4=5",
                               "CONFIG_FREERTOS_NUMBER_OF_CORES=2",
                               "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=1",
                               "CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER=1",
                               "CONFIG_DSHOT_STAGE_PROFILING=1", "CONFIG_DSHOT_STAGE_PROFILE_INTERVAL=16",
                               "CONFIG_DSHOT_DEFAULT_TASK_CORE=1",
                               f"CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64={counter64}"))
        run_suite("bench4_stage_disabled_test", BENCH_SOURCES, compiler, build, includes,
                  defines=(*stage_defines, "CONFIG_DSHOT_STAGE_PROFILING=0", "DSHOT_BENCH_FOUR_MOTORS=1",
                           "CONFIG_DSHOT_BENCH_GPIO_2=3", "CONFIG_DSHOT_BENCH_GPIO_3=4",
                           "CONFIG_DSHOT_BENCH_GPIO_4=5", "CONFIG_FREERTOS_NUMBER_OF_CORES=2"))
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"error: {error}\n")
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"error: {error.cmd[0]} failed (exit {error.returncode}).\n")
    except subprocess.TimeoutExpired as error:
        parser.exit(1, f"error: {error.cmd[0]} exceeded the {error.timeout}-second timeout.\n")


if __name__ == "__main__":
    main()
