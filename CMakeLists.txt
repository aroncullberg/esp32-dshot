cmake_minimum_required(VERSION 3.22)

if(NOT IDF_TARGET STREQUAL "esp32s3" AND NOT IDF_TARGET STREQUAL "esp32c3")
    message(FATAL_ERROR "dshot supports ESP32-C3 and ESP32-S3. Use idf.py set-target esp32c3 or esp32s3.")
endif()

idf_component_register(
    SRCS
        "src/config.cpp"
        "src/command.cpp"
        "src/command_sequence.cpp"
        "src/driver.cpp"
        "src/frame_scheduler.cpp"
        "src/link.cpp"
        "src/passthrough.cpp"
        "src/frame_codec.cpp"
        "src/telemetry_codec.cpp"
        "src/telemetry_state.cpp"
        "src/idf_transport.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_driver_rmt esp_driver_gptimer esp_timer freertos
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
