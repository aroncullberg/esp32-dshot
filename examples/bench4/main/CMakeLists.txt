if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "The four-motor bench requires ESP32-S3 (four TX and four RX channels).")
endif()

get_filename_component(DSHOT_COMPONENT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
get_filename_component(DSHOT_COMPONENT_NAME "${DSHOT_COMPONENT_ROOT}" NAME)
idf_component_register(
    SRCS "../../bench/main/bench_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES ${DSHOT_COMPONENT_NAME} console esp_timer esp_driver_gpio freertos
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE DSHOT_BENCH_FOUR_MOTORS=1)
# Catch large local state/return-value temporaries before they exhaust the bench stack.
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror=frame-larger-than=2048)
