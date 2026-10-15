get_filename_component(DSHOT_COMPONENT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
get_filename_component(DSHOT_COMPONENT_NAME "${DSHOT_COMPONENT_ROOT}" NAME)
idf_component_register(
    SRCS "bench_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES ${DSHOT_COMPONENT_NAME} console esp_timer esp_driver_gpio freertos
)
# Reporting must leave headroom for nested ESP-IDF and libc calls on the 8 KiB task stack.
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror=frame-larger-than=2048)
