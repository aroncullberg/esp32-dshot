get_filename_component(DSHOT_COMPONENT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
get_filename_component(DSHOT_COMPONENT_NAME "${DSHOT_COMPONENT_ROOT}" NAME)
idf_component_register(
    SRCS "bench_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES ${DSHOT_COMPONENT_NAME} console esp_timer esp_driver_gpio freertos
)
