cmake_minimum_required(VERSION 3.22)

get_filename_component(DSHOT_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${DSHOT_COMPONENT_DIR}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dshot_bench)
