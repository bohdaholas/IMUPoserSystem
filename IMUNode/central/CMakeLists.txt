# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

add_compile_options(-Os)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS
        ${CMAKE_CURRENT_LIST_DIR}/../common/nvs_utils
        ${CMAKE_CURRENT_LIST_DIR}/../common/button
        ${CMAKE_CURRENT_LIST_DIR}/../common/led
        ${CMAKE_CURRENT_LIST_DIR}/../common/imu
)
set(IDF_COMPONENT_MANAGER "1" CACHE INTERNAL "")
project(central)
