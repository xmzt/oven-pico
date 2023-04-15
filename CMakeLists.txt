cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(oven-pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(oven)
pico_enable_stdio_usb(oven 1)
pico_generate_pio_header(oven ${CMAKE_CURRENT_LIST_DIR}/nmtc.pio)
target_sources(oven PRIVATE oven.c nmtc.c tmp101.c)
target_link_libraries(oven PRIVATE pico_stdlib hardware_i2c hardware_pio)
pico_add_extra_outputs(oven)
