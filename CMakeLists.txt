cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(oven-pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

string(TIMESTAMP build_ts)
add_compile_definitions(BUILD_TS="${build_ts}")

add_executable(oven)
pico_enable_stdio_usb(oven 1)
pico_generate_pio_header(oven ${CMAKE_CURRENT_LIST_DIR}/nmtc.pio)
target_sources(oven PRIVATE base.h mainq.h mainq.c nmtc.c nmtc.h oven.c tmp101.c tmp101.h util.h util.c)
target_link_libraries(oven PRIVATE pico_stdlib hardware_i2c hardware_pio)
pico_add_extra_outputs(oven)
