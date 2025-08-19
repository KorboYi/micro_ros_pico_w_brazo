cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD "pico_w")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include_directories(${PICO_SDK_PATH}/src/common/pico_base/include)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(pico_micro_ros_example
    src/pico_micro_ros_example.c
    src/picow_udp_transports.c
)

target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
    pico_cyw43_arch_lwip_threadsafe_background
)

target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include
)

target_include_directories(pico_micro_ros_example PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
