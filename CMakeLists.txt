cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD "pico_w")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include_directories(${PICO_SDK_PATH}/src/common/pico_base/include)

project(micro_ros_pico_w_brazo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(3rdparty)

link_directories(libmicroros)
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/picow_udp_transports.c
    src/Servo.cpp
    src/Brazo.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    microros
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    fixmath
)

target_include_directories(${PROJECT_NAME} PUBLIC
    libmicroros/include
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    3rdparty/pico-libfixmath/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(${PROJECT_NAME})
