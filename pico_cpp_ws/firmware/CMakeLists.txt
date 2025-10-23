cmake_minimum_required(VERSION 3.13)

# Imposta PICO_SDK_PATH nell'ambiente o usa pico_sdk_import.cmake nel repo superiore
include(pico_sdk_import.cmake)

project(pico_firmware_cpp C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_firmware
    src/main.cpp
    src/protocol.cpp
    src/bno055.cpp
    src/encoder.cpp
    src/pcf8574.cpp
    src/sonar3.cpp
    src/ina226.cpp
)

target_include_directories(pico_firmware PRIVATE src)

target_link_libraries(pico_firmware
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_pwm
)

pico_enable_stdio_usb(pico_firmware 1)
pico_enable_stdio_uart(pico_firmware 0)

pico_add_extra_outputs(pico_firmware)
