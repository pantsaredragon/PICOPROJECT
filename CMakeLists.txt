cmake_minimum_required(VERSION 3.5)

# Pull in SDK (must be before project)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(PicoProject C CXX ASM)
include(PicoSDKConfig.cmake)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_subdirectory(adxl355)
add_subdirectory(kemet)