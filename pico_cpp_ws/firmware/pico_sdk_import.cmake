# Helper to import the Pico SDK if PICO_SDK_PATH is set in your environment
if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif ()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH not set. Please set environment or edit pico_sdk_import.cmake")
endif ()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
