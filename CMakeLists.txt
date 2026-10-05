cmake_minimum_required(VERSION 3.20)

project(APPS-BSPD LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F446RCTX_FLASH.ld")
set(STARTUP_FILE "${CMAKE_SOURCE_DIR}/startup_stm32f446rctx.s")

file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/Core/Src/*.c"
    "${CMAKE_SOURCE_DIR}/Core/Src/*.cpp")
file(GLOB_RECURSE DRIVER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE USB_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/USB_DEVICE/App/*.c"
    "${CMAKE_SOURCE_DIR}/USB_DEVICE/Target/*.c"
    "${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c"
    "${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c")

add_executable(${PROJECT_NAME}.elf
    ${CORE_SOURCES}
    ${DRIVER_SOURCES}
    ${USB_SOURCES}
    "${STARTUP_FILE}")

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    USB_DEVICE/App
    USB_DEVICE/Target
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32F446xx)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -Wall -Wextra
    -fdata-sections -ffunction-sections
    "$<$<CONFIG:Debug>:-Og;-g3>"
    $<$<CONFIG:Release>:-O2>)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    "-T${LINKER_SCRIPT}"
    -Wl,--gc-sections
    "-Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map"
    -Wl,--print-memory-usage)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        "$<TARGET_FILE:${PROJECT_NAME}.elf>" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary
        "$<TARGET_FILE:${PROJECT_NAME}.elf>" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin"
    COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${PROJECT_NAME}.elf>"
    VERBATIM)

set(OPENOCD_EXECUTABLE openocd CACHE FILEPATH "Path to the OpenOCD executable")
set(OPENOCD_SCRIPTS_DIR "" CACHE PATH "Optional path to the OpenOCD scripts directory")

set(OPENOCD_COMMAND ${OPENOCD_EXECUTABLE})
if(OPENOCD_SCRIPTS_DIR)
    list(APPEND OPENOCD_COMMAND -s "${OPENOCD_SCRIPTS_DIR}")
endif()
list(APPEND OPENOCD_COMMAND
    -f interface/stlink.cfg
    -f target/stm32f4x.cfg
    -c "program $<TARGET_FILE:${PROJECT_NAME}.elf> verify reset exit")

add_custom_target(flash
    COMMAND ${OPENOCD_COMMAND}
    DEPENDS ${PROJECT_NAME}.elf
    USES_TERMINAL
    VERBATIM)
