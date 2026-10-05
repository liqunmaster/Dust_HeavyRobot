cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

set(CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/fdcan.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/gpio.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/pwm.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/spi.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/uart.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/usb.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Topic/topic.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/imu.conf
    CACHE STRING "Application configuration files" FORCE
)
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/fdcan.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/ws2812b.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/uart.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/usb.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/imu.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/dji_c610.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/dji_c620.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/dm.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/cubemars.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/servo.overlay
    CACHE STRING "Application devicetree overlays" FORCE
)

set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361 CACHE PATH "" FORCE)
set(BOARD_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361
    CACHE INTERNAL "Board directories" FORCE)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "Suppress deprecated CMake warnings" FORCE)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyRobot)

zephyr_compile_options(-Ofast)

add_subdirectory(Algorithm)
add_subdirectory(App)
add_subdirectory(Bsp)
add_subdirectory(Communication)
add_subdirectory(Device)
add_subdirectory(Health_Monitor)
add_subdirectory(Module)
add_subdirectory(ThirdParty)
add_subdirectory(Topic)

set_source_files_properties(
    "${ZEPHYR_EXTRA_MODULES}/drivers/serial/uart_hpmicro.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
