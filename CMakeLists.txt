cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

# The external HPM MCAN driver must report one-shot TX cancellations.
set(HPM_MCAN_DRIVER "${ZEPHYR_EXTRA_MODULES}/drivers/can/mcan_hpmicro.c")
if(NOT EXISTS "${HPM_MCAN_DRIVER}")
    message(FATAL_ERROR "HPM MCAN driver not found: ${HPM_MCAN_DRIVER}")
endif()
file(READ "${HPM_MCAN_DRIVER}" HPM_MCAN_DRIVER_SOURCE)
string(FIND "${HPM_MCAN_DRIVER_SOURCE}" "mcan_is_tx_buf_cancellation_finished(can, 0)" HPM_MCAN_CANCEL_CHECK)
string(FIND "${HPM_MCAN_DRIVER_SOURCE}" "txbuf_cancel_finish_interrupt_mask = 1U" HPM_MCAN_CANCEL_IRQ)
if(HPM_MCAN_CANCEL_CHECK EQUAL -1 OR HPM_MCAN_CANCEL_IRQ EQUAL -1)
    message(FATAL_ERROR "HPM MCAN cancellation fix is missing; apply patches/hpm-mcan-tx-cancel.patch in ${ZEPHYR_EXTRA_MODULES}")
endif()
string(FIND "${HPM_MCAN_DRIVER_SOURCE}" "mcan_get_rxfifo_fill_level(can, 0)" HPM_MCAN_FIFO0_DRAIN)
string(FIND "${HPM_MCAN_DRIVER_SOURCE}" "mcan_get_rxfifo_fill_level(can, 1)" HPM_MCAN_FIFO1_DRAIN)
if(HPM_MCAN_FIFO0_DRAIN EQUAL -1 OR HPM_MCAN_FIFO1_DRAIN EQUAL -1)
    message(FATAL_ERROR "HPM MCAN RX FIFO drain fix is missing; apply patches/hpm-mcan-rx-fifo-drain.patch in ${ZEPHYR_EXTRA_MODULES}")
endif()
unset(HPM_MCAN_DRIVER_SOURCE)

set(HPM_SPI_DRIVER "${ZEPHYR_EXTRA_MODULES}/drivers/spi/spi_hpmicro.c")
if(NOT EXISTS "${HPM_SPI_DRIVER}")
    message(FATAL_ERROR "HPM SPI driver not found: ${HPM_SPI_DRIVER}")
endif()
file(READ "${HPM_SPI_DRIVER}" HPM_SPI_DRIVER_SOURCE)
string(FIND "${HPM_SPI_DRIVER_SOURCE}" "tx_bufs->buffers[0].len <= 32U" HPM_SPI_SMALL_PIO)
if(HPM_SPI_SMALL_PIO EQUAL -1)
    message(FATAL_ERROR "HPM SPI small-transfer PIO fix is missing; apply patches/hpm-spi-small-transfer-pio.patch in ${ZEPHYR_EXTRA_MODULES}")
endif()
unset(HPM_SPI_DRIVER_SOURCE)

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
