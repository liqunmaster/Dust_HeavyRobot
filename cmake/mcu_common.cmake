get_filename_component(ROBOT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
add_compile_definitions(ROBOT_BOARD_${ROBOT_BOARD})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

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
string(FIND "${HPM_MCAN_DRIVER_SOURCE}" "offsetof(struct can_frame, data)" HPM_MCAN_RX_HOTPATH)
if(HPM_MCAN_RX_HOTPATH EQUAL -1)
    message(FATAL_ERROR "HPM MCAN RX hotpath fix is missing; apply patches/hpm-mcan-rx-hotpath.patch in ${ZEPHYR_EXTRA_MODULES}")
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
    ${ROBOT_ROOT}/prj.conf
    ${ROBOT_ROOT}/Bsp/canfd.conf
    ${ROBOT_ROOT}/Bsp/gpio.conf
    ${ROBOT_ROOT}/Bsp/pwm.conf
    ${ROBOT_ROOT}/Bsp/spi.conf
    ${ROBOT_ROOT}/Bsp/uart.conf
    ${ROBOT_ROOT}/Bsp/usb.conf
    ${ROBOT_ROOT}/Topic/topic.conf
    ${ROBOT_ROOT}/Device/imu.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf
    CACHE STRING "Application configuration files" FORCE
)
set(DTC_OVERLAY_FILE
    ${ROBOT_ROOT}/Bsp/canfd.overlay
    ${ROBOT_ROOT}/Device/ws2812b.overlay
    ${ROBOT_ROOT}/Bsp/uart.overlay
    ${ROBOT_ROOT}/Bsp/usb.overlay
    ${ROBOT_ROOT}/Bsp/remote.overlay
    ${ROBOT_ROOT}/Device/imu.overlay
    ${ROBOT_ROOT}/Device/servo.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/app.overlay
    CACHE STRING "Application devicetree overlays" FORCE
)

set(BOARD_DIR ${ROBOT_ROOT}/Board/hpm5361 CACHE PATH "" FORCE)
set(BOARD_DIRECTORIES ${ROBOT_ROOT}/Board/hpm5361
    CACHE INTERNAL "Board directories" FORCE)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "Suppress deprecated CMake warnings" FORCE)

list(APPEND DTS_ROOT ${ROBOT_ROOT}/Board/hpm5361)

function(mcu_add_common_sources)
    zephyr_compile_options(-O2)
    foreach(component IN ITEMS math Bsp platform protocol Device Health_Monitor Module ThirdParty Topic)
        add_subdirectory("${ROBOT_ROOT}/${component}" "${CMAKE_BINARY_DIR}/shared/${component}")
    endforeach()
    add_subdirectory("${ROBOT_ROOT}/firmware/common" "${CMAKE_BINARY_DIR}/shared/common")
    set_source_files_properties(
        "${ZEPHYR_EXTRA_MODULES}/drivers/serial/uart_hpmicro.c"
        PROPERTIES COMPILE_OPTIONS "-Wno-unused-function")
endfunction()
