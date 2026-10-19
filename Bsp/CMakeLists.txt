target_sources(app PRIVATE
    bsp_crc.c
    bsp_canfd.cpp
    bsp_spi.cpp
    bsp_gpio.c
    bsp_pwm.cpp
    bsp_uart.cpp
    bsp_rs485.cpp
    bsp_usb.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(TINYUSB_SRC ${ROBOT_ROOT}/ThirdParty/tinyusb/src)
target_sources(app PRIVATE
    ${TINYUSB_SRC}/tusb.c
    ${TINYUSB_SRC}/common/tusb_fifo.c
    ${TINYUSB_SRC}/device/usbd.c
    ${TINYUSB_SRC}/class/cdc/cdc_device.c
    ${TINYUSB_SRC}/portable/chipidea/ci_hs/dcd_ci_hs.c
    ${HPM_SDK_DIR}/drivers/src/hpm_crc_drv.c
    ${HPM_SDK_DIR}/drivers/src/hpm_usb_drv.c
)
