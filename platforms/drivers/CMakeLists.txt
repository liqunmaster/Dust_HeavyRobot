robot_component(robot_zephyr
	dvc_canfd.cpp
	dvc_crc.c
	dvc_gpio.c
	dvc_pwm.cpp
	dvc_rs485.cpp
	dvc_spi.cpp
	dvc_uart.cpp
	dvc_usb.cpp
	dvc_ws2812b.cpp
)
add_subdirectory(robot_platform_common)
set(TINYUSB_SRC ${ROBOT_ROOT}/ThirdParty/tinyusb/src)
target_sources(app PRIVATE
	${TINYUSB_SRC}/tusb.c
	${TINYUSB_SRC}/common/tusb_fifo.c
	${TINYUSB_SRC}/device/usbd.c
	${TINYUSB_SRC}/class/cdc/cdc_device.c
	${TINYUSB_SRC}/portable/chipidea/ci_hs/dcd_ci_hs.c
	${HPM_SDK_DIR}/drivers/src/hpm_crc_drv.c
	${HPM_SDK_DIR}/drivers/src/hpm_usb_drv.c)
