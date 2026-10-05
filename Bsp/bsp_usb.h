#pragma once

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <hpm_clock_drv.h>
#include <hpm_usb_drv.h>
#include <tusb.h>

#ifdef __cplusplus
extern "C"
{
#endif

    int bsp_usb_init(void);

    void bsp_usb_task(void);

    int bsp_usb_wait_event(k_timeout_t timeout);

    int bsp_usb_receive(void *data, size_t length);

    int bsp_usb_transmit(const void *data, size_t length);

    bool bsp_usb_connected(void);

#ifdef __cplusplus
}
#endif
