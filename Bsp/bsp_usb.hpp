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

    int bsp_usb_receive(void *data, size_t length);

    int bsp_usb_transmit(const void *data, size_t length);

    bool bsp_usb_connected(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

struct UsbState
{
    static constexpr size_t configuration_length = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN;
    bool ready{};
    uint32_t irq_count{};
    uint32_t reset_count{};
    uint32_t setup_count{};
    uint8_t other_speed_configuration[configuration_length]{};
    uint16_t string_descriptor[32]{};
};
#endif
