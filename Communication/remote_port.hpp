#pragma once

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "bsp_uart.hpp"
#include "input.hpp"
#include "remote_types.hpp"

struct remote_rx_chunk
{
    remote_uart_source source;
    uint16_t length;
    uint32_t timestamp_ms;
    bool discontinuity;
    uint8_t bytes[128];
};

int remote_port_init();

uint32_t remote_port_feedback_count();
