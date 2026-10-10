#pragma once

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include "bsp_uart.hpp"
#include "input.hpp"
#include "remote_types.hpp"

// 一组从串口通道收集到的原始数据片断，供上层组建完整遥控协议数据
struct remote_rx_chunk
{
    remote_uart_source source;
    uint16_t length;
    uint32_t timestamp_ms;
    bool discontinuity;
    uint8_t bytes[128];
};

void remote_port_init();

uint32_t remote_port_feedback_count();
