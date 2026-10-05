#pragma once

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "bsp_usb.h"

int usb_port_init();

int usb_port_receive(void *data, size_t length);

int usb_port_transmit(const void *data, size_t length);

bool usb_port_connected();

uint32_t usb_port_received_count();

uint32_t usb_port_dropped_count();
