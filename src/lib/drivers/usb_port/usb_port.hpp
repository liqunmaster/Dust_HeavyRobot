#pragma once

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include "dvc_usb.hpp"

class UsbPort
{
public:
	void Init();

	int Receive(void *data, size_t length);

	int Transmit(const void *data, size_t length);

	bool Connected();

	uint32_t ReceivedCount();

	uint32_t DroppedCount();

private:
	void UsbPortProcess();

	static void ThreadEntry(void *arg1, void *arg2, void *arg3);

	void Task();
	uint8_t _rx_storage[2048] {};
	struct ring_buf _rx_ring {};
	struct k_thread _usb_thread {};
	atomic_t _received_count{};
	atomic_t _dropped_count{};
	bool _ready = false;
};

void usb_port_init();

int usb_port_receive(void *data, size_t length);

int usb_port_transmit(const void *data, size_t length);

bool usb_port_connected();

uint32_t usb_port_received_count();

uint32_t usb_port_dropped_count();
