#pragma once

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include "bsp_usb.hpp"

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
    uint8_t rx_storage_[2048]{};
    struct ring_buf rx_ring_{};
    struct k_thread usb_thread_{};
    atomic_t received_count_{};
    atomic_t dropped_count_{};
    bool ready_ = false;
};

void usb_port_init();

int usb_port_receive(void *data, size_t length);

int usb_port_transmit(const void *data, size_t length);

bool usb_port_connected();

uint32_t usb_port_received_count();

uint32_t usb_port_dropped_count();
