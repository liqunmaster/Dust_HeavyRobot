#include "usb_port.hpp"

namespace
{
    UsbPort usb_port;
    K_THREAD_STACK_DEFINE(usb_stack, 1024);
}

void UsbPort::UsbPortProcess()
{
    uint8_t bytes[128];
    int count;
    while ((count = bsp_usb_receive(bytes, sizeof(bytes))) > 0) {
        const unsigned int key = irq_lock();
        const uint32_t written = ring_buf_put(&rx_ring_, bytes, static_cast<uint32_t>(count));
        irq_unlock(key);
        atomic_add(&received_count_, written);
        atomic_add(&dropped_count_, static_cast<uint32_t>(count) - written);
    }
}

void UsbPort::ThreadEntry(void *context, void *, void *)
{
    static_cast<UsbPort *>(context)->Task();
}

void UsbPort::Task()
{
    while (1) {
        bsp_usb_task();
        UsbPortProcess();
    }
}

void UsbPort::Init()
{
    if (k_is_in_isr()) {
        return;
    }
    if (ready_) {
        return;
    }
    const int ret = bsp_usb_init();
    if (ret != 0) {
        return;
    }

    ring_buf_init(&rx_ring_, sizeof(rx_storage_), rx_storage_);
    ready_ = true;
    k_thread_create(&usb_thread_, usb_stack, K_THREAD_STACK_SIZEOF(usb_stack), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

int UsbPort::Receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready_) {
        return -ENODEV;
    }
    if (data == nullptr && length != 0) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0) {
        return 0;
    }

    const unsigned int key = irq_lock();
    const uint32_t count = ring_buf_get(&rx_ring_, static_cast<uint8_t *>(data), static_cast<uint32_t>(length));
    irq_unlock(key);
    return static_cast<int>(count);
}

int UsbPort::Transmit(const void *data, size_t length)
{
    return bsp_usb_transmit(data, length);
}

bool UsbPort::Connected()
{
    return bsp_usb_connected();
}

uint32_t UsbPort::ReceivedCount()
{
    return static_cast<uint32_t>(atomic_get(&received_count_));
}

uint32_t UsbPort::DroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&dropped_count_));
}

void usb_port_init()
{
    usb_port.Init();
}

int usb_port_receive(void *data, size_t length)
{
    return usb_port.Receive(data, length);
}

int usb_port_transmit(const void *data, size_t length)
{
    return usb_port.Transmit(data, length);
}

bool usb_port_connected()
{
    return usb_port.Connected();
}

uint32_t usb_port_received_count()
{
    return usb_port.ReceivedCount();
}

uint32_t usb_port_dropped_count()
{
    return usb_port.DroppedCount();
}
