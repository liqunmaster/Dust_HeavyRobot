#include "usb_port.hpp"

namespace
{
    constexpr uint32_t rx_capacity = 2048U;
    uint8_t rx_storage[rx_capacity];
    struct ring_buf rx_ring;
    K_THREAD_STACK_DEFINE(usb_stack, 1024);
    struct k_thread usb_thread;
    atomic_t received_count;
    atomic_t dropped_count;
    bool ready;

    void usb_port_process()
    {
        uint8_t bytes[128];
        int count;
        while ((count = bsp_usb_receive(bytes, sizeof(bytes))) > 0) {
            const unsigned int key = irq_lock();
            const uint32_t written = ring_buf_put(&rx_ring, bytes, static_cast<uint32_t>(count));
            irq_unlock(key);
            atomic_add(&received_count, written);
            atomic_add(&dropped_count, static_cast<uint32_t>(count) - written);
        }
    }

    void usb_thread_entry(void *, void *, void *)
    {
        while (true) {
            (void)bsp_usb_wait_event(K_MSEC(10));
            bsp_usb_task();
            usb_port_process();
        }
    }
}
int usb_port_init()
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (ready) {
        return 0;
    }
    const int ret = bsp_usb_init();
    if (ret != 0) {
        return ret;
    }

    ring_buf_init(&rx_ring, sizeof(rx_storage), rx_storage);
    ready = true;
    k_thread_create(&usb_thread, usb_stack, K_THREAD_STACK_SIZEOF(usb_stack),
                    usb_thread_entry, nullptr, nullptr, nullptr,
                    K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    return 0;
}

int usb_port_receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
        return -ENODEV;
    }
    if (data == nullptr && length != 0U) {
        return -EINVAL;
    }
    if (length > INT_MAX) {
        return -EMSGSIZE;
    }
    if (length == 0U) {
        return 0;
    }

    const unsigned int key = irq_lock();
    const uint32_t count = ring_buf_get(&rx_ring, static_cast<uint8_t *>(data), static_cast<uint32_t>(length));
    irq_unlock(key);
    return static_cast<int>(count);
}

int usb_port_transmit(const void *data, size_t length)
{
    return bsp_usb_transmit(data, length);
}

bool usb_port_connected()
{
    return bsp_usb_connected();
}

uint32_t usb_port_received_count()
{
    return static_cast<uint32_t>(atomic_get(&received_count));
}

uint32_t usb_port_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&dropped_count));
}
