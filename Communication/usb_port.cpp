#include "usb_port.hpp"

namespace
{
    constexpr uint32_t rx_capacity = 2048;
    uint8_t rx_storage[rx_capacity];
    struct ring_buf rx_ring;
    K_THREAD_STACK_DEFINE(usb_stack, 1024);
    struct k_thread usb_thread;
    atomic_t received_count;
    atomic_t dropped_count;
    bool ready;

    /**
     * @brief 从 USB 底层读取数据并写入环形缓冲区，统计收包与丢包数量
     *
    */
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

    /**
     * @brief USB 接收线程入口：周期性等待事件、驱动底层并处理接收数据
     *
    */
    void usb_thread_entry(void *, void *, void *)
    {
        while (1) {
            bsp_usb_wait_event(K_MSEC(10));
            bsp_usb_task();
            usb_port_process();
        }
    }
}
/**
 * @brief 初始化 USB 端口并启动接收线程；已初始化或在中断上下文则直接返回
 *
*/
void usb_port_init()
{
    if (k_is_in_isr()) {
        return;
    }
    if (ready) {
        return;
    }
    const int ret = bsp_usb_init();
    if (ret != 0) {
        return;
    }

    ring_buf_init(&rx_ring, sizeof(rx_storage), rx_storage);
    ready = true;
    k_thread_create(&usb_thread, usb_stack, K_THREAD_STACK_SIZEOF(usb_stack), usb_thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

/**
 * @brief 从接收环形缓冲区拷贝最多 length 字节到 data
 *
 * @param data   目标缓冲区；长度为 0 时可为空
 * @param length 期望读取的字节数
 * @return 实际读取的字节数；未就绪返回 -ENODEV，中断上下文返回 -EWOULDBLOCK
*/
int usb_port_receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!ready) {
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
    const uint32_t count = ring_buf_get(&rx_ring, static_cast<uint8_t *>(data), static_cast<uint32_t>(length));
    irq_unlock(key);
    return static_cast<int>(count);
}

/**
 * @brief 通过 USB 底层发送 length 字节
 *
 * @param data   待发送的数据
 * @param length 发送的字节数
 * @return 底层发送接口的返回值，0 表示成功
*/
int usb_port_transmit(const void *data, size_t length)
{
    return bsp_usb_transmit(data, length);
}

/**
 * @brief 查询 USB 是否处于连接状态
 *
 * @return 已连接返回 true，否则返回 false
*/
bool usb_port_connected()
{
    return bsp_usb_connected();
}

/**
 * @brief 获取累计成功接收的字节数
 *
 * @return 累计接收字节数
*/
uint32_t usb_port_received_count()
{
    return static_cast<uint32_t>(atomic_get(&received_count));
}

/**
 * @brief 获取由于缓冲区溢出而丢弃的字节数
 *
 * @return 累计丢弃字节数
*/
uint32_t usb_port_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&dropped_count));
}