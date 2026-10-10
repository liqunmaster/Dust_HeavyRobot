#include "bsp_uart.hpp"

extern "C" void __stdout_hook_install(int (*hook)(int));

namespace
{

    uart_dma_buffers dma_buffers[7] __attribute__((section("AHB_SRAM"), aligned(4)));

    /**
     * @brief 标准输出字符写入钩子 将字符经 uart3 发送
     *
     * @param character 待写入的字符
     * @return 发送成功返回原字符 失败返回 EOF
     */
    int stdout_character(int character)
    {
        const uint8_t byte = static_cast<uint8_t>(character);
        return uart3.transmit(&byte, 1) == 1 ? byte : EOF;
    }
}
uart uart0{DEVICE_DT_GET(DT_NODELABEL(uart0)), dma_buffers[0]};
uart uart1{DEVICE_DT_GET(DT_NODELABEL(uart1)), dma_buffers[1]};
uart uart2{DEVICE_DT_GET(DT_NODELABEL(uart2)), dma_buffers[2]};
uart uart3{DEVICE_DT_GET(DT_NODELABEL(uart3)), dma_buffers[3]};
uart uart4{DEVICE_DT_GET(DT_NODELABEL(uart4)), dma_buffers[4]};
uart uart5{DEVICE_DT_GET(DT_NODELABEL(uart5)), dma_buffers[5]};
uart uart6{DEVICE_DT_GET(DT_NODELABEL(uart6)), dma_buffers[6]};

/**
 * @brief 根据 Zephyr 设备句柄查找对应的 uart 对象
 *
 * @param device Zephyr UART 设备句柄 为空时返回默认的 uart3
 * @return 匹配的 uart 对象指针 未找到返回 nullptr
 */
uart *bsp_uart_get(const struct device *device)
{
    if (device == nullptr) {
        return &uart3;
    }
    uart *const ports[] = {&uart0, &uart1, &uart2, &uart3, &uart4, &uart5, &uart6};
    for (auto *port : ports) {
        if (port->device() == device) {
            return port;
        }
    }
    return nullptr;
}

/**
 * @brief 查询串口是否已完成初始化
 *
 * @return 初始化完成返回 true 否则返回 false
 */
bool uart::is_ready() const
{
    return atomic_get(&init_state_) == 2;
}

/**
 * @brief 初始化串口）
 *
 * @return 成功返回 0 失败返回负错误码
 */
int uart::init()
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (is_ready()) {
        return 0;
    }
    if (!atomic_cas(&init_state_, 0, 1)) {
        return -EBUSY;
    }
    if (device_ == nullptr || !device_is_ready(device_)) {
        atomic_clear(&init_state_);
        return -ENODEV;
    }
    k_mutex_init(&tx_lock_);
    k_sem_init(&tx_done_, 0, 1);
    ring_buffer_init(&rx_buffer_, rx_storage_, sizeof(rx_storage_));
    ring_buffer_init(&tx_buffer_, tx_storage_, sizeof(tx_storage_));
    atomic_clear(&rx_overflow_);
    atomic_clear(&tx_busy_);
    rx_active_ = false;
    rx_owned_[0] = false;
    rx_owned_[1] = false;

    int result = uart_callback_set(device_, uart_callback, this);
    if (result != 0) {
        atomic_clear(&init_state_);
        return result;
    }

    if (this != &uart3) {
        result = start_rx();
        if (result != 0) {
            (void)uart_callback_set(device_, nullptr, nullptr);
            atomic_clear(&init_state_);
            return result;
        }
    }
    atomic_set(&init_state_, 2);
    if (this == &uart3) {
        __stdout_hook_install(stdout_character);
    }
    return 0;
}

/**
 * @brief Zephyr UART 驱动回调 锁中断后将事件交给成员处理
 *
 * @param device Zephyr UART 设备句柄
 * @param event UART 事件
 * @param user_data 透传的 uart 对象指针
 */
void uart::uart_callback(const struct device *device, struct uart_event *event, void *user_data)
{
    auto *port = static_cast<uart *>(user_data);
    if (port == nullptr || event == nullptr || device != port->device_) {
        return;
    }
    const unsigned int key = irq_lock();
    port->handle_event(*event);
    irq_unlock(key);
}

/**
 * @brief 处理各类 UART 事件
 *
 * @param event UART 事件
 */
void uart::handle_event(const struct uart_event &event)
{
    switch (event.type) {
        case UART_RX_RDY:
        if (ring_buffer_write(&rx_buffer_, event.data.rx.buf + event.data.rx.offset, event.data.rx.len) != event.data.rx.len) {
            atomic_set(&rx_overflow_, 1);
        }
        break;
        case UART_RX_BUF_REQUEST:
        for (size_t i = 0; i < 2; ++i) {
            if (!rx_owned_[i]) {
                rx_owned_[i] = true;
                if (uart_rx_buf_rsp(device_, dma_.rx[i], sizeof(dma_.rx[i])) != 0) {
                    rx_owned_[i] = false;
                }
                break;
            }
        }
        break;
        case UART_RX_BUF_RELEASED:
        for (size_t i = 0; i < 2; ++i) {
            if (event.data.rx_buf.buf == dma_.rx[i]) {
                rx_owned_[i] = false;
            }
        }
        break;
        case UART_RX_DISABLED:
        rx_active_ = false;
        atomic_set(&rx_overflow_, 1);
        break;
        case UART_RX_STOPPED:
        atomic_set(&rx_overflow_, 1);
        break;
        case UART_TX_DONE:
        case UART_TX_ABORTED:
        tx_count_ = event.data.tx.len;
        tx_error_ = event.type == UART_TX_DONE ? 0 : -EIO;
        atomic_clear(&tx_busy_);
        k_sem_give(&tx_done_);
        break;
        default:
        break;
    }
}

/**
 * @brief 启动 DMA 接收
 *
 * @return 成功返回 0 失败返回负错误码
 */
int uart::start_rx()
{
    const unsigned int key = irq_lock();
    rx_owned_[0] = true;
    rx_owned_[1] = false;
    rx_active_ = true;
    const int result = uart_rx_enable(device_, dma_.rx[0], sizeof(dma_.rx[0]), 1000);
    if (result != 0) {
        rx_active_ = false;
        rx_owned_[0] = false;
    }
    irq_unlock(key);
    return result;
}

/**
 * @brief 从串口接收数据 阻塞读取至指定长度
 *
 * @param data 接收数据缓冲区
 * @param length 期望接收的字节数
 * @return 实际接收的字节数 失败返回负错误码
 */
int uart::receive(void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!is_ready()) {
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
    const int result = rx_active_ ? 0 : start_rx();
    const bool overflow = atomic_set(&rx_overflow_, 0) != 0;
    if (overflow) {
        ring_buffer_clear(&rx_buffer_);
    }
    const int count = overflow ? -ENOBUFS : static_cast<int>(ring_buffer_read(&rx_buffer_, data, length));
    irq_unlock(key);
    return count != 0 ? count : result;
}

/**
 * @brief 向串口发送数据 分块写入 DMA 并等待发送完成
 *
 * @param data 待发送数据缓冲区
 * @param length 待发送的字节数
 * @return 实际发送的字节数 失败返回负错误码
 */
int uart::transmit(const void *data, size_t length)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!is_ready()) {
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

    k_mutex_lock(&tx_lock_, K_FOREVER);
    const auto *bytes = static_cast<const uint8_t *>(data);
    size_t sent = 0;
    int result = atomic_get(&tx_busy_) ? -EBUSY : 0;
    while (result == 0 && sent < length) {
        const size_t chunk = ring_buffer_write(&tx_buffer_, bytes + sent, length - sent);
        if (chunk == 0) {
            result = -ENOBUFS;
            break;
        }
        ring_buffer_read(&tx_buffer_, dma_.tx, chunk);
        k_sem_reset(&tx_done_);
        tx_count_ = 0;
        tx_error_ = 0;
        atomic_set(&tx_busy_, 1);
        result = uart_tx(device_, dma_.tx, chunk, SYS_FOREVER_US);
        if (result != 0) {
            atomic_clear(&tx_busy_);
            break;
        }
        if (k_sem_take(&tx_done_, K_MSEC(1000)) != 0) {
            const unsigned int key = irq_lock();
            if (atomic_get(&tx_busy_)) {
                (void)uart_tx_abort(device_);
                result = -ETIMEDOUT;
            }
            irq_unlock(key);
        }
        sent += tx_count_;
        if (result == 0) {
            result = tx_error_;
        }
        if (result == 0 && tx_count_ != chunk) {
            result = -EIO;
        }
    }
    k_mutex_unlock(&tx_lock_);
    return sent != 0 ? static_cast<int>(sent) : result;
}

/**
 * @brief 初始化指定设备的串口
 *
 * @param device Zephyr UART 设备句柄 为空时使用默认串口
 */
void bsp_uart_init(const struct device *device)
{
    auto *port = bsp_uart_get(device);
    if (port != nullptr) {
        port->init();
    }
}

/**
 * @brief 从指定串口接收数据
 *
 * @param data 接收缓冲区
 * @param length 期望接收的字节数
 * @param device Zephyr UART 设备句柄 为空时使用默认串口
 * @return 实际接收字节数 失败返回负错误码
 */
int bsp_uart_receive(void *data, size_t length, const struct device *device)
{
    auto *port = bsp_uart_get(device);
    return port != nullptr ? port->receive(data, length) : -ENODEV;
}

/**
 * @brief 向指定串口发送数据
 *
 * @param data 待发送缓冲区
 * @param length 待发送的字节数
 * @param device Zephyr UART 设备句柄 为空时使用默认串口
 * @return 实际发送字节数 失败返回负错误码
 */
int bsp_uart_transmit(const void *data, size_t length, const struct device *device)
{
    auto *port = bsp_uart_get(device);
    return port != nullptr ? port->transmit(data, length) : -ENODEV;
}

/**
 * @brief printf 重定向实现 将格式化字符串经 uart3 输出
 *
 * @param format 格式化字符串
 * @param ... 可变参数
 * @return 输出的字符数 失败返回 EOF
 */
extern "C" int printf(const char *format, ...)
{
    if (k_is_in_isr()) {
        errno = EWOULDBLOCK;
        return EOF;
    }
    char message[256];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (length < 0) {
        return length;
    }
    if (static_cast<size_t>(length) >= sizeof(message)) {
        errno = EMSGSIZE;
        return EOF;
    }
    const int result = uart3.transmit(message, length);
    if (result != length) {
        errno = result < 0 ? -result : EIO;
        return EOF;
    }
    return result;
}
