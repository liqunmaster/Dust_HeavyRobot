#pragma once

#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "ring_buffer.hpp"

// UART DMA 缓冲区结构体，保存双缓冲的接收缓冲与发送缓冲
struct UartDmaBuffers
{
    uint8_t rx[2][128];
    uint8_t tx[128];
};

// UART 驱动程序类，基于 DMA 提供非阻塞收发与环形缓冲
class Uart
{
    public:
    static constexpr size_t rx_capacity = 512;
    static constexpr size_t tx_capacity = 128;
    static constexpr size_t dma_capacity = 128;

    /**
     * @brief 构造函数，绑定 Zephyr UART 设备与 DMA 缓冲区
     *
     * @param device Zephyr UART 设备句柄
     * @param dma DMA 缓冲区引用
    */
    constexpr Uart(const struct device *device, UartDmaBuffers &dma)
        : device_(device), dma_(dma) {}

    Uart(const Uart &) = delete;
    Uart &operator=(const Uart &) = delete;
    Uart(Uart &&) = delete;
    Uart &operator=(Uart &&) = delete;

    int init();
    int receive(void *data, size_t length);
    int transmit(const void *data, size_t length);
    bool is_ready() const;

    /**
     * @brief 获取绑定的 Zephyr UART 设备句柄
     *
     * @return 设备句柄
    */
    const struct device *device() const { return device_; }

    private:
    static void uart_callback(const struct device *device, struct uart_event *event, void *user_data);
    void handle_event(const struct uart_event &event);
    int start_rx();

    const struct device *const device_;
    UartDmaBuffers &dma_;
    ring_buffer_t rx_buffer_{};
    ring_buffer_t tx_buffer_{};
    uint8_t rx_storage_[rx_capacity]{};
    uint8_t tx_storage_[tx_capacity]{};
    bool rx_owned_[2]{};
    bool rx_active_ = false;

    atomic_t init_state_ = 0;
    atomic_t rx_overflow_ = 0;
    atomic_t tx_busy_ = 0;
    size_t tx_count_ = 0;
    int tx_error_ = 0;
    struct k_mutex tx_lock_{};
    struct k_sem tx_done_{};
};

extern Uart uart0;
extern Uart uart1;
extern Uart uart2;
extern Uart uart3;
extern Uart uart4;
extern Uart uart5;
extern Uart uart6;

Uart *bsp_uart_get(const struct device *device);

void bsp_uart_init(const struct device *device = nullptr);

int bsp_uart_receive(void *data, size_t length, const struct device *device = nullptr);

int bsp_uart_transmit(const void *data, size_t length, const struct device *device = nullptr);