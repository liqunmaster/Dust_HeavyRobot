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

struct UartDmaBuffers
{
    uint8_t rx[2][128];
    uint8_t tx[128];
};

class Uart
{
    public:
    static constexpr size_t rx_capacity = 512;
    static constexpr size_t tx_capacity = 128;
    static constexpr size_t dma_capacity = 128;

    constexpr Uart(const struct device *device, UartDmaBuffers &dma)
        : device_(device), dma_(dma) {}

    Uart(const Uart &) = delete;
    Uart &operator=(const Uart &) = delete;
    Uart(Uart &&) = delete;
    Uart &operator=(Uart &&) = delete;

    int Init();
    int Receive(void *data, size_t length);
    int Transmit(const void *data, size_t length);
    int TransmitAsync(const void *data, size_t length);
    int AbortTransmit();
    int ClearRx();
    int SetRxNotify(void (*notify)(void *), void *context);
    int SetTxNotify(void (*notify)(void *, int), void *context);
    bool IsReady() const;

    const struct device *GetDevice() const { return device_; }

    private:
    static void UartCallback(const struct device *device, struct uart_event *event, void *user_data);
    void HandleEvent(const struct uart_event &event);
    int StartRx();

    const struct device *const device_;
    UartDmaBuffers &dma_;
    RingBuffer rx_buffer_{};
    RingBuffer tx_buffer_{};
    uint8_t rx_storage_[rx_capacity]{};
    uint8_t tx_storage_[tx_capacity]{};
    bool rx_owned_[2]{};
    bool rx_active_ = false;
    void (*rx_notify_)(void *) = nullptr;
    void *rx_notify_context_ = nullptr;
    void (*tx_notify_)(void *, int) = nullptr;
    void *tx_notify_context_ = nullptr;

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
