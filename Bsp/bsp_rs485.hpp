#pragma once
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include "bsp_uart.hpp"

using Rs485Complete = void (*)(void *, int, const uint8_t *, size_t);

class Rs485
{
    public:
    static constexpr size_t response_capacity = 64;
    explicit constexpr Rs485(Uart &port) : port_(&port) {}
    Rs485(const Rs485 &) = delete;
    Rs485 &operator=(const Rs485 &) = delete;
    int Init();
    int Start(const void *request, size_t request_length, size_t response_length,
              uint32_t timeout_ms, Rs485Complete complete, void *context);
    bool IsReady() const;

    private:
    enum class Phase : uint8_t { idle, sending, receiving, completing };
    static void RxNotify(void *context);
    static void TxNotify(void *context, int result);
    static void LineIrq(const struct device *, void *context);
    static void Timeout(struct k_timer *timer);
    static void Work(struct k_work *work);
    void BeginReceive();
    void Finish(int result);
    void Process();
    Uart *const port_;
    struct k_work work_{};
    struct k_timer timer_{};
    atomic_t init_state_ = 0;
    Phase phase_ = Phase::idle;
    Rs485Complete complete_ = nullptr;
    void *context_ = nullptr;
    uint8_t response_[response_capacity]{};
    size_t response_length_ = 0;
    size_t received_ = 0;
    uint32_t response_timeout_ms_ = 0;
    int result_ = 0;
    uint32_t received_bytes_ = 0;
    uint32_t transaction_count_ = 0;
    uint32_t timeout_count_ = 0;
    size_t last_response_length_ = 0;
};
extern Rs485 rs485_uart5;
int bsp_rs485_init(const struct device *device = nullptr);

int bsp_rs485_start(const void *request, size_t request_length, size_t response_length,
                    uint32_t timeout_ms, Rs485Complete complete, void *context);
bool bsp_rs485_is_ready();
