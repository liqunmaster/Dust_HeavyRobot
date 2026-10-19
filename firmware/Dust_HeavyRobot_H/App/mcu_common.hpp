#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "canfd_bus.hpp"
#include "mcu.hpp"

#include "ins.hpp"
#include "input.hpp"

class McuCommon
{
    public:
    void Init();

    bool Online();

    uint32_t ReceivedCount();

    uint32_t TransmitErrorCount();

    private:
    static void TimerCallback(struct k_timer *arg1);

    int SendFrame(uint32_t id, const uint8_t payload[8]);
    void SendHCommandFrames();
    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    struct k_thread mcu_thread_{};
    struct k_timer mcu_timer_{};
    atomic_t received_count_{};
    atomic_t transmit_error_count_{};
    atomic_t online_{};
    bool initialized_ = false;
    uint32_t last_rx_timestamp_ms_ = 0;
    uint32_t last_rx_sequence_[3]{};
    uint8_t transmit_round_ = 0;
    PlatformCanfdRxHandle feedback_handle_{};
};

void mcu_common_init();

bool mcu_common_online();

uint32_t mcu_common_received_count();

uint32_t mcu_common_transmit_error_count();
