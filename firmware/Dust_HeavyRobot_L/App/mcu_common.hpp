#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "canfd_bus.hpp"
#include "mcu.hpp"

#include "board_input.hpp"

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
    void ProcessLFrame(const PlatformCanfdSnapshot &snapshot);

    void SendLFeedback();
    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    BoardInput input_{};

    struct k_thread mcu_thread_{};
    struct k_timer mcu_timer_{};
    atomic_t received_count_{};
    atomic_t transmit_error_count_{};
    atomic_t online_{};
    bool initialized_ = false;
    uint32_t last_rx_timestamp_ms_ = 0;
    uint32_t last_rx_sequence_[3]{};
    uint32_t last_chassis_timestamp_ms_ = 0;
    bool chassis_expired_ = true;
    bool command_expired_ = true;
    uint32_t last_command_timestamp_ms_ = 0;
    PlatformCanfdRxHandle command_handles_[3]{};
};

void mcu_common_init();

bool mcu_common_online();

uint32_t mcu_common_received_count();

uint32_t mcu_common_transmit_error_count();
