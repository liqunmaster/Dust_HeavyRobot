#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "motor_channel.hpp"
#include "motor_port.hpp"

class MotorHealth
{
    public:
    void Init();

    void Poll();

    bool IsOffline(size_t index);

    private:
    enum class MotorState : uint8_t
    {
        unknown = 0,
        enabled = 1,
        disabled = 2,
        error = 3,
        offline = 4,
    };
    MotorState ClassifyState(const CanfdMotorHealthData &motor, uint32_t now_ms, uint32_t last_feedback);

    void CheckMotors(uint32_t now_ms, uint32_t elapsed_ms);
    CanfdMotorHealthData motors_[CANFD_PORT_MAX_MOTORS]{};
    bool present_[CANFD_PORT_MAX_MOTORS]{};
    uint32_t last_count_[CANFD_PORT_MAX_MOTORS]{};
    uint32_t last_feedback_ms_[CANFD_PORT_MAX_MOTORS]{};
    uint16_t recovery_ticks_[CANFD_PORT_MAX_MOTORS]{};
    bool registered_[CANFD_PORT_MAX_MOTORS]{};
    atomic_t offline_[CANFD_PORT_MAX_MOTORS]{};
    MotorState states_[CANFD_PORT_MAX_MOTORS]{};
    bool initialized_ = false;
    bool checked_ = false;
    uint32_t last_check_ms_ = 0;
};

void motor_health_init();

void motor_health_poll();

bool motor_health_is_offline(size_t index);
