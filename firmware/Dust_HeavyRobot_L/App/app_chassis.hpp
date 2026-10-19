#pragma once

#include <cmath>
#include <zephyr/kernel.h>
#include "control.hpp"
#include "canfd_bus.hpp"
#include "dji_c620.hpp"
#include "motor_channel.hpp"
#include "motor_port.hpp"
#include "remote_channel.hpp"
#include "chassis_channel.hpp"

class Chassis
{
    public:
    void Init();

    void SetVelocity(float vx_m_s, float vy_m_s, float yaw_rad_s);

    private:
    void MecanumInverse(float vx_m_s, float vy_m_s, float yaw_rad_s, float wheel_rad_s[4]);

    void MecanumLimitWheelSpeeds(float wheel_rad_s[4], float limit_rad_s);

    static void TimerCallback(struct k_timer *arg1);

    void ControlStep(float dt_s);

    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    C620 motors_[4]{};
    PlatformCanfdRxHandle feedback_handles_[4]{};
    uint32_t feedback_sequences_[4]{};
    bool feedback_valid_[4]{};
    SpeedLoop speed_loops_[4]{};
    struct k_thread chassis_thread_{};
    struct k_timer chassis_timer_{};
    bool initialized_ = false;
    bool offline_command_sent_ = false;
    uint32_t last_offline_command_ms_ = 0;
};

void app_chassis_init();

void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s);
