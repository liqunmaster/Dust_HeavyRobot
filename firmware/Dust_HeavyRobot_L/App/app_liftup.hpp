#pragma once

#include <cmath>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "control.hpp"
#include "canfd_bus.hpp"
#include "cubemars.hpp"
#include "motor_channel.hpp"
#include "motor_port.hpp"
#include "remote_channel.hpp"

struct LiftMotorStatus
{
    bool feedback_fresh;
    uint32_t feedback_age_ms;
    uint32_t feedback_count;
    float target_angle_rad;
    float measured_angle_rad;
    float command_torque_nm;
    int control_error;
    uint32_t can_tx_completed;
    uint32_t can_tx_errors;
    uint32_t can_rx_received;
    uint32_t can_rx_dropped;
};

class Liftup
{
    public:
    void Init();

    void SetTarget(uint8_t can_index, float relative_angle_rad);

    void GetStatus(uint8_t can_index, LiftMotorStatus &status);

    private:
    static void TimerCallback(struct k_timer *arg1);

    void ControlStep(float dt_s);

    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    Cubemars motors_[4]{};
    PlatformCanfdRxHandle feedback_handles_[4]{};
    uint32_t feedback_sequences_[4]{};
    bool feedback_valid_[4]{};
    PositionSpeedLoop loops_[4]{};
    float origin_rad_[4]{};
    bool has_origin_[4]{};
    bool initialized_ = false;
    LiftMotorStatus motor_status_[4]{};
    struct k_spinlock status_lock_{};
    struct k_thread lift_thread_{};
    struct k_timer lift_timer_{};
};

void app_liftup_init();

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad);

void app_liftup_get_status(uint8_t can_index, LiftMotorStatus &status);
