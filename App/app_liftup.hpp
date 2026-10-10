#pragma once

#include <cmath>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "control.hpp"
#include "cubemars.hpp"
#include "fdcan_channel.hpp"
#include "fdcan_port.hpp"
#include "remote_channel.hpp"

namespace liftup_config
{
    constexpr bool motor_present_can0 = true;
    constexpr bool motor_present_can1 = true;
    constexpr bool motor_present_can2 = false;
    constexpr bool motor_present_can3 = false;
    constexpr float motor_direction_can0 = 1.0f;
    constexpr float motor_direction_can1 = 1.0f;
    constexpr float motor_direction_can2 = 1.0f;
    constexpr float motor_direction_can3 = 1.0f;
    constexpr float max_relative_angle_rad = 12.5f;

    constexpr float motor_angle_max_rad = 12.5f;
    constexpr float max_speed_rad_s = 50.0f;
    constexpr float max_torque_nm = 65.0f;
    constexpr float position_kp = 20.0f;
    constexpr float position_ki = 0.0f;
    constexpr float position_kd = 0.0f;
    constexpr float speed_kp = 2.0f;
    constexpr float speed_ki = 0.0f;
    constexpr float speed_kd = 0.0f;
}

void app_liftup_init();

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad);

// 抬升电机运行状态信息
struct lift_motor_status
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

void app_liftup_get_status(uint8_t can_index, lift_motor_status &status);
