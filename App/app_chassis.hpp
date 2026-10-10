#pragma once

#include <cmath>

#include <zephyr/kernel.h>

#include "control.hpp"
#include "dji_c620.hpp"
#include "fdcan_channel.hpp"
#include "fdcan_port.hpp"
#include "remote_channel.hpp"


namespace chassis_config
{
    // Replace these measured dimensions and motor signs before driving the chassis.
    constexpr float gear_ratio_num = 3591.0f;
    constexpr float gear_ratio_den = 187.0f;
    constexpr float wheel_radius_m = 0.075f;
    constexpr float half_length_m = 0.20f;
    constexpr float half_width_m = 0.076f;
    constexpr float wheel_direction_fl = 1.0f;
    constexpr float wheel_direction_fr = 1.0f;
    constexpr float wheel_direction_rl = 1.0f;
    constexpr float wheel_direction_rr = 1.0f;

    // C620 feedback omega is a signed 16-bit motor-side rpm value. Convert its
    // full representable range to the wheel/output shaft through the gearbox.
    constexpr float max_motor_rpm = 32767.0f;
    constexpr float max_wheel_rad_s = max_motor_rpm * 6.28318530718f / (60.0f * (3591.0f / 187.0f));
    constexpr float speed_kp = 3.0f;
    constexpr float speed_ki = 0.08f;
    constexpr float speed_kd = 0.0f;
    constexpr float max_current_a = 8.0f;
}

void app_chassis_init();

// Body axes: +x forward, +y left, +yaw counterclockwise.
void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s);
