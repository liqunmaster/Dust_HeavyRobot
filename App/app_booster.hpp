#pragma once

#include <cmath>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "dm_motor.hpp"
#include "fdcan_channel.hpp"
#include "fdcan_port.hpp"
#include "pid.hpp"


enum class BoosterState : uint8_t
{
    disabled,
    ready,
    feeding,
    reversing,
    settling,
    jam_fault,
    motor_fault,
};

struct BoosterStatus
{
    BoosterState state;
    bool feedback_fresh;
    uint8_t motor_status;
    uint32_t feedback_age_ms;
    float angle_rad;
    float target_angle_rad;
    float speed_rad_s;
    float measured_torque_nm;
    float command_torque_nm;
    uint8_t jam_retries;
    int last_error;
};

void app_booster_init();

void app_booster_get_status(BoosterStatus &status);
