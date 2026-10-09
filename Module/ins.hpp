#pragma once

#include <cmath>
#include <errno.h>

#include <stdint.h>

#include <zephyr/kernel.h>

#include "quaternion.hpp"
#include "type_math.hpp"

struct ins_euler_angles
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_unwrapped_rad;
};

void ins_init();

bool ins_process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds);

void ins_set_axis_rotation(const float rotation[3][3]);

void ins_reset_axis_rotation();

bool ins_get_euler_angles(ins_euler_angles &angles);
