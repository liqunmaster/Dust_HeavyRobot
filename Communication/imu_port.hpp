#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "icm42688p_hxy.hpp"
#include "ins.hpp"
#include "type_math.hpp"

using imu_sample = icm42688phxy_sample;

void imu_port_init();

int imu_port_get_sample(imu_sample &sample);

uint32_t imu_port_feedback_count();
