#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/sys/byteorder.h>

#include "bsp_fdcan.hpp"

namespace dji_motor
{

    bool valid_device(fdcan_device device);

    bool valid_id(int motor_id);

    uint16_t read_u16(const uint8_t *data);

    int16_t signed_value(uint16_t raw);

    int validate_feedback(const fdcan_frame &frame, uint8_t motor_id);

    int set(fdcan_device device, uint8_t motor_id, int16_t raw);

    int build_control_frame(fdcan_device device, uint8_t motor_id, fdcan_frame &frame);

}

