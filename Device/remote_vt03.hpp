#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "crc.hpp"
#include "remote_types.hpp"

struct vt03_sample
{
    uint16_t channel[4];
    uint8_t mode_switch;
    bool pause;
    bool custom_left;
    bool custom_right;
    uint16_t wheel;
    bool trigger;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left;
    bool mouse_right;
    bool mouse_middle;
    remote_keyboard keyboard;
};

class vt03
{
    public:
    static constexpr size_t frame_size = 21U;
    static int decode_frame(const uint8_t *frame, size_t length, vt03_sample &sample);
};
