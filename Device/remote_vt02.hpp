#pragma once

#include <errno.h>
#include <string.h>
#include "crc.hpp"
#include "remote_types.hpp"

struct vt02_sample
{
    int16_t mouse_x, mouse_y, mouse_z;
    bool mouse_left, mouse_right;
    remote_keyboard keyboard;
    int16_t pulley_wheel;
};

class vt02
{
    public:
    static constexpr size_t header_size = 5U;

    static constexpr size_t frame_size = 21U;

    static constexpr uint16_t command_id = 0x0304U;

    static int frame_length_from_header(const uint8_t *frame, size_t length, size_t &frame_length);

    static int decode_frame(const uint8_t *frame, size_t length, vt02_sample &sample);
};
