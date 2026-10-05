#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

struct dt7_sample
{
    uint16_t channel[4];
    uint8_t switch_left;
    uint8_t switch_right;
};

class dt7
{
    public:
    static constexpr size_t frame_size = 18U;

    static int decode_frame(const uint8_t *frame, size_t length, dt7_sample &sample);
};
