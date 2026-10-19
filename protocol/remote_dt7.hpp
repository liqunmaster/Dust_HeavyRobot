#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

struct Dt7Sample
{
    uint16_t channel[4];
    uint8_t switch_left;
    uint8_t switch_right;
};

class Dt7
{
    public:
    static constexpr size_t frame_size = 18;

    static int DecodeFrame(const uint8_t *frame, size_t length, Dt7Sample &sample);
};
