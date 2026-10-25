#pragma once

#include <stdint.h>

#include "bsp_ws2812b.hpp"

struct Ws2812bColor
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

int ws2812b_init();

// Nonblocking submission; -EBUSY leaves the active frame untouched.
int ws2812b_set_color(Ws2812bColor color);

int ws2812b_off();
