#pragma once

#include <stdint.h>

#include "bsp_pwm.hpp"

struct Ws2812bColor
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

int ws2812b_init();

void ws2812b_set_color(Ws2812bColor color);

void ws2812b_off();
