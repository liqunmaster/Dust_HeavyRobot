#pragma once

#include <stdint.h>

#include "bsp_pwm.hpp"

struct ws2812b_color
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

int ws2812b_init();

int ws2812b_set_color(ws2812b_color color);

int ws2812b_off();
