#pragma once

#include <stdint.h>

#include "bsp_pwm.hpp"

// WS2812B 灯珠颜色结构体 RGB 各 8 位
struct ws2812b_color
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

int ws2812b_init();

void ws2812b_set_color(ws2812b_color color);

void ws2812b_off();

