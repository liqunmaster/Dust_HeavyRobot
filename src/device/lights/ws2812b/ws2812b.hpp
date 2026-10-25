#pragma once

#include <stdint.h>

#include "dvc_ws2812b.hpp"

struct Ws2812bColor {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

int ws2812b_init();

// 非阻塞提交；返回 -EBUSY 时保持正在发送的帧不变。
int ws2812b_set_color(Ws2812bColor color);

int ws2812b_off();
