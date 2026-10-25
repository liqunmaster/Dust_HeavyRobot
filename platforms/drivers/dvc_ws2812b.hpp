#pragma once

#include <stdint.h>

int bsp_ws2812b_init();

// 立即返回：0 表示已提交，-EBUSY 表示上一帧仍在发送。
// 出错后端口保持关闭直到重启；发送中的 DMA 缓冲区不会被复用。
int bsp_ws2812b_write(uint8_t red, uint8_t green, uint8_t blue);
