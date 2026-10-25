#pragma once

#include <stdint.h>

int bsp_ws2812b_init();

// Returns immediately: 0 = submitted, -EBUSY = previous frame still active.
// Errors latch the port off until reboot; an active DMA buffer is never reused.
int bsp_ws2812b_write(uint8_t red, uint8_t green, uint8_t blue);

