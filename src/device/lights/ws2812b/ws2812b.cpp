#include "ws2812b.hpp"

int ws2812b_init()
{
	return bsp_ws2812b_init();
}

int ws2812b_set_color(Ws2812bColor color)
{
	return bsp_ws2812b_write(color.red, color.green, color.blue);
}

int ws2812b_off()
{
	return ws2812b_set_color({0, 0, 0});
}
