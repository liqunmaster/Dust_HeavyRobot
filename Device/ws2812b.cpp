#include "ws2812b.hpp"

int ws2812b_init()
{
    return bsp_pwm_init(1250);
}

void ws2812b_set_color(Ws2812bColor color)
{
    const int ret = ws2812b_init();
    if (ret != 0) {
        return;
    }

    const uint8_t grb[3] = {color.green, color.red, color.blue};
    uint16_t high_ns[24]{};
    for (size_t byte = 0; byte < 3; ++byte) {
        for (size_t bit = 0; bit < 8; ++bit) {
            high_ns[byte * 8 + bit] = (grb[byte] & (1 << (7 - bit))) != 0 ? 700 : 350;
        }
    }

    bsp_pwm_write(high_ns, 24);
}

void ws2812b_off()
{
    ws2812b_set_color({0, 0, 0});
}
