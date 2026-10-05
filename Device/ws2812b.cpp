#include "ws2812b.hpp"

namespace
{
    constexpr uint32_t bit_period_ns = 1250U;

    constexpr uint16_t zero_high_ns = 350U;

    constexpr uint16_t one_high_ns = 700U;

    constexpr size_t bits_per_led = 24U;
}

int ws2812b_init()
{
    return bsp_pwm_init(bit_period_ns);
}

int ws2812b_set_color(ws2812b_color color)
{
    const int ret = ws2812b_init();
    if (ret != 0) {
        return ret;
    }

    const uint8_t grb[3] = {color.green, color.red, color.blue};
    uint16_t high_ns[bits_per_led]{};
    for (size_t byte = 0U; byte < 3U; ++byte) {
        for (size_t bit = 0U; bit < 8U; ++bit) {
            high_ns[byte * 8U + bit] = (grb[byte] & (1U << (7U - bit))) != 0U ? one_high_ns : zero_high_ns;
        }
    }

    return bsp_pwm_write(high_ns, bits_per_led);
}

int ws2812b_off()
{
    return ws2812b_set_color({0U, 0U, 0U});
}
