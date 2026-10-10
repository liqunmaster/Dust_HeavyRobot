#include "ws2812b.hpp"

namespace
{
    constexpr uint32_t bit_period_ns = 1250;

    constexpr uint16_t zero_high_ns = 350;

    constexpr uint16_t one_high_ns = 700;

    constexpr size_t bits_per_led = 24;
}

    /**
     * @brief 初始化 WS2812B 所需的 PWM，设置位周期
     *
     * @return bsp_pwm_init 的返回值，0 表示成功
    */
    int ws2812b_init()
{
    return bsp_pwm_init(bit_period_ns);
}

    /**
     * @brief 将颜色编码为 GRB 位序列并通过 PWM 输出到灯珠
     *
     * @param color 目标颜色
    */
    void ws2812b_set_color(ws2812b_color color)
{
    const int ret = ws2812b_init();
    if (ret != 0) {
        return;
    }

    const uint8_t grb[3] = {color.green, color.red, color.blue};
    uint16_t high_ns[bits_per_led]{};
    for (size_t byte = 0; byte < 3; ++byte) {
        for (size_t bit = 0; bit < 8; ++bit) {
            high_ns[byte * 8 + bit] = (grb[byte] & (1 << (7 - bit))) != 0 ? one_high_ns : zero_high_ns;
        }
    }

    bsp_pwm_write(high_ns, bits_per_led);
}

    /**
     * @brief 关闭 WS2812B 灯珠（输出全黑）
     *
    */
    void ws2812b_off()
{
    ws2812b_set_color({0, 0, 0});
}