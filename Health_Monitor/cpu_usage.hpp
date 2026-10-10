#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/cpu_load.h>

#include "ws2812b.hpp"

// 通过 WS2812B 灯带显示 CPU 占用率的工具类
class cpu_usage final
{
    public:
    /**
     * @brief 对采样到的 CPU 占用率做低通滤波，并映射为红绿颜色显示到 WS2812B
     *
    */
    void display_on_ws2812b()
    {
        float percent = 0.0f;
        const int ret = sample(percent);
        if (ret == -EAGAIN) {
            return;
        }
        if (ret != 0) {
            ws2812b_off();
            has_filtered_ = false;
            return;
        }

        if (has_filtered_) {
            filtered_percent_ += 0.25f * (percent - filtered_percent_);
        } else {
            filtered_percent_ = percent;
            has_filtered_ = true;
        }

        constexpr float brightness = 126.0f;
        const uint8_t red = static_cast<uint8_t>(filtered_percent_ * brightness / 100.0f);
        const uint8_t green = static_cast<uint8_t>((100.0f - filtered_percent_) * brightness / 100.0f);
        ws2812b_set_color({red, green, 0});
    }

    /**
     * @brief 采样当前 CPU 负载率（千分比换算为百分比）
     *
     * @param percent 输出 CPU 占用百分比
     * @return 成功返回 0；数据不足返回 -EAGAIN，否则返回负错误码
    */
    int sample(float &percent)
    {
        const int permille = cpu_load_get(true);
        if (permille < 0) {
            return permille;
        }
        percent = static_cast<float>(permille) * 0.1f;
        return 0;
    }

    private:
    float filtered_percent_{0.0f};
    bool has_filtered_{false};
};

/**
 * @brief CPU 占用监控巡检入口：按采样间隔定期刷新灯带显示
 *
*/
inline void cpu_usage_supervisor()
{
    static cpu_usage usage;
    static uint32_t last_sample_ms = 0;
    static bool sampled = false;
    const uint32_t now_ms = k_uptime_get_32();
    if (!sampled || now_ms - last_sample_ms >= 500) {
        sampled = true;
        last_sample_ms = now_ms;
        usage.display_on_ws2812b();
    }
}