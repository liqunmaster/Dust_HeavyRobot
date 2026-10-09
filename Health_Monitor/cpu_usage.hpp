#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/cpu_load.h>

#include "ws2812b.hpp"

class cpu_usage final
{
    public:
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

    int sample(float &percent)
    {
        const int permille = cpu_load_get(true);
        if (permille < 0) {
            return permille;
        }
        percent = static_cast<float>(permille) * 0.1F;
        return 0;
    }

    private:
    float filtered_percent_{0.0f};
    bool has_filtered_{false};
};

inline void cpu_usage_supervisor()
{
    static cpu_usage usage;
    static uint32_t last_sample_ms = 0U;
    static bool sampled = false;
    const uint32_t now_ms = k_uptime_get_32();
    if (!sampled || now_ms - last_sample_ms >= 500) {
        sampled = true;
        last_sample_ms = now_ms;
        usage.display_on_ws2812b();
    }
}
