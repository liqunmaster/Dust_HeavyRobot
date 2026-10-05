#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "ws2812b.hpp"

class cpu_usage final
{
    public:
    int display_on_ws2812b()
    {
        float percent = 0.0f;
        const int ret = sample(percent);
        if (ret == -EAGAIN) {
            return ret;
        }
        if (ret != 0) {
            (void)ws2812b_off();
            has_filtered_ = false;
            return ret;
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
        return ws2812b_set_color({red, green, 0U});
    }

    int sample(float &percent)
    {
#if defined(CONFIG_SCHED_THREAD_USAGE_ALL)
        k_thread_runtime_stats current{};
        const int ret = k_thread_runtime_stats_all_get(&current);
        if (ret != 0) {
            return ret;
        }

        if (!has_previous_) {
            previous_ = current;
            has_previous_ = true;
            return -EAGAIN;
        }

        if (current.execution_cycles < previous_.execution_cycles || current.total_cycles < previous_.total_cycles || current.idle_cycles < previous_.idle_cycles) {
            previous_ = current;
            return -EAGAIN;
        }

        const uint64_t total = current.execution_cycles - previous_.execution_cycles;
        const uint64_t busy = current.total_cycles - previous_.total_cycles;
        previous_ = current;
        if (total == 0U || busy > total) {
            return -EAGAIN;
        }

        percent = 100.0f * (static_cast<float>(busy) / static_cast<float>(total));
        return 0;
#else
        (void)percent;
        return -ENOTSUP;
#endif
    }

    private:
    float filtered_percent_{0.0f};
    bool has_filtered_{false};
#if defined(CONFIG_SCHED_THREAD_USAGE_ALL)

    k_thread_runtime_stats previous_{};

    bool has_previous_{false};
#endif
};

inline void cpu_usage_supervisor()
{
    static cpu_usage usage;
    (void)usage.display_on_ws2812b();
    k_sleep(K_MSEC(500));
}
