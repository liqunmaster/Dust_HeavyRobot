#include "cpu_usage.hpp"

#include <errno.h>
#include <zephyr/sys/cpu_load.h>

#include "motor_health.hpp"
#include "ws2812b.hpp"

namespace
{
    CpuUsage cpu_usage;
    HealthMonitor health_monitor;
}

void CpuUsage::DisplayOnWs2812b()
{
    float percent = 0.0f;
    const int ret = Sample(percent);
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

int CpuUsage::Sample(float &percent)
{
    const int permille = cpu_load_get(true);
    if (permille < 0) {
        return permille;
    }
    atomic_set(&latest_permille_, permille);
    percent = static_cast<float>(permille) * 0.1f;
    return 0;
}

float CpuUsage::GetLatestPercent() const
{
    const atomic_val_t permille = atomic_get(&latest_permille_);
    return permille < 0 ? -1.0f : static_cast<float>(permille) * 0.1f;
}

void CpuUsage::Poll()
{
    const uint32_t now_ms = k_uptime_get_32();
    if (!sampled_) {

        (void)cpu_load_get(true);
        sampled_ = true;
        last_sample_ms_ = now_ms;
        return;
    }
    if (now_ms - last_sample_ms_ >= 500) {
        last_sample_ms_ = now_ms;
        DisplayOnWs2812b();
    }
}

void HealthMonitor::TimerCallback(struct k_timer *timer)
{
    auto *monitor = static_cast<HealthMonitor *>(k_timer_user_data_get(timer));
    k_sem_give(&monitor->sem_);
}

void HealthMonitor::Poll()
{
    motor_health_poll();
    cpu_usage_supervisor();
}

void HealthMonitor::Run()
{
    k_sem_init(&sem_, 0, 1);
    k_timer_init(&timer_, TimerCallback, nullptr);
    k_timer_user_data_set(&timer_, this);
    k_timer_start(&timer_, K_MSEC(10), K_MSEC(10));
    while (1) {
        k_sem_take(&sem_, K_FOREVER);
        Poll();
    }
}

float cpu_usage_latest_percent()
{
    return cpu_usage.GetLatestPercent();
}

void cpu_usage_supervisor()
{
    cpu_usage.Poll();
}

void health_monitor_poll()
{
    health_monitor.Poll();
}

void health_monitor_run()
{
    health_monitor.Run();
}
