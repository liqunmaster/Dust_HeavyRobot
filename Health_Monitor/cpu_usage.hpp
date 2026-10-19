#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

class CpuUsage final
{
    public:
    void DisplayOnWs2812b();
    int Sample(float &percent);
    float GetLatestPercent() const;
    void Poll();

    private:
    atomic_t latest_permille_ = -1;
    float filtered_percent_ = 0.0f;
    bool has_filtered_ = false;
    uint32_t last_sample_ms_ = 0;
    bool sampled_ = false;
};

class HealthMonitor final
{
    public:
    void Poll();
    void Run();

    private:
    static void TimerCallback(struct k_timer *timer);
    struct k_sem sem_{};
    struct k_timer timer_{};
};

float cpu_usage_latest_percent();
void cpu_usage_supervisor();
void health_monitor_poll();
void health_monitor_run();
