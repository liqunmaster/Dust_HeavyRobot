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
	atomic_t _latest_permille = -1;
	float _filtered_percent = 0.0f;
	bool _has_filtered = false;
	uint32_t _last_sample_ms = 0;
	bool _sampled = false;
};

class HealthMonitor final
{
public:
	void Poll();
	void Run();

private:
	static void TimerCallback(struct k_timer *timer);
	struct k_sem _sem {};
	struct k_timer _timer {};
};

float cpu_usage_latest_percent();
void cpu_usage_supervisor();
void health_monitor_poll();
void health_monitor_run();
