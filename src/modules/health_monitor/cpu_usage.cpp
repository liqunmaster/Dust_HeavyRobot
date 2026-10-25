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
		_has_filtered = false;
		return;
	}

	if (_has_filtered) {
		_filtered_percent += 0.25f * (percent - _filtered_percent);

	} else {
		_filtered_percent = percent;
		_has_filtered = true;
	}

	_filtered_percent = _filtered_percent < 0.0f ? 0.0f :
			    _filtered_percent > 100.0f ? 100.0f : _filtered_percent;

	if (_filtered_percent < 50.0f) {
		ws2812b_set_color({0, 126, 0});

	} else if (_filtered_percent <= 75.0f) {
		ws2812b_set_color({126, 126, 0});

	} else {
		ws2812b_set_color({126, 0, 0});
	}
}

int CpuUsage::Sample(float &percent)
{
	const int permille = cpu_load_get(true);

	if (permille < 0) {
		return permille;
	}

	atomic_set(&_latest_permille, permille);
	percent = static_cast<float>(permille) * 0.1f;
	return 0;
}

float CpuUsage::GetLatestPercent() const
{
	const atomic_val_t permille = atomic_get(&_latest_permille);
	return permille < 0 ? -1.0f : static_cast<float>(permille) * 0.1f;
}

void CpuUsage::Poll()
{
	const uint32_t now_ms = k_uptime_get_32();

	if (!_sampled) {

		(void)cpu_load_get(true);
		_sampled = true;
		_last_sample_ms = now_ms;
		return;
	}

	if (now_ms - _last_sample_ms >= 500) {
		_last_sample_ms = now_ms;
		DisplayOnWs2812b();
	}
}

void HealthMonitor::TimerCallback(struct k_timer *timer)
{
	auto *monitor = static_cast<HealthMonitor *>(k_timer_user_data_get(timer));
	k_sem_give(&monitor->_sem);
}

void HealthMonitor::Poll()
{
	motor_health_poll();
	cpu_usage_supervisor();
}

void HealthMonitor::Run()
{
	k_sem_init(&_sem, 0, 1);
	k_timer_init(&_timer, TimerCallback, nullptr);
	k_timer_user_data_set(&_timer, this);
	k_timer_start(&_timer, K_MSEC(10), K_MSEC(10));

	while (1) {
		k_sem_take(&_sem, K_FOREVER);
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
