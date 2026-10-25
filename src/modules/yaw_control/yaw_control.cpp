#include "yaw_control.hpp"

#include <string.h>
#include "type_math.hpp"

#if defined(ROBOT_BOARD_H)

namespace
{
YawControl yaw;
K_THREAD_STACK_DEFINE(yaw_stack, 2048);
K_SEM_DEFINE(yaw_sem, 0, 1);
}

void YawControl::timer_callback(struct k_timer *)
{
	k_sem_give(&yaw_sem);
}

void YawControl::task_trampoline(void *context, void *, void *)
{
	static_cast<YawControl *>(context)->Run();
}

void YawControl::control_step()
{
	const uint32_t now_ms = k_uptime_get_32();
	yaw_setpoint_s request{};
	const bool command_fresh = _setpoint_sub.copy(&request) && request.valid
				   && robot::fresh(request.timestamp, _config.command_timeout_ms * 1000ULL);
	const bool feedback_fresh = _config.motor->refresh(_config.feedback_timeout_ms);
	vehicle_attitude_s angles{};
	const bool imu_fresh = _attitude_sub.copy(&angles) && angles.valid
			       && robot::fresh(angles.timestamp_sample, _config.attitude_timeout_ms * 1000ULL);
	const bool active = command_fresh && feedback_fresh && imu_fresh;
	const uint32_t elapsed_ms = _last_control_ms == 0 ? 1U : now_ms - _last_control_ms;
	const float dt_s = static_cast<float>(elapsed_ms < 1U ? 1U : elapsed_ms > 20U ? 20U : elapsed_ms) * 0.001f;
	_last_control_ms = now_ms;
	float current = 0.0f;
	float target_angle = 0.0f;
	const auto &data = _config.motor->measurement();

	if (command_fresh) {
		target_angle = request.angle_rad;
	}

	current = _controller.update(active, target_angle, angles.yaw_rad, data.speed_rad_s, dt_s);
	(void)_config.motor->set_current(current);

	if (active || _control_active != active || now_ms - _last_send_ms >= 20U) {
		(void)_config.motor->send();
		_last_send_ms = now_ms;
	}

	_control_active = active;
}

void YawControl::Run()
{
	while (1) {
		k_sem_take(&yaw_sem, K_FOREVER);
		control_step();
	}
}

int YawControl::init(const YawControlConfig &config)
{
	if (_initialized) {
		return 0;
	}

	_config = config;

	if (_config.motor == nullptr || _config.period_ms == 0 || _config.motor->init() != 0) { return -ENODEV; }

	_controller.configure(_config.position_pid, _config.speed_pid);
	(void)_config.motor->set_current(0.0f);

	k_timer_init(&_timer, timer_callback, nullptr);
	k_timer_start(&_timer, K_MSEC(_config.period_ms), K_MSEC(_config.period_ms));
	k_thread_create(&_thread, yaw_stack, K_THREAD_STACK_SIZEOF(yaw_stack),
			task_trampoline, this, nullptr, nullptr,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	k_thread_name_set(&_thread, "yaw_c620");
	_initialized = true;
	return 0;
}

int app_yaw_init(const YawControlConfig &config)
{
	return yaw.init(config);
}

#endif
