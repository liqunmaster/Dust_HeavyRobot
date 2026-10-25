#include "chassis_control.hpp"

#include <string.h>

namespace
{
ChassisControl chassis;

K_THREAD_STACK_DEFINE(chassis_task, 2048);

K_SEM_DEFINE(chassis_sem, 0, 1);
}

void ChassisControl::timer_callback(struct k_timer *arg1)
{
	k_sem_give(&chassis_sem);
}

void ChassisControl::control_step(float dt_s)
{
	chassis_setpoint_s request{};
	const uint32_t now_ms = k_uptime_get_32();
	const bool command_fresh = _setpoint_sub.copy(&request)
				   && robot::fresh(request.timestamp, _config.command_timeout_ms * 1000ULL);
	chassis_status_s status{};
	status.timestamp = robot::now_us();
	status.command_fresh = command_fresh;

	if (!command_fresh) {
		_status_pub.publish(status);

		if (_offline_command_sent && now_ms - _last_offline_command_ms < 20) {
			return;
		}

		_offline_command_sent = true;
		_last_offline_command_ms = now_ms;

		for (int index = 0; index < 4; ++index) {
			_controller.reset();
			(void)_config.motors[index]->set_current(0.0f);
		}

		_config.motors[0]->send();
		return;
	}

	bool any_feedback_fresh = false;
	float targets[4] {};
	_controller.inverse(request.vx_m_s, request.vy_m_s, request.yaw_rad_s, targets);

	for (int index = 0; index < 4; ++index) {
		const bool feedback_fresh = _config.motors[index]->refresh(_config.feedback_timeout_ms);
		const auto &feedback = _config.motors[index]->measurement();
		any_feedback_fresh |= feedback_fresh;
		status.feedback_fresh[index] = feedback_fresh;
		status.wheel_speed_rad_s[index] = feedback.speed_rad_s;
		const float current_a = _controller.update(index, targets[index], feedback.speed_rad_s, feedback_fresh, dt_s);
		status.command_current_a[index] = current_a;
		(void)_config.motors[index]->set_current(current_a);
	}

	_status_pub.publish(status);

	if (!any_feedback_fresh) {
		if (_offline_command_sent && now_ms - _last_offline_command_ms < 20) {
			return;
		}

		_offline_command_sent = true;
		_last_offline_command_ms = now_ms;

	} else {
		_offline_command_sent = false;
	}

	_config.motors[0]->send();
}

void ChassisControl::task_trampoline(void *context, void *, void *)
{
	static_cast<ChassisControl *>(context)->Run();
}

void ChassisControl::Run()
{
	while (1) {
		k_sem_take(&chassis_sem, K_FOREVER);
		control_step(static_cast<float>(_config.period_ms) * 0.001f);
	}
}

int ChassisControl::init(const ChassisControlConfig &config)
{
	if (_initialized) {
		return 0;
	}

	_config = config;

	if (!std::isfinite(_config.wheel_radius_m) || _config.wheel_radius_m <= 0.0f || _config.period_ms == 0) { return -ENODEV; }

	for (int index = 0; index < 4; ++index) {
		if (_config.motors[index] == nullptr || _config.motors[index]->init() != 0) { return -ENODEV; }

		_config.motors[index]->set_current(0.0f);
	}

	_controller.configure({_config.rotation_coefficient, _config.wheel_radius_m, _config.max_wheel_rad_s},
			      _config.speed_pid);
	_initialized = true;
	k_timer_init(&_chassis_timer, timer_callback, nullptr);
	k_timer_start(&_chassis_timer, K_MSEC(_config.period_ms), K_MSEC(_config.period_ms));
	k_thread_create(&_chassis_thread, chassis_task, K_THREAD_STACK_SIZEOF(chassis_task), task_trampoline, this, nullptr,
			nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	return 0;
}

void ChassisControl::SetVelocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
	if (!_initialized) {
		return;
	}

	if (std::isfinite(vx_m_s) && std::isfinite(vy_m_s) && std::isfinite(yaw_rad_s)) { _setpoint_pub.publish({vx_m_s, vy_m_s, yaw_rad_s, robot::now_us()}); }
}

int app_chassis_init(const ChassisControlConfig &config)
{
	return chassis.init(config);
}

void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
	chassis.SetVelocity(vx_m_s, vy_m_s, yaw_rad_s);
}
