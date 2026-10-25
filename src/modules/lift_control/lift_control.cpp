#include "lift_control.hpp"

#include <string.h>

namespace
{
LiftControl liftup;

K_THREAD_STACK_DEFINE(lift_task, 2048);

K_SEM_DEFINE(lift_sem, 0, 1);
}

void LiftControl::timer_callback(struct k_timer *arg1)
{
	k_sem_give(&lift_sem);
}

void LiftControl::control_step(float dt_s)
{
	for (uint8_t index = 0; index < 4; ++index) {
		const bool has_feedback = _config.motors[index]->refresh(_config.feedback_timeout_ms);
		const bool feedback_fresh = has_feedback;
		const auto &feedback = _config.motors[index]->measurement();
		lift_setpoint_s request{};
		const bool command_fresh = _setpoint_sub[index].copy(&request)
					   && robot::fresh(request.timestamp, _config.command_timeout_ms * 1000ULL);
		const float target_rad = command_fresh ? request.relative_angle_rad : 0.0f;
		float torque_nm = 0.0f;
		float measured_rad = 0.0f;

		if (feedback_fresh) {
			if (!_has_origin[index]) {
				_origin_rad[index] = feedback.angle_rad;
				_has_origin[index] = true;
			}

			measured_rad = (feedback.angle_rad - _origin_rad[index]);
			const float measured_speed = feedback.speed_rad_s;
			torque_nm = _loops[index].Update(target_rad, measured_rad, measured_speed, dt_s);

		} else {
			_loops[index].Reset();
		}

		const int result = feedback_fresh ? _config.motors[index]->send_torque(torque_nm) : 0;

		const LiftMotorStatus snapshot{
			feedback_fresh,
			has_feedback ? static_cast<uint32_t>((robot::now_us() - feedback.timestamp) / 1000) : UINT32_MAX,
			has_feedback ? feedback.valid_count : 0,
			target_rad, measured_rad, torque_nm, result,
			0, 0, 0, 0, robot::now_us()
		};
		_status_pub[index].publish(snapshot);
		const k_spinlock_key_t key = k_spin_lock(&_status_lock);
		_motor_status[index] = snapshot;
		k_spin_unlock(&_status_lock, key);
	}
}

void LiftControl::task_trampoline(void *context, void *, void *)
{
	static_cast<LiftControl *>(context)->Run();
}

void LiftControl::Run()
{
	while (1) {
		k_sem_take(&lift_sem, K_FOREVER);
		control_step(static_cast<float>(_config.period_ms) * 0.001f);
	}
}

int LiftControl::init(const LiftControlConfig &config)
{
	if (_initialized) {
		return 0;
	}

	_config = config;

	if (_config.period_ms == 0) { return -ENODEV; }

	for (uint8_t index = 0; index < 4; ++index) {
		if (_config.motors[index] == nullptr || _config.motors[index]->init() != 0) { return -ENODEV; }

		_loops[index].Configure(_config.position_pid, _config.speed_pid);
	}

	for (auto *motor : _config.motors) { if (motor->enable() != 0) { return -ENODEV; } }

	_initialized = true;
	k_timer_init(&_lift_timer, timer_callback, nullptr);
	k_timer_start(&_lift_timer, K_MSEC(_config.period_ms), K_MSEC(_config.period_ms));
	k_thread_create(&_lift_thread, lift_task, K_THREAD_STACK_SIZEOF(lift_task), task_trampoline, this, nullptr, nullptr,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	return 0;
}

void LiftControl::SetTarget(uint8_t can_index, float relative_angle_rad)
{
	if (!_initialized) {
		return;
	}

	if (can_index >= 4 || !std::isfinite(relative_angle_rad) ||
	    std::fabs(relative_angle_rad) > _config.max_angle_rad) {
		return;
	}

	_setpoint_pub[can_index].publish({relative_angle_rad, robot::now_us()});
}

void LiftControl::GetStatus(uint8_t can_index, LiftMotorStatus &status)
{
	status = {};

	if (can_index >= 4 || !_initialized) {
		return;
	}

	const k_spinlock_key_t key = k_spin_lock(&_status_lock);
	status = _motor_status[can_index];
	k_spin_unlock(&_status_lock, key);
	const auto counters = _config.motors[can_index]->counters();
	status.can_tx_completed = counters.tx_completed;
	status.can_tx_errors = counters.tx_errors;
	status.can_rx_received = counters.rx_received;
	status.can_rx_dropped = counters.rx_dropped;
}

int app_liftup_init(const LiftControlConfig &config)
{
	return liftup.init(config);
}

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad)
{
	liftup.SetTarget(can_index, relative_angle_rad);
}

void app_liftup_get_status(uint8_t can_index, LiftMotorStatus &status)
{
	liftup.GetStatus(can_index, status);
}
