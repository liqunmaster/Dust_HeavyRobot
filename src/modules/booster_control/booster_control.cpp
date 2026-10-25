#include "booster_control.hpp"

#include <string.h>

namespace
{
BoosterControl booster;

K_THREAD_STACK_DEFINE(booster_stack, 2048);

K_SEM_DEFINE(booster_sem, 0, 1);
}

int BoosterControl::SendRecovery(bool clear_error)
{
	return _config.motor->recover(clear_error);
}

int BoosterControl::SendTorque(float torque_nm)
{
	return _config.motor->send_torque(torque_nm);
}

void BoosterControl::control_step()
{
	const auto previous_reset = _motion.reset_generation();
	const uint32_t now_ms = k_uptime_get_32();
	const bool has_feedback = _config.motor->refresh(UINT32_MAX);
	const auto &data = _config.motor->measurement();
	const uint32_t age_ms = has_feedback ? static_cast<uint32_t>((robot::now_us() - data.timestamp) / 1000) : UINT32_MAX;
	booster_setpoint_s request{};
	const bool armed = _setpoint_sub.copy(&request) && request.valid
			   && robot::fresh(request.timestamp, _config.command_timeout_ms * 1000ULL);
	const bool fresh = age_ms < (armed ? _config.armed_feedback_timeout_ms : _config.idle_feedback_timeout_ms);
	const bool motor_ready = fresh && data.motor_status == 0x01;

	const uint8_t s1 = armed ? request.switch_position : 0;

	if (!armed) {
		_trigger_armed = false;

	} else if (s1 == 1 || s1 == 3) {
		_trigger_armed = true;
	}

	const bool rising = s1 == 2 && _trigger_armed;

	if (rising) {
		_trigger_armed = false;
	}

	int result = 0;
	float command_torque = 0.0f;

	if (!armed && _motion.state() == BoosterState::jam_fault) {
		_motion.ResetMotion(data.angle_rad, now_ms, BoosterState::disabled);
	}

	if (!motor_ready) {
		if (_motion.state() != BoosterState::motor_fault && _motion.state() != BoosterState::jam_fault) {
			_motion.ResetMotion(data.angle_rad, now_ms, BoosterState::motor_fault);
		}

		if (!fresh) {
			_offline_ticks += _timer_period_ms;

			if (_offline_ticks >= 500) {
				_offline_ticks = 0;
				result = SendRecovery(false);
			}

		} else {
			_offline_ticks = 0;
			_fault_ticks += _timer_period_ms;

			if (_fault_ticks >= 100) {
				_fault_ticks = 0;
				result = SendRecovery(data.motor_status >= 0x08);

			} else {
				result = SendTorque(0.0f);
			}
		}

	} else {
		_offline_ticks = 0;
		_fault_ticks = 0;
		_motion.UpdateMotion(data.angle_rad, data.speed_rad_s, data.torque_nm, now_ms, armed, rising);

		if (previous_reset != _motion.reset_generation()) { _angle_pid.Reset(); }

		if (armed && _motion.state() != BoosterState::jam_fault && _motion.state() != BoosterState::disabled) {
			command_torque = _angle_pid.UpdateAngle(_motion.target_angle(), data.angle_rad, 0.001f);
		}

		if (armed || _motion.state() != BoosterState::disabled ||
		    now_ms - _last_keepalive_ms >= 5) {
			result = SendTorque(command_torque);
			_last_keepalive_ms = now_ms;
		}
	}

	if (previous_reset != _motion.reset_generation() && !motor_ready) { _angle_pid.Reset(); }

	if (result != 0) {
		_last_error = result;
	}

	const BoosterStatus snapshot{
		static_cast<uint8_t>(_motion.state()), fresh, data.motor_status, age_ms,
		data.angle_rad, _motion.target_angle(), data.speed_rad_s,
		data.torque_nm, command_torque, _motion.retries(), _last_error, robot::now_us()
	};
	_status_pub.publish(snapshot);
	const k_spinlock_key_t key = k_spin_lock(&_status_lock);
	_status = snapshot;
	k_spin_unlock(&_status_lock, key);

	const uint32_t next_period_ms = !fresh ? 20 : !armed && _motion.state() == BoosterState::disabled ? 5 : 1;

	if (next_period_ms != _timer_period_ms) {
		_timer_period_ms = next_period_ms;
		k_timer_start(&_booster_timer, K_MSEC(next_period_ms), K_MSEC(next_period_ms));
	}
}

void BoosterControl::timer_callback(struct k_timer *arg1)
{
	k_sem_give(&booster_sem);
}

void BoosterControl::task_trampoline(void *context, void *, void *)
{
	static_cast<BoosterControl *>(context)->Run();
}

void BoosterControl::Run()
{
	while (1) {
		k_sem_take(&booster_sem, K_FOREVER);
		control_step();
	}
}

int BoosterControl::init(const BoosterControlConfig &config)
{
	if (_initialized) {
		return 0;
	}

	_config = config;

	if (_config.motor == nullptr || _config.motor->init() != 0) { return -ENODEV; }

	_angle_pid.Configure(_config.angle_pid);
	_initialized = true;
	k_timer_init(&_booster_timer, timer_callback, nullptr);
	k_timer_start(&_booster_timer, K_MSEC(1), K_MSEC(1));
	k_thread_create(&_booster_thread, booster_stack, K_THREAD_STACK_SIZEOF(booster_stack), task_trampoline, this, nullptr,
			nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	return 0;
}

void BoosterControl::GetStatus(BoosterStatus &snapshot)
{
	snapshot = {};

	if (!_initialized) {
		return;
	}

	const k_spinlock_key_t key = k_spin_lock(&_status_lock);
	snapshot = _status;
	k_spin_unlock(&_status_lock, key);
}

int app_booster_init(const BoosterControlConfig &config)
{
	return booster.init(config);
}

void app_booster_get_status(BoosterStatus &snapshot)
{
	booster.GetStatus(snapshot);
}
