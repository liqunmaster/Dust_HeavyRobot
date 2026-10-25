#include "motor_health.hpp"

namespace
{
MotorHealth motor_health;
}

MotorHealth::MotorState MotorHealth::ClassifyState(const CanfdMotorHealthData &motor, uint32_t now_ms,
		uint32_t last_feedback)
{
	if (motor.kind == MotorChannelKind::cubemars) {
		if (!motor.enable_acknowledged || now_ms - last_feedback >= 100) {
			return MotorState::offline;
		}

		return MotorState::enabled;
	}

	if (now_ms - last_feedback >= 30) {
		return MotorState::offline;
	}

	if (motor.kind == MotorChannelKind::dm) {
		if (motor.status == 0x01) {
			return MotorState::enabled;
		}

		return motor.status >= 0x08 ? MotorState::error : MotorState::disabled;
	}

	return MotorState::enabled;
}

void MotorHealth::CheckMotors(uint32_t now_ms, uint32_t elapsed_ms)
{

	const size_t motor_count = canfd_port_motor_count();

	for (size_t index = 0; index < motor_count; ++index) {
		_present[index] = false;
		_motors[index] = {};

		if (canfd_port_motor_health(index, _motors[index]) != 0) {
			continue;
		}

		_present[index] = true;
		const uint32_t count = _motors[index].valid_feedback_count;

		if (!_registered[index]) {
			_registered[index] = true;
			_last_count[index] = count;

			if (_motors[index].kind == MotorChannelKind::cubemars) {
				atomic_set(&_offline[index], 1);
			}

			_last_feedback_ms[index] = _motors[index].kind == MotorChannelKind::cubemars ? (_motors[index].request_count != 0
						   && count == 0 ? _motors[index].pending_since_ms : now_ms) : now_ms - 30;

		} else if (count != _last_count[index]) {
			_last_count[index] = count;
			_last_feedback_ms[index] = now_ms;
		}

		const MotorState next = ClassifyState(_motors[index], now_ms, _last_feedback_ms[index]);

		if (next != _states[index]) {
			_states[index] = next;
			_recovery_ticks[index] = 0;
		}

		const bool is_offline = next == MotorState::offline;

		if (atomic_set(&_offline[index], is_offline ? 1 : 0) != (is_offline ? 1 : 0) &&
		    _motors[index].kind == MotorChannelKind::cubemars) {
			canfd_port_set_motor_offline(index, is_offline);
		}
	}

	for (size_t index = 0; index < motor_count; ++index) {
		if (!_present[index] || !_motors[index].automatic_recovery || _states[index] == MotorState::enabled
		    || _recovery_ticks[index] != 0) {
			continue;
		}

		const CanfdMotorHealthData &motor = _motors[index];

		if (motor.kind == MotorChannelKind::c610 || motor.kind == MotorChannelKind::c620) {
			continue;
		}

		if (motor.kind == MotorChannelKind::cubemars &&
		    now_ms - _last_feedback_ms[index] < 100) {
			continue;
		}

		const bool clear_error = _states[index] == MotorState::error;
		CanfdFrame frame{};

		if (canfd_port_build_recovery_frame(index, clear_error, frame) != 0) {
			continue;
		}

		(void)canfd_port_send_once(motor.bus, frame);
		_recovery_ticks[index] = motor.kind == MotorChannelKind::cubemars ? 50 : 1;
	}

	for (size_t index = 0; index < motor_count; ++index) {
		if (_present[index] && _states[index] != MotorState::enabled &&
		    _recovery_ticks[index] != 0) {
			_recovery_ticks[index] = _recovery_ticks[index] > elapsed_ms
						 ? static_cast<uint16_t>(_recovery_ticks[index] - elapsed_ms)
						 : 0;
		}
	}
}

void MotorHealth::Init()
{
	if (_initialized) {
		return;
	}

	const int result = canfd_port_init();

	if (result != 0) {
		return;
	}

	_initialized = true;
}

void MotorHealth::Poll()
{
	if (!_initialized) {
		return;
	}

	const uint32_t now_ms = k_uptime_get_32();

	if (_checked && now_ms - _last_check_ms < 10) {
		return;
	}

	const uint32_t elapsed_ms = _checked ? now_ms - _last_check_ms : 10;
	_checked = true;
	_last_check_ms = now_ms;
	CheckMotors(now_ms, elapsed_ms);
}

bool MotorHealth::IsOffline(size_t index)
{
	return index < CANFD_PORT_MAX_MOTORS && atomic_get(&_offline[index]) != 0;
}

void motor_health_init()
{
	motor_health.Init();
}

void motor_health_poll()
{
	motor_health.Poll();
}

bool motor_health_is_offline(size_t index)
{
	return motor_health.IsOffline(index);
}
