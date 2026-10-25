#include "booster_controller.hpp"
float BoosterMotion::WrapAngle(float angle)
{
	return std::remainder(angle, 2.0f * math::pi);
}

void BoosterMotion::SetState(BoosterState next, uint32_t now_ms)
{
	_state = next;
	_state_since_ms = now_ms;

	if (next == BoosterState::feeding) {
		_last_motion_update_ms = now_ms;
		_feeding_travel_rad = 0.0f;
	}
}

void BoosterMotion::ResetMotion(float angle, uint32_t now_ms, BoosterState next)
{
	++_reset_generation;
	_target_angle_rad = angle;
	_jam_count = 0;
	_recovery_count = 0;
	_jam_retries = 0;
	SetState(next, now_ms);
}

void BoosterMotion::UpdateMotion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising)
{
	if (!armed) {
		if (_state != BoosterState::disabled) {
			ResetMotion(angle, now_ms, BoosterState::disabled);
		}

		return;
	}

	if (_state == BoosterState::disabled || _state == BoosterState::motor_fault) {
		ResetMotion(angle, now_ms, BoosterState::ready);
	}

	switch (_state) {
	case BoosterState::ready:
		if (rising) {
			_target_angle_rad = WrapAngle(angle - math::pi / 3.0f);
			_jam_count = 0;
			_jam_retries = 0;
			SetState(BoosterState::feeding, now_ms);
		}

		break;

	case BoosterState::feeding:
		_feeding_travel_rad += std::fabs(speed) * static_cast<float>(now_ms - _last_motion_update_ms) * 0.001f;
		_last_motion_update_ms = now_ms;

		if (now_ms - _state_since_ms >= 1000 ||
		    _feeding_travel_rad >= 2.0f * (math::pi / 3.0f)) {
			++_reset_generation;
			SetState(BoosterState::jam_fault, now_ms);
			break;
		}

		if (std::fabs(torque) > 4.0f && std::fabs(speed) < 0.5f) {
			if (++_jam_count >= 5) {
				++_jam_retries;
				_recovery_count = 0;
				_target_angle_rad = WrapAngle(angle + math::pi / 3.0f);
				SetState(BoosterState::reversing, now_ms);
			}

		} else {
			_jam_count = 0;

			if (std::fabs(WrapAngle(_target_angle_rad - angle)) < 0.05f &&
			    std::fabs(speed) < 0.5f) {
				SetState(BoosterState::ready, now_ms);
			}
		}

		break;

	case BoosterState::reversing:
		if (std::fabs(WrapAngle(_target_angle_rad - angle)) < 0.05f && std::fabs(speed) < 0.5f) {
			if (++_recovery_count >= 5) {
				SetState(BoosterState::settling, now_ms);
			}

		} else {
			_recovery_count = 0;
		}

		if (now_ms - _state_since_ms >= 1000) {
			SetState(BoosterState::settling, now_ms);
		}

		break;

	case BoosterState::settling:
		if (now_ms - _state_since_ms >= 100) {
			if (_jam_retries >= 3) {
				++_reset_generation;
				SetState(BoosterState::jam_fault, now_ms);

			} else {
				_target_angle_rad = WrapAngle(angle - math::pi / 3.0f);
				_jam_count = 0;
				SetState(BoosterState::feeding, now_ms);
			}
		}

		break;

	case BoosterState::jam_fault:
	case BoosterState::disabled:
	case BoosterState::motor_fault:
		break;
	}
}
