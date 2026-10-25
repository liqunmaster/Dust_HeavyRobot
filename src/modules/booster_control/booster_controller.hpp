#pragma once
#include <stdint.h>
#include <cmath>
#include "type_math.hpp"
enum class BoosterState : uint8_t {
	disabled,
	ready,
	feeding,
	reversing,
	settling,
	jam_fault,
	motor_fault,
};

class BoosterMotion
{
public:
	void ResetMotion(float angle, uint32_t now_ms, BoosterState next);
	void UpdateMotion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising);
	BoosterState state() const {return _state;}
	float target_angle() const {return _target_angle_rad;}
	uint8_t retries() const {return _jam_retries;}
	uint32_t reset_generation() const {return _reset_generation;}
private:
	static float WrapAngle(float angle);
	void SetState(BoosterState next, uint32_t now_ms);
	uint32_t _reset_generation{0};
	uint8_t _jam_count = 0;
	uint8_t _recovery_count = 0;
	uint8_t _jam_retries = 0;
	uint32_t _state_since_ms = 0;
	uint32_t _last_motion_update_ms = 0;
	float _feeding_travel_rad = 0.0f;
	float _target_angle_rad = 0.0f;
	BoosterState _state = BoosterState::disabled;
};
