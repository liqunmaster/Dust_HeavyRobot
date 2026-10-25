#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "motor_service.hpp"
#include "motor_port.hpp"

class MotorHealth
{
public:
	void Init();

	void Poll();

	bool IsOffline(size_t index);

private:
	enum class MotorState : uint8_t {
		unknown = 0,
		enabled = 1,
		disabled = 2,
		error = 3,
		offline = 4,
	};
	MotorState ClassifyState(const CanfdMotorHealthData &motor, uint32_t now_ms, uint32_t last_feedback);

	void CheckMotors(uint32_t now_ms, uint32_t elapsed_ms);
	CanfdMotorHealthData _motors[CANFD_PORT_MAX_MOTORS] {};
	bool _present[CANFD_PORT_MAX_MOTORS] {};
	uint32_t _last_count[CANFD_PORT_MAX_MOTORS] {};
	uint32_t _last_feedback_ms[CANFD_PORT_MAX_MOTORS] {};
	uint16_t _recovery_ticks[CANFD_PORT_MAX_MOTORS] {};
	bool _registered[CANFD_PORT_MAX_MOTORS] {};
	atomic_t _offline[CANFD_PORT_MAX_MOTORS] {};
	MotorState _states[CANFD_PORT_MAX_MOTORS] {};
	bool _initialized = false;
	bool _checked = false;
	uint32_t _last_check_ms = 0;
};

void motor_health_init();

void motor_health_poll();

bool motor_health_is_offline(size_t index);
