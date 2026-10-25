#pragma once
#include "booster_controller.hpp"
#include <uORB/topics/booster_setpoint.h>
#include <uORB/topics/booster_status.h>
#include "MotorDevice.hpp"
#include "time.hpp"
#include "Publication.hpp"
#include "PublicationInstance.hpp"
#include "Subscription.hpp"

#include <cmath>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "pid.hpp"
#include "type_math.hpp"

struct BoosterControlConfig {
	MotorDevice *motor{nullptr};
	alg::PIDConfig angle_pid{};
	uint32_t command_timeout_ms{12};
	uint32_t armed_feedback_timeout_ms{12};
	uint32_t idle_feedback_timeout_ms{30};
};

using BoosterStatus = booster_status_s;

class BoosterControl
{
public:
	int init(const BoosterControlConfig &config);
	bool initialized() const { return _initialized; }

	void GetStatus(BoosterStatus &snapshot);

private:
	BoosterControlConfig _config{};
	BoosterMotion _motion{};
	uORB::Subscription _setpoint_sub{ORB_ID(booster_setpoint)};
	uORB::Publication<booster_status_s> _status_pub{ORB_ID(booster_status)};

	int SendRecovery(bool clear_error);

	int SendTorque(float torque_nm);


	void control_step();

	static void timer_callback(struct k_timer *arg1);

	static void task_trampoline(void *arg1, void *arg2, void *arg3);

	void Run();

	alg::Pid _angle_pid{};
	BoosterStatus _status{};
	struct k_spinlock _status_lock {};
	bool _trigger_armed = false;
	bool _initialized = false;
	uint32_t _last_keepalive_ms = 0;
	uint16_t _offline_ticks = 499;
	uint8_t _fault_ticks = 0;
	int _last_error = 0;
	struct k_thread _booster_thread {};
	struct k_timer _booster_timer {};
	uint32_t _timer_period_ms = 1;
};

int app_booster_init(const BoosterControlConfig &config);

void app_booster_get_status(BoosterStatus &status);
