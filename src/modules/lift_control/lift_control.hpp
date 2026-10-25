#pragma once
#include <uORB/topics/lift_setpoint.h>
#include <uORB/topics/lift_status.h>
#include "pid.hpp"
#include "MotorDevice.hpp"
#include "time.hpp"
#include "Publication.hpp"
#include "PublicationInstance.hpp"
#include "Subscription.hpp"

#include <cmath>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "control.hpp"

using LiftMotorStatus = lift_status_s;

struct LiftControlConfig {
	MotorDevice *motors[4] {};
	alg::PIDConfig position_pid{};
	alg::PIDConfig speed_pid{};
	uint32_t period_ms{2};
	uint32_t command_timeout_ms{100};
	uint32_t feedback_timeout_ms{100};
	float max_angle_rad{12.5f};
};

class LiftControl
{
public:
	int init(const LiftControlConfig &config);
	bool initialized() const { return _initialized; }

	void SetTarget(uint8_t can_index, float relative_angle_rad);

	void GetStatus(uint8_t can_index, LiftMotorStatus &status);

private:
	LiftControlConfig _config{};
	uORB::Subscription _setpoint_sub[4] = {{ORB_ID(lift_setpoint), 0}, {ORB_ID(lift_setpoint), 1}, {ORB_ID(lift_setpoint), 2}, {ORB_ID(lift_setpoint), 3}};
	uORB::PublicationInstance<lift_setpoint_s> _setpoint_pub[4] = {{ORB_ID(lift_setpoint), 0}, {ORB_ID(lift_setpoint), 1}, {ORB_ID(lift_setpoint), 2}, {ORB_ID(lift_setpoint), 3}};
	uORB::PublicationInstance<lift_status_s> _status_pub[4] = {{ORB_ID(lift_status), 0}, {ORB_ID(lift_status), 1}, {ORB_ID(lift_status), 2}, {ORB_ID(lift_status), 3}};
	static void timer_callback(struct k_timer *arg1);

	void control_step(float dt_s);

	static void task_trampoline(void *arg1, void *arg2, void *arg3);

	void Run();

	PositionSpeedLoop _loops[4] {};
	float _origin_rad[4] {};
	bool _has_origin[4] {};
	bool _initialized = false;
	LiftMotorStatus _motor_status[4] {};
	struct k_spinlock _status_lock {};
	struct k_thread _lift_thread {};
	struct k_timer _lift_timer {};
};

int app_liftup_init(const LiftControlConfig &config);

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad);

void app_liftup_get_status(uint8_t can_index, LiftMotorStatus &status);
