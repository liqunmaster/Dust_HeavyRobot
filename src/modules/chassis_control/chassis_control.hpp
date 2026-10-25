#pragma once
#include "chassis_controller.hpp"
#include <uORB/topics/chassis_setpoint.h>
#include <uORB/topics/chassis_status.h>
#include "pid.hpp"
#include "MotorDevice.hpp"
#include "time.hpp"
#include "Publication.hpp"
#include "PublicationInstance.hpp"
#include "Subscription.hpp"

#include <cmath>
#include <zephyr/kernel.h>
#include "control.hpp"

struct ChassisControlConfig {
	MotorDevice *motors[4] {};
	alg::PIDConfig speed_pid{};
	float rotation_coefficient{0.0f};
	float wheel_radius_m{0.075f};
	float max_wheel_rad_s{0.0f};
	uint32_t period_ms{1};
	uint32_t command_timeout_ms{100};
	uint32_t feedback_timeout_ms{100};
};

class ChassisControl
{
public:
	int init(const ChassisControlConfig &config);
	bool initialized() const { return _initialized; }

	void SetVelocity(float vx_m_s, float vy_m_s, float yaw_rad_s);

private:
	ChassisControlConfig _config{};
	uORB::Subscription _setpoint_sub{ORB_ID(chassis_setpoint)};
	uORB::Publication<chassis_status_s> _status_pub{ORB_ID(chassis_status)};
	uORB::Publication<chassis_setpoint_s> _setpoint_pub{ORB_ID(chassis_setpoint)};

	static void timer_callback(struct k_timer *arg1);

	void control_step(float dt_s);

	static void task_trampoline(void *arg1, void *arg2, void *arg3);

	void Run();

	ChassisController _controller{};
	struct k_thread _chassis_thread {};
	struct k_timer _chassis_timer {};
	bool _initialized = false;
	bool _offline_command_sent = false;
	uint32_t _last_offline_command_ms = 0;
};

int app_chassis_init(const ChassisControlConfig &config);

void app_chassis_set_velocity(float vx_m_s, float vy_m_s, float yaw_rad_s);
