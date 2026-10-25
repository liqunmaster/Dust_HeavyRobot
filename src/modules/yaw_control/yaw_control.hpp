#pragma once
#include "yaw_controller.hpp"
#include <uORB/topics/yaw_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include "MotorDevice.hpp"
#include "time.hpp"
#include "Publication.hpp"
#include "PublicationInstance.hpp"
#include "Subscription.hpp"

#if defined(ROBOT_BOARD_H)
#include "pid.hpp"

struct YawControlConfig {
	MotorDevice *motor{nullptr};
	alg::PIDConfig position_pid{};
	alg::PIDConfig speed_pid{};
	uint32_t period_ms{1};
	uint32_t command_timeout_ms{100};
	uint32_t feedback_timeout_ms{100};
	uint32_t attitude_timeout_ms{20};
};

class YawControl
{
public:
	int init(const YawControlConfig &config);
	bool initialized() const { return _initialized; }

private:
	YawControlConfig _config{};
	uORB::Subscription _setpoint_sub{ORB_ID(yaw_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	static void timer_callback(struct k_timer *timer);
	static void task_trampoline(void *context, void *, void *);
	void Run();

	void control_step();

	YawController _controller{};
	bool _control_active = false;
	uint32_t _last_control_ms = 0;
	uint32_t _last_send_ms = 0;

	struct k_thread _thread {};
	struct k_timer _timer {};
	bool _initialized = false;
};

int app_yaw_init(const YawControlConfig &config);

#endif
