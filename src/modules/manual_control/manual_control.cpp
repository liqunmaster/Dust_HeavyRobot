#include "manual_control.hpp"
#include "Publication.hpp"
#include "PublicationInstance.hpp"
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/chassis_setpoint.h>
#include <uORB/topics/lift_setpoint.h>
#include <uORB/topics/yaw_setpoint.h>
#include <uORB/topics/booster_setpoint.h>
#include "time.hpp"
#include "type_math.hpp"
#include <cmath>
#include <errno.h>
namespace
{
ManualControl input_manager;
uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
uORB::Publication<chassis_setpoint_s> chassis_pub{ORB_ID(chassis_setpoint)};
uORB::Publication<yaw_setpoint_s> yaw_pub{ORB_ID(yaw_setpoint)};
uORB::Publication<booster_setpoint_s> booster_pub{ORB_ID(booster_setpoint)};
uORB::PublicationInstance<lift_setpoint_s> lift_pub[4] = {
	{ORB_ID(lift_setpoint), 0}, {ORB_ID(lift_setpoint), 1},
	{ORB_ID(lift_setpoint), 2}, {ORB_ID(lift_setpoint), 3}
};
void publish(const manual_control_setpoint_s &command)
{
	manual_pub.publish(command);
	booster_pub.publish({command.switch_right, command.valid, command.timestamp});
}
}
int ManualControl::configure(const ManualControlConfig &config)
{
	const float values[] = {config.center, config.span, config.deadzone, config.linear_speed_m_s, config.yaw_speed_rad_s, config.lift_range_rad, config.yaw_range_rad};

	for (float value : values) {if (!std::isfinite(value)) {return -EINVAL;}}

	if (config.span <= 0 || config.deadzone < 0 || config.deadzone >= 1 || config.forward_channel >= 4
	    || config.lateral_channel >= 4 || config.yaw_lift_channel >= 4) {return -EINVAL;}

	_config = config; return 0;
}
float ManualControl::RawAxis(uint16_t raw)
{
	const float value = (static_cast<float>(raw) - _config.center) / _config.span;
	return value > 1.0f ? 1.0f : value < -1.0f ? -1.0f : value;
}
float ManualControl::Axis(uint16_t raw)
{
	const float value = RawAxis(raw);
	return std::fabs(value) < _config.deadzone ? 0.0f : value;
}
void ManualControl::PublishSample(const InputSample &sample)
{
	manual_control_setpoint_s command{};
	command.timestamp = robot::source_time_us(sample.timestamp_ms);
	command.protocol = static_cast<uint8_t>(sample.protocol);

	switch (sample.protocol) {
	case RemoteProtocol::dt7:
		command.forward = Axis(sample.dt7_data.channel[_config.forward_channel]);
		command.lateral = Axis(sample.dt7_data.channel[_config.lateral_channel]);
		command.switch_left = sample.dt7_data.switch_left;
		command.switch_right = sample.dt7_data.switch_right;
		command.valid = true;
		break;

	case RemoteProtocol::vt03:
		command.switch_left = sample.vt03_data.mode_switch;
		break;

	case RemoteProtocol::vt02: break;

	default: return;
	}

	publish(command);

	if (sample.protocol != RemoteProtocol::dt7) {return;}

	chassis_pub.publish({command.forward * _config.linear_speed_m_s, command.lateral * _config.linear_speed_m_s, 0.0f, command.timestamp});
	yaw_pub.publish({RawAxis(sample.dt7_data.channel[_config.yaw_lift_channel])* _config.yaw_range_rad, command.timestamp, true});

	for (auto &publication : lift_pub) {publication.publish({Axis(sample.dt7_data.channel[_config.yaw_lift_channel]) * _config.lift_range_rad, command.timestamp});}
}
void ManualControl::PublishMcu(uint16_t x, uint16_t y, uint16_t rotation, uint8_t flags, uint8_t mouse_lr,
			       uint16_t keyboard, uint32_t timestamp_ms)
{
	manual_control_setpoint_s command{};
	command.forward = Axis(x); command.lateral = Axis(y); command.rotation = Axis(rotation);
	command.protocol = static_cast<uint8_t>(RemoteProtocol::dt7);
	command.switch_left = (flags >> 1) & 0x03;
	command.switch_right = (flags & (1u << 5)) != 0 ? 2 : 1;
	command.valid = command.switch_left >= 1 && command.switch_left <= 3;
	command.timestamp = robot::source_time_us(timestamp_ms);
	publish(command);
	chassis_pub.publish({command.forward * _config.linear_speed_m_s, command.lateral * _config.linear_speed_m_s, command.rotation * _config.yaw_speed_rad_s, command.timestamp});
	(void)mouse_lr; (void)keyboard;
}
void input_manager_publish_sample(const InputSample &sample) {input_manager.PublishSample(sample);}
void input_manager_publish_mcu(uint16_t x, uint16_t y, uint16_t rotation, uint8_t flags, uint8_t mouse_lr,
			       uint16_t keyboard, uint32_t timestamp_ms)
{
	input_manager.PublishMcu(x, y, rotation, flags, mouse_lr, keyboard, timestamp_ms);
}

int manual_control_init(const ManualControlConfig &config) {return input_manager.configure(config);}
