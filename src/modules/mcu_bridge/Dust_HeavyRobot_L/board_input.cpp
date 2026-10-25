#include "board_input.hpp"

#include <zephyr/kernel.h>
#include "manual_control.hpp"

void BoardInput::PublishChassis(const McuChassisCommand &value, uint32_t timestamp_ms)
{
	_chassis = value;
	_chassis_timestamp_ms = timestamp_ms;
	const bool command_fresh = k_uptime_get_32() - _command_timestamp_ms < 30U;
	input_manager_publish_mcu(_chassis.x, _chassis.y, _chassis.rotation, _chassis.flags,
				  command_fresh ? _command.mouse_lr : 0,
				  command_fresh ? _command.keyboard : 0, timestamp_ms);
}

void BoardInput::PublishCommand(const McuCommandCommand &value, uint32_t timestamp_ms)
{
	_command = value;
	_command_timestamp_ms = timestamp_ms;
	const bool chassis_fresh = k_uptime_get_32() - _chassis_timestamp_ms < 30U;
	input_manager_publish_mcu(chassis_fresh ? _chassis.x : 1024,
				  chassis_fresh ? _chassis.y : 1024,
				  chassis_fresh ? _chassis.rotation : 1024,
				  chassis_fresh ? _chassis.flags : 0,
				  _command.mouse_lr, _command.keyboard,
				  _chassis_timestamp_ms);
}
