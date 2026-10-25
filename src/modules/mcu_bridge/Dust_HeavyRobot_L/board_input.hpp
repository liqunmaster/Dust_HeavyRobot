#pragma once

#include "mcu.hpp"

class BoardInput
{
public:
	void PublishChassis(const McuChassisCommand &value, uint32_t timestamp_ms);
	void PublishCommand(const McuCommandCommand &value, uint32_t timestamp_ms);

private:
	McuChassisCommand _chassis{1024, 1024, 1024, 0};
	McuCommandCommand _command{0, 0, 0.0f};
	uint32_t _chassis_timestamp_ms = 0;
	uint32_t _command_timestamp_ms = 0;
};
