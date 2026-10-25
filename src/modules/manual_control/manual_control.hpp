#pragma once

#include "input_parser.hpp"

struct ManualControlConfig {
	float center{1024.0f};
	float span{660.0f};
	float deadzone{0.05f};
	float linear_speed_m_s{0.5f};
	float yaw_speed_rad_s{0.5f};
	float lift_range_rad{12.5f};
	float yaw_range_rad{3.14159265358979323846f};
	uint8_t forward_channel{2};
	uint8_t lateral_channel{3};
	uint8_t yaw_lift_channel{0};
};

class ManualControl
{
public:
	int configure(const ManualControlConfig &config);
	void PublishSample(const InputSample &sample);
	void PublishMcu(uint16_t chassis_x, uint16_t chassis_y, uint16_t rotation, uint8_t flags, uint8_t mouse_lr,
			uint16_t keyboard, uint32_t timestamp_ms);

private:
	ManualControlConfig _config{};
	float RawAxis(uint16_t raw);
	float Axis(uint16_t raw);
};

void input_manager_publish_sample(const InputSample &sample);
void input_manager_publish_mcu(uint16_t chassis_x, uint16_t chassis_y, uint16_t rotation, uint8_t flags,
			       uint8_t mouse_lr, uint16_t keyboard, uint32_t timestamp_ms);

// 在启动接收和板间桥接任务前完成配置；不支持运行时重新配置。
int manual_control_init(const ManualControlConfig &config);
