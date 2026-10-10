#pragma once

#include <cmath>
#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "input_parser.hpp"
#include "zbus_channel.hpp"

#include <msg/remote_topic.hpp>

struct input_sample;

// DT7 left X/Y drive chassis X/Y; right X drives the lift.
// Initial scales and signs should be tuned on the physical robot.
#define REMOTE_MAX_FORWARD_M_S 0.5f
#define REMOTE_MAX_LATERAL_M_S 0.5f
#define REMOTE_AXIS_DEADZONE 0.05f
#define REMOTE_LIFT_ANGLE_RANGE_RAD 12.5f
#define REMOTE_DT7_CHASSIS_X_SIGN 1.0f
#define REMOTE_DT7_CHASSIS_Y_SIGN 1.0f
#define REMOTE_DT7_LIFT_SIGN 1.0f

// 底盘速度指令主题数据
struct ChassisVelocityTopicData
{
    float vx_m_s;
    float vy_m_s;
    float yaw_rad_s;
    uint32_t timestamp_ms;
};

// 升降轴角度指令主题数据
struct LiftAngleTopicData
{
    float relative_angle_rad;
    uint32_t timestamp_ms;
};

void remote_channel_publish_sample(const input_sample &sample);

int remote_channel_latest_input(RemoteTopicData &data);

void remote_channel_publish_chassis(float vx_m_s, float vy_m_s, float yaw_rad_s);

int remote_channel_latest_chassis(ChassisVelocityTopicData &data);

void remote_channel_publish_lift(uint8_t index, float relative_angle_rad);

int remote_channel_latest_lift(uint8_t index, LiftAngleTopicData &data);

// Wait for a new remote sample. The topic itself remains latest-value based;
// this event path prevents app threads from polling it every millisecond.
