#pragma once

#include <stdint.h>
#include <msg/remote_topic.hpp>
#include <msg/lift_setpoint.hpp>
#include "zbus_channel.hpp"

class RemoteChannel
{
    public:
    void PublishInput(const RemoteTopicData &data);

    int LatestInput(RemoteTopicData &data);

    void PublishLift(uint8_t index, const LiftSetpoint &data);

    int LatestLift(uint8_t index, LiftSetpoint &data);

    private:
    ZbusChannel<RemoteTopicData> remote_input_;
    ZbusChannel<LiftSetpoint> lift_[4];
};

void remote_channel_publish_input(const RemoteTopicData &data);
int remote_channel_latest_input(RemoteTopicData &data);
void remote_channel_publish_lift(uint8_t index, const LiftSetpoint &data);
void remote_channel_publish_lift(uint8_t index, float relative_angle_rad);
int remote_channel_latest_lift(uint8_t index, LiftSetpoint &data);

void remote_channel_publish_chassis(float vx_m_s, float vy_m_s, float yaw_rad_s);
