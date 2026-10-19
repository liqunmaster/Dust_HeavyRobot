#include "remote_channel.hpp"

#include "chassis_channel.hpp"
#include "zbus_channel.hpp"
#include <cmath>

namespace
{
    RemoteChannel remote_channel;
}

void RemoteChannel::PublishInput(const RemoteTopicData &data)
{
    remote_input_.Publish(data);
}

int RemoteChannel::LatestInput(RemoteTopicData &data)
{
    return remote_input_.Read(data);
}

void RemoteChannel::PublishLift(uint8_t index, const LiftSetpoint &data)
{
    if (index < 4 && std::isfinite(data.relative_angle_rad)) {
        lift_[index].Publish(data);
    }
}

int RemoteChannel::LatestLift(uint8_t index, LiftSetpoint &data)
{
    return index < 4 ? lift_[index].Read(data) : -EINVAL;
}

void remote_channel_publish_input(const RemoteTopicData &data)
{
    remote_channel.PublishInput(data);
}

int remote_channel_latest_input(RemoteTopicData &data)
{
    return remote_channel.LatestInput(data);
}

void remote_channel_publish_lift(uint8_t index, const LiftSetpoint &data)
{
    remote_channel.PublishLift(index, data);
}

void remote_channel_publish_lift(uint8_t index, float angle)
{
    remote_channel_publish_lift(index, LiftSetpoint{angle, k_uptime_get_32()});
}

int remote_channel_latest_lift(uint8_t index, LiftSetpoint &data)
{
    return remote_channel.LatestLift(index, data);
}

void remote_channel_publish_chassis(float vx, float vy, float yaw)
{
    chassis_channel_publish_setpoint({vx, vy, yaw, k_uptime_get_32()});
}
