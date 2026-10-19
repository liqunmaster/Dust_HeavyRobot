#include "chassis_channel.hpp"

#include <cmath>
#include "zbus_channel.hpp"

namespace
{
    ChassisChannel chassis_channel;
}

void ChassisChannel::PublishSetpoint(const ChassisSetpoint &data)
{
    if (std::isfinite(data.vx_m_s) && std::isfinite(data.vy_m_s) && std::isfinite(data.yaw_rad_s)) {
        setpoint_.Publish(data);
    }
}

int ChassisChannel::LatestSetpoint(ChassisSetpoint &data)
{
    return setpoint_.Read(data);
}

void ChassisChannel::PublishStatus(const ChassisStatus &data)
{
    status_.Publish(data);
}

int ChassisChannel::LatestStatus(ChassisStatus &data)
{
    return status_.Read(data);
}

void chassis_channel_publish_setpoint(const ChassisSetpoint &data)
{
    chassis_channel.PublishSetpoint(data);
}

int chassis_channel_latest_setpoint(ChassisSetpoint &data)
{
    return chassis_channel.LatestSetpoint(data);
}

void chassis_channel_publish_status(const ChassisStatus &data)
{
    chassis_channel.PublishStatus(data);
}

int chassis_channel_latest_status(ChassisStatus &data)
{
    return chassis_channel.LatestStatus(data);
}
