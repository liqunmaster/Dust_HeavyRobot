#pragma once

#include <msg/chassis_setpoint.hpp>
#include <msg/chassis_status.hpp>
#include "zbus_channel.hpp"

class ChassisChannel
{
    public:
    void PublishSetpoint(const ChassisSetpoint &data);

    int LatestSetpoint(ChassisSetpoint &data);

    void PublishStatus(const ChassisStatus &data);

    int LatestStatus(ChassisStatus &data);

    private:
    ZbusChannel<ChassisSetpoint> setpoint_;
    ZbusChannel<ChassisStatus> status_;
};

void chassis_channel_publish_setpoint(const ChassisSetpoint &data);
int chassis_channel_latest_setpoint(ChassisSetpoint &data);
void chassis_channel_publish_status(const ChassisStatus &data);
int chassis_channel_latest_status(ChassisStatus &data);
