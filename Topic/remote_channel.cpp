#include "remote_channel.hpp"

namespace
{
    ZbusChannel<RemoteTopicData> remote_input;
    ZbusChannel<ChassisVelocityTopicData> chassis;
    ZbusChannel<LiftAngleTopicData> lift[4];

    float axis(uint16_t raw)
    {
        float value = (static_cast<float>(raw) - 1024.0F) / 660.0F;
        if (value > 1.0F) {
            value = 1.0F;
        } else if (value < -1.0F) {
            value = -1.0F;
        }
        return std::fabs(value) < REMOTE_AXIS_DEADZONE ? 0.0F : value;
    }

    void publish_chassis(float vx, float vy, float yaw, uint32_t timestamp_ms)
    {
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)) {
            return;
        }
        chassis.publish({vx, vy, yaw, timestamp_ms});
    }
}

void remote_channel_publish_sample(const input_sample &sample)
{
    RemoteTopicData command{};
    command.timestamp_ms = sample.timestamp_ms;
    command.protocol = static_cast<uint8_t>(sample.protocol);
    switch (sample.protocol) {
        case remote_protocol::dt7:
            command.forward  = REMOTE_DT7_CHASSIS_X_SIGN * axis(sample.dt7_data.channel[2]);
            command.lateral  = REMOTE_DT7_CHASSIS_Y_SIGN * axis(sample.dt7_data.channel[3]);
            command.rotation = 0.0F;
            command.switch_left  = sample.dt7_data.switch_left;
            command.switch_right = sample.dt7_data.switch_right;
            command.valid = true;
            break;
        case remote_protocol::vt03:
            command.switch_left = sample.vt03_data.mode_switch;
            break;
        case remote_protocol::vt02:
            break;
        default:
            return;
    }

    remote_input.publish(command);
    if (sample.protocol != remote_protocol::dt7) {
        return;
    }
    publish_chassis(command.forward * REMOTE_MAX_FORWARD_M_S, command.lateral * REMOTE_MAX_LATERAL_M_S, 0.0F, command.timestamp_ms);
    const float lift_angle = REMOTE_DT7_LIFT_SIGN * axis(sample.dt7_data.channel[0]) * REMOTE_LIFT_ANGLE_RANGE_RAD;
    for (uint8_t index = 0U; index < 4U; ++index) {
        lift[index].publish({lift_angle, command.timestamp_ms});
    }
}

int remote_channel_latest_input(RemoteTopicData &data)
{
    return remote_input.read(data);
}

void remote_channel_publish_chassis(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    publish_chassis(vx_m_s, vy_m_s, yaw_rad_s, k_uptime_get_32());
}

int remote_channel_latest_chassis(ChassisVelocityTopicData &data)
{
    return chassis.read(data);
}

void remote_channel_publish_lift(uint8_t index, float relative_angle_rad)
{
    if (index >= 4U || !std::isfinite(relative_angle_rad)) {
        return;
    }
    lift[index].publish({relative_angle_rad, k_uptime_get_32()});
}

int remote_channel_latest_lift(uint8_t index, LiftAngleTopicData &data)
{
    if (index >= 4U) {
        return -EINVAL;
    }
    return lift[index].read(data);
}
