#include "remote_channel.hpp"

namespace
{
    zbus_channel<RemoteTopicData> remote_input;
    zbus_channel<chassis_velocity_topic_data> chassis;
    zbus_channel<lift_angle_topic_data> lift[4];

    /**
     * @brief 将遥杆原始值归一化到 [-1,1] 并做死区处理
     *
     * @param raw 遥杆原始值
     * @return 归一化后的值
     */
    float axis(uint16_t raw)
    {
        float value = (static_cast<float>(raw) - 1024.0f) / 660.0f;
        if (value > 1.0f) {
            value = 1.0f;
        } else if (value < -1.0f) {
            value = -1.0f;
        }
        return std::fabs(value) < REMOTE_AXIS_DEADZONE ? 0.0f : value;
    }

    /**
     * @brief 发布底盘速度指令
     *
     * @param vx 前进方向速度
     * @param vy 横移方向速度
     * @param yaw 偏航角速度
     * @param timestamp_ms 时间戳毫秒）
     */
    void publish_chassis(float vx, float vy, float yaw, uint32_t timestamp_ms)
    {
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)) {
            return;
        }
        chassis.publish({vx, vy, yaw, timestamp_ms});
    }
}

/**
 * @brief 发布一次遥杆采样：按协议解析并分发到各主题
 *
 * @param sample 解析后的遥杆输入采样
 */
void remote_channel_publish_sample(const input_sample &sample)
{
    RemoteTopicData command{};
    command.timestamp_ms = sample.timestamp_ms;
    command.protocol = static_cast<uint8_t>(sample.protocol);
    switch (sample.protocol) {
        case remote_protocol::dt7:
        command.forward  = REMOTE_DT7_CHASSIS_X_SIGN * axis(sample.dt7_data.channel[2]);
        command.lateral  = REMOTE_DT7_CHASSIS_Y_SIGN * axis(sample.dt7_data.channel[3]);
        command.rotation = 0.0f;
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
    publish_chassis(command.forward * REMOTE_MAX_FORWARD_M_S, command.lateral * REMOTE_MAX_LATERAL_M_S, 0.0f, command.timestamp_ms);
    const float lift_angle = REMOTE_DT7_LIFT_SIGN * axis(sample.dt7_data.channel[0]) * REMOTE_LIFT_ANGLE_RANGE_RAD;
    for (uint8_t index = 0; index < 4; ++index) {
        lift[index].publish({lift_angle, command.timestamp_ms});
    }
}

/**
 * @brief 读取最新一帧遥杆输入指令
 *
 * @param data 数据输出参数
 * @return 成功返回 0 暂无数据返回错误码
 */
int remote_channel_latest_input(RemoteTopicData &data)
{
    return remote_input.read(data);
}

/**
 * @brief 发布底盘速度指令
 *
 * @param vx_m_s 前进方向速度
 * @param vy_m_s 横移方向速度
 * @param yaw_rad_s 偏航角速度
 */
void remote_channel_publish_chassis(float vx_m_s, float vy_m_s, float yaw_rad_s)
{
    publish_chassis(vx_m_s, vy_m_s, yaw_rad_s, k_uptime_get_32());
}

/**
 * @brief 读取最新底盘速度指令
 *
 * @param data 数据输出参数
 * @return 成功返回 0 暂无数据返回错误码
 */
int remote_channel_latest_chassis(chassis_velocity_topic_data &data)
{
    return chassis.read(data);
}

/**
 * @brief 发布某一升降轴的角度指令
 *
 * @param index 升降轴索引
 * @param relative_angle_rad 相对角度
 */
void remote_channel_publish_lift(uint8_t index, float relative_angle_rad)
{
    if (index >= 4 || !std::isfinite(relative_angle_rad)) {
        return;
    }
    lift[index].publish({relative_angle_rad, k_uptime_get_32()});
}

/**
 * @brief 读取某一升降轴的最新角度指令
 *
 * @param index 升降轴索引
 * @param data 数据输出参数
 * @return 成功返回 0 参数无效或暂无数据返回错误码
 */
int remote_channel_latest_lift(uint8_t index, lift_angle_topic_data &data)
{
    if (index >= 4) {
        return -EINVAL;
    }
    return lift[index].read(data);
}
