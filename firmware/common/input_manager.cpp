#include "input_manager.hpp"

#include <cmath>
#include "remote_channel.hpp"
#include "chassis_channel.hpp"

namespace
{
    InputManager input_manager;
}

float InputManager::Axis(uint16_t raw)
{
    float value = (static_cast<float>(raw) - 1024.0f) / 660.0f;
    if (value > 1.0f) {
        value = 1.0f;
    } else if (value < -1.0f) {
        value = -1.0f;
    }
    return std::fabs(value) < 0.05f ? 0.0f : value;
}

void InputManager::PublishSample(const InputSample &sample)
{
    RemoteTopicData command{};
    command.timestamp_ms = sample.timestamp_ms;
    command.protocol = static_cast<uint8_t>(sample.protocol);
    switch (sample.protocol) {
        case RemoteProtocol::dt7:
            command.forward = Axis(sample.dt7_data.channel[2]);
            command.lateral = Axis(sample.dt7_data.channel[3]);
            command.rotation = 0.0f;
            command.switch_left = sample.dt7_data.switch_left;
            command.switch_right = sample.dt7_data.switch_right;
            command.valid = true;
            break;
        case RemoteProtocol::vt03:
            command.switch_left = sample.vt03_data.mode_switch;
            break;
        case RemoteProtocol::vt02:
            break;
        default:
            return;
    }

    remote_channel_publish_input(command);
    if (sample.protocol != RemoteProtocol::dt7) {
        return;
    }
    chassis_channel_publish_setpoint({command.forward * 0.5f, command.lateral * 0.5f, 0.0f, command.timestamp_ms});
    const float lift_angle = Axis(sample.dt7_data.channel[0]) * 12.5f;
    for (uint8_t index = 0; index < 4; ++index) {
        remote_channel_publish_lift(index, {lift_angle, command.timestamp_ms});
    }
}

void InputManager::PublishMcu(uint16_t chassis_x, uint16_t chassis_y, uint16_t rotation, uint8_t flags, uint8_t mouse_lr, uint16_t keyboard, uint32_t timestamp_ms)
{
    const auto axis_value = [](uint16_t raw) {
        float value = (static_cast<float>(raw) - 1024.0f) / 660.0f;
        if (value > 1.0f) {
            value = 1.0f;
        } else if (value < -1.0f) {
            value = -1.0f;
        }
        return std::fabs(value) < 0.05f ? 0.0f : value;
    };

    RemoteTopicData command{};
    command.forward = axis_value(chassis_x);
    command.lateral = axis_value(chassis_y);
    command.rotation = axis_value(rotation);
    command.protocol = static_cast<uint8_t>(RemoteProtocol::dt7);
    command.switch_left = (flags >> 1) & 0x03;

    command.switch_right = (flags & (1u << 5)) != 0 ? 2 : 1;
    command.valid = command.switch_left >= 1 && command.switch_left <= 3;
    command.timestamp_ms = timestamp_ms;
    remote_channel_publish_input(command);

    chassis_channel_publish_setpoint({command.forward * 0.5f, command.lateral * 0.5f, command.rotation * 0.5f, timestamp_ms});

    (void)mouse_lr;
    (void)keyboard;
}

void input_manager_publish_sample(const InputSample &sample)
{
    input_manager.PublishSample(sample);
}

void input_manager_publish_mcu(uint16_t x, uint16_t y, uint16_t rotation,
                               uint8_t flags, uint8_t mouse_lr, uint16_t keyboard, uint32_t timestamp_ms)
{
    input_manager.PublishMcu(x, y, rotation, flags, mouse_lr, keyboard, timestamp_ms);
}
