#include "mcu.hpp"

#include <string.h>

namespace
{

    uint16_t ReadU16Be(const uint8_t *data)
    {
        return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) << 8 | data[1]);
    }

    void WriteU16Be(uint8_t *data, uint16_t value)
    {
        data[0] = static_cast<uint8_t>(value >> 8);
        data[1] = static_cast<uint8_t>(value);
    }
}

bool mcu_decode_frame(const uint8_t *data, uint8_t length, McuDecodedFrame &frame)
{
    if (data == nullptr || length != 8) {
        return false;
    }

    frame = {};
    frame.type = data[0];
    switch (frame.type) {
        case MCU_FRAME_CHASSIS:
            frame.chassis.x = ReadU16Be(&data[1]);
            frame.chassis.y = ReadU16Be(&data[3]);
            frame.chassis.rotation = ReadU16Be(&data[5]);
            frame.chassis.flags = data[7];
            return frame.chassis.x <= 2047 && frame.chassis.y <= 2047 && frame.chassis.rotation <= 2047;
        case MCU_FRAME_COMMAND:
            frame.command.mouse_lr = data[1];
            frame.command.keyboard = ReadU16Be(&data[2]);
            memcpy(&frame.command.imu_yaw_rad, &data[4], sizeof(frame.command.imu_yaw_rad));

            return (data[7] & 0x7Fu) != 0x7Fu || (data[6] & 0x80u) == 0;
        case MCU_FRAME_AUTOAIM:
        case MCU_FRAME_REFEREE:
            return true;
        default:
            return false;
    }
}

void mcu_encode_chassis(const McuChassisCommand &command, uint8_t data[8])
{
    memset(data, 0, 8);
    data[0] = MCU_FRAME_CHASSIS;
    WriteU16Be(&data[1], command.x);
    WriteU16Be(&data[3], command.y);
    WriteU16Be(&data[5], command.rotation);
    data[7] = command.flags;
}

void mcu_encode_command(const McuCommandCommand &command, uint8_t data[8])
{
    memset(data, 0, 8);
    data[0] = MCU_FRAME_COMMAND;
    data[1] = command.mouse_lr;
    WriteU16Be(&data[2], command.keyboard);
    memcpy(&data[4], &command.imu_yaw_rad, sizeof(command.imu_yaw_rad));
}

void mcu_encode_autoaim(uint8_t data[8])
{
    memset(data, 0, 8);
    data[0] = MCU_FRAME_AUTOAIM;
}

void mcu_encode_referee(uint8_t data[8])
{
    memset(data, 0, 8);
    data[0] = MCU_FRAME_REFEREE;
}
