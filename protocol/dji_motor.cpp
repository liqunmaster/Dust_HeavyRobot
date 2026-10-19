#include "dji_motor.hpp"

namespace
{
    struct DjiCommandBank
    {
        int16_t values[CANFD_DEVICE_COUNT][2][4]{};
        struct k_spinlock lock{};
    } commands;
}

bool DjiMotor::ValidDevice(CanfdDevice device)
{
    return device >= CANFD_DEVICE_CAN0 && device < CANFD_DEVICE_COUNT;
}

bool DjiMotor::ValidId(int motor_id)
{
    return motor_id >= 1 && motor_id <= 8;
}

uint16_t DjiMotor::ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t DjiMotor::SignedValue(uint16_t raw)
{
    return static_cast<int16_t>(raw);
}

int DjiMotor::ValidateFeedback(const CanfdFrame &frame, uint8_t motor_id)
{
    if (frame.id_type != CANFD_ID_STANDARD || frame.protocol != CANFD_PROTOCOL_CLASSIC) {
        return -EBADMSG;
    }
    if (frame.id != 0x200U + motor_id) {
        return -ENOMSG;
    }
    return frame.length == 8 ? 0 : -EBADMSG;
}

DjiMotor::Feedback DjiMotor::DecodeFeedback(const uint8_t data[8])
{
    Feedback decoded{};
    decoded.encoder = ReadU16(data);
    decoded.omega = SignedValue(ReadU16(data + 2));
    decoded.current = ReadU16(data + 4);
    decoded.auxiliary_0 = data[6];
    decoded.auxiliary_1 = data[7];
    return decoded;
}

void DjiMotor::EncodeControl(const int16_t current[4], uint8_t data[8])
{
    for (uint8_t index = 0; index < 4; ++index) {
        const uint16_t raw = static_cast<uint16_t>(current[index]);
        data[index * 2] = static_cast<uint8_t>(raw >> 8);
        data[index * 2 + 1] = static_cast<uint8_t>(raw);
    }
}

int DjiMotor::Set(CanfdDevice device, uint8_t motor_id, int16_t raw)
{
    if (!ValidDevice(device) || !ValidId(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1) / 4;
    const uint8_t slot = (motor_id - 1) % 4;
    const k_spinlock_key_t key = k_spin_lock(&commands.lock);
    commands.values[device][group][slot] = raw;
    k_spin_unlock(&commands.lock, key);
    return 0;
}

int DjiMotor::BuildControlFrame(CanfdDevice device, uint8_t motor_id, CanfdFrame &frame)
{
    if (!ValidDevice(device) || !ValidId(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1) / 4;
    int16_t snapshot[4];
    const k_spinlock_key_t key = k_spin_lock(&commands.lock);
    for (uint8_t slot = 0; slot < 4; ++slot) {
        snapshot[slot] = commands.values[device][group][slot];
    }
    k_spin_unlock(&commands.lock, key);

    frame = {};
    frame.id = group == 0 ? 0x200 : 0x1FF;
    frame.id_type = CANFD_ID_STANDARD;
    frame.protocol = CANFD_PROTOCOL_CLASSIC;
    frame.bitrate_switch = CANFD_BRS_DISABLED;
    frame.length = 8;

    EncodeControl(snapshot, frame.data);
    return 0;
}
