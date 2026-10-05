#include "dji_motor.hpp"

namespace
{
    int16_t commands[FDCAN_DEVICE_COUNT][2][4]{};
    struct k_spinlock commands_lock{};
}

bool dji_motor::valid_device(fdcan_device device)
{
    return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
}

bool dji_motor::valid_id(int motor_id)
{
    return motor_id >= 1 && motor_id <= 8;
}

uint16_t dji_motor::read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
}

int16_t dji_motor::signed_value(uint16_t raw)
{
    return static_cast<int16_t>(static_cast<int32_t>(raw) - ((raw & 0x8000U) != 0U ? 0x10000 : 0));
}

int dji_motor::validate_feedback(const fdcan_frame &frame, uint8_t motor_id)
{
    if (frame.id != 0x200U + motor_id) {
        return -ENOMSG;
    }
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8U) {
        return -EBADMSG;
    }
    return 0;
}

int dji_motor::set(fdcan_device device, uint8_t motor_id, int16_t raw)
{
    if (!valid_device(device) || !valid_id(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1U) / 4U;
    const uint8_t slot = (motor_id - 1U) % 4U;
    const k_spinlock_key_t key = k_spin_lock(&commands_lock);
    commands[device][group][slot] = raw;
    k_spin_unlock(&commands_lock, key);
    return 0;
}

int dji_motor::build_control_frame(fdcan_device device, uint8_t motor_id, fdcan_frame &frame)
{
    if (!valid_device(device) || !valid_id(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1U) / 4U;
    int16_t snapshot[4];
    const k_spinlock_key_t key = k_spin_lock(&commands_lock);
    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        snapshot[slot] = commands[device][group][slot];
    }
    k_spin_unlock(&commands_lock, key);

    frame = {};
    frame.id = group == 0U ? 0x200U : 0x1FFU;
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = 8U;

    const uint64_t payload = (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[0])) << 48U) |
                             (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[1])) << 32U) |
                             (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[2])) << 16U) |
                             static_cast<uint64_t>(static_cast<uint16_t>(snapshot[3]));
    sys_put_be64(payload, frame.data);
    return 0;
}
