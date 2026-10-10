#include "dji_motor.hpp"

namespace
{
    int16_t commands[FDCAN_DEVICE_COUNT][2][4]{};
    struct k_spinlock commands_lock{};
}

    /**
     * @brief 校验 CAN 通道编号是否合法
     *
     * @param device CAN 通道编号
     * @return 合法返回 true，否则返回 false
    */
    bool dji_motor::valid_device(fdcan_device device)
    {
        return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
    }

    /**
     * @brief 校验电机 ID 是否在合法范围内
     *
     * @param motor_id 电机 ID
     * @return 合法返回 true，否则返回 false
    */
    bool dji_motor::valid_id(int motor_id)
    {
        return motor_id >= 1 && motor_id <= 8;
    }

    /**
     * @brief 从反馈数据中读取大端序的 16 位无符号值
     *
     * @param data 反馈数据指针
     * @return 读取的 16 位无符号值
    */
    uint16_t dji_motor::read_u16(const uint8_t *data)
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    }

    /**
     * @brief 将原始值解释为有符号 16 位数值
     *
     * @param raw 原始无符号值
     * @return 转换后的有符号值
    */
    int16_t dji_motor::signed_value(uint16_t raw)
    {
        return static_cast<int16_t>(static_cast<int32_t>(raw) - ((raw & 0x8000) != 0 ? 0x10000 : 0));
    }

    /**
     * @brief 校验收到的反馈报文是否与目标电机匹配
     *
     * @param frame 收到的反馈报文
     * @param motor_id 目标电机 ID
     * @return 合法返回 0，否则返回负错误码
    */
    int dji_motor::validate_feedback(const fdcan_frame &frame, uint8_t motor_id)
    {
        if (frame.id != 0x200 + motor_id) {
            return -ENOMSG;
        }
        if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8) {
            return -EBADMSG;
        }
        return 0;
    }

    /**
     * @brief 写入指定电机通道的设定值（按分组存储待打包）
     *
     * @param device CAN 通道编号
     * @param motor_id 电机 ID
     * @param raw 原始设定值
     * @return 成功返回 0，参数非法返回负错误码
    */
    int dji_motor::set(fdcan_device device, uint8_t motor_id, int16_t raw)
    {
        if (!valid_device(device) || !valid_id(motor_id)) {
            return -EINVAL;
        }

        const uint8_t group = (motor_id - 1) / 4;
        const uint8_t slot = (motor_id - 1) % 4;
        const k_spinlock_key_t key = k_spin_lock(&commands_lock);
        commands[device][group][slot] = raw;
        k_spin_unlock(&commands_lock, key);
        return 0;
    }

    /**
     * @brief 依据已写入的一组设定值构造控制报文
     *
     * @param device CAN 通道编号
     * @param motor_id 电机 ID
     * @param frame 输出控制报文字段
     * @return 成功返回 0，参数非法返回负错误码
    */
    int dji_motor::build_control_frame(fdcan_device device, uint8_t motor_id, fdcan_frame &frame)
{
    if (!valid_device(device) || !valid_id(motor_id)) {
        return -EINVAL;
    }

    const uint8_t group = (motor_id - 1) / 4;
    int16_t snapshot[4];
    const k_spinlock_key_t key = k_spin_lock(&commands_lock);
    for (uint8_t slot = 0; slot < 4; ++slot) {
        snapshot[slot] = commands[device][group][slot];
    }
    k_spin_unlock(&commands_lock, key);

    frame = {};
    frame.id = group == 0 ? 0x200 : 0x1FF;
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = 8;

    const uint64_t payload = (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[0])) << 48) |
                             (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[1])) << 32) |
                             (static_cast<uint64_t>(static_cast<uint16_t>(snapshot[2])) << 16) |
                             static_cast<uint64_t>(static_cast<uint16_t>(snapshot[3]));
    sys_put_be64(payload, frame.data);
    return 0;
}