#include "cubemars.hpp"

namespace
{
    /**
     * @brief 将浮点值限制在指定的最小/最大值之间
     *
     * @param value 待限幅的浮点值
     * @param minimum 下限值
     * @param maximum 上限值
     * @return clamp 后的浮点值
    */
    float clamp(float value, float minimum, float maximum)
    {
        if (value < minimum) {
            return minimum;
        }
        if (value > maximum) {
            return maximum;
        }
        return value;
    }
}

    /**
     * @brief 将浮点值线性映射为指定比特数目的无符号整数
     *
     * @param value 待编码的浮点值
     * @param minimum 编码范围下限
     * @param maximum 编码范围上限
     * @param bits 编码位数
     * @return 编码后的无符号整数
    */
    uint16_t cubemars::encode(float value, float minimum, float maximum, uint8_t bits)
{
    const uint32_t maximum_raw = (1L << bits) - 1L;
    return static_cast<uint16_t>((clamp(value, minimum, maximum) - minimum) * maximum_raw / (maximum - minimum));
}

    /**
     * @brief 将无符号整数解码为指定限幅范围内的浮点值
     *
     * @param raw 待解码的无符号整数
     * @param limit 正负对称的限幅值
     * @param bits 原始数据比特数
     * @return 解码后的浮点值
    */
    float cubemars::decode(uint16_t raw, float limit, uint8_t bits)
{
    const uint32_t maximum_raw = (1L << bits) - 1L;
    return static_cast<float>(raw) * (2.0f * limit) / maximum_raw - limit;
}

    /**
     * @brief 将角度差值包裹回对称范围，用于处理多圈累计角度
     *
     * @param delta 角度差值
     * @param angle_max 单圈限幅值
     * @return 包裹后的角度差值
    */
    float cubemars::wrap_delta(float delta, float angle_max)
{
    if (delta > angle_max) {
        delta -= 2.0f * angle_max;
    } else if (delta < -angle_max) {
        delta += 2.0f * angle_max;
    }
    return delta;
}

    /**
     * @brief 初始化 CubeMars 电机并绑定 FDCAN 通道
     *
     * @param device FDCAN 通道枚举
     * @param angle_max 角度限幅
     * @param omega_max 角速度限幅
     * @param torque_max 力矩限幅
     * @return 成功返回 0，失败返回负错误码
    */
    int cubemars::init(fdcan_device device, float angle_max, float omega_max, float torque_max)
{
    if (device < FDCAN_DEVICE_CAN0 || device >= FDCAN_DEVICE_COUNT || !__builtin_isfinite(angle_max) || angle_max <= 0.0f || !__builtin_isfinite(omega_max) || omega_max <= 0.0f || !__builtin_isfinite(torque_max) || torque_max <= 0.0f) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    device_ = device;
    angle_max_ = angle_max;
    omega_max_ = omega_max;
    torque_max_ = torque_max;
    target_angle_ = 0.0f;
    target_omega_ = 0.0f;
    target_torque_ = 0.0f;
    target_kp_ = 0.0f;
    target_kd_ = 0.0f;
    data_ = {};
    raw_angle_initialized_ = false;
    last_raw_angle_ = 0.0f;
    initialized_ = true;
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    return 0;
}

    /**
     * @brief 设置 MIT 模式的目标位置/速度/刚度/阻尼/力矩
     *
     * @param position 目标位置
     * @param velocity 目标角速度
     * @param kp 刚度系数
     * @param kd 阻尼系数
     * @param torque 前馈力矩
     * @return 成功返回 0，未初始化返回 -ENODEV，越界返回 -ERANGE
    */
    int cubemars::set_mit(float position, float velocity, float kp, float kd, float torque)
{
    if (!__builtin_isfinite(position) || !__builtin_isfinite(velocity) || !__builtin_isfinite(kp) || !__builtin_isfinite(kd) || !__builtin_isfinite(torque)) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    if (position < -angle_max_ || position > angle_max_ || velocity < -omega_max_ || velocity > omega_max_ || torque < -torque_max_ || torque > torque_max_ || kp < 0.0f || kp > kp_max_ || kd < 0.0f || kd > kd_max_) {
        k_spin_unlock(&lock_, key);
        return -ERANGE;
    }

    target_angle_ = position;
    target_omega_ = velocity;
    target_kp_ = kp;
    target_kd_ = kd;
    target_torque_ = torque;
    k_spin_unlock(&lock_, key);
    return 0;
}

    /**
     * @brief 将 MIT 模式的五路浮点目标压缩打包为 8 字节指令负载
     *
     * @param position 目标位置
     * @param velocity 目标角速度
     * @param kp 刚度系数
     * @param kd 阻尼系数
     * @param torque 前馈力矩
     * @param out 输出的 8 字节负载数组
     * @param position_max 位置限幅
     * @param velocity_max 速度限幅
     * @param kp_max 刚度上限
     * @param kd_max 阻尼上限
     * @param torque_max 力矩限幅
    */
    void cubemars::pack_mit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max)
{
    if (out == nullptr || position_max <= 0.0f || velocity_max <= 0.0f || kp_max <= 0.0f || kd_max <= 0.0f || torque_max <= 0.0f || !__builtin_isfinite(position) || !__builtin_isfinite(velocity) || !__builtin_isfinite(kp) || !__builtin_isfinite(kd) || !__builtin_isfinite(torque)) {
        return;
    }

    const uint16_t p = encode(position, -position_max, position_max, 16);
    const uint16_t v = encode(velocity, -velocity_max, velocity_max, 12);
    const uint16_t k_p = encode(kp, 0.0f, kp_max, 12);
    const uint16_t k_d = encode(kd, 0.0f, kd_max, 12);
    const uint16_t t = encode(torque, -torque_max, torque_max, 12);

    out[0] = static_cast<uint8_t>(p >> 8);
    out[1] = static_cast<uint8_t>(p);
    out[2] = static_cast<uint8_t>(v >> 4);
    out[3] = static_cast<uint8_t>((v & 0x0F) << 4 | (k_p >> 8));
    out[4] = static_cast<uint8_t>(k_p);
    out[5] = static_cast<uint8_t>(k_d >> 4);
    out[6] = static_cast<uint8_t>((k_d & 0x0F) << 4 | (t >> 8));
    out[7] = static_cast<uint8_t>(t);
}

    /**
     * @brief 构建 MIT 模式控制帧
     *
     * @param frame 输出的 FDCAN 帧
     * @return 成功返回 0，未初始化返回 -ENODEV
    */
    int cubemars::build_control_frame(fdcan_frame &frame) const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const float position = target_angle_;
    const float velocity = target_omega_;
    const float torque = target_torque_;
    const float kp = target_kp_;
    const float kd = target_kd_;
    const float position_max = angle_max_;
    const float velocity_max = omega_max_;
    const float torque_max = torque_max_;
    k_spin_unlock(&lock_, key);

    frame = {};
    frame.id = control_frame_id();
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = 8;
    pack_mit(position, velocity, kp, kd, torque, frame.data,
             position_max, velocity_max, kp_max_, kd_max_, torque_max);
    return 0;
}

    /**
     * @brief 构建通用命令帧，数据尾字节由 tail 指定
     *
     * @param tail 命令字节（使能/失能/存零等）
     * @param frame 输出的 FDCAN 帧
     * @return 成功返回 0，未初始化返回 -ENODEV
    */
    int cubemars::build_command_frame(uint8_t tail, fdcan_frame &frame) const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool ready = initialized_;
    k_spin_unlock(&lock_, key);
    if (!ready) {
        return -ENODEV;
    }

    frame = {};
    frame.id = control_frame_id();
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = 8;
    for (uint8_t index = 0; index < 7; ++index) {
        frame.data[index] = 0xFF;
    }
    frame.data[7] = tail;
    return 0;
}

    /**
     * @brief 构建使能命令帧
     *
     * @param frame 输出的 FDCAN 帧
     * @return 成功返回 0，否则返回错误码
    */
    int cubemars::build_enable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFC, frame);
}

    /**
     * @brief 构建失能命令帧
     *
     * @param frame 输出的 FDCAN 帧
     * @return 成功返回 0，否则返回错误码
    */
    int cubemars::build_disable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFD, frame);
}

    /**
     * @brief 构建保存当前角度为零点的命令帧
     *
     * @param frame 输出的 FDCAN 帧
     * @return 成功返回 0，否则返回错误码
    */
    int cubemars::build_save_zero_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFE, frame);
}

    /**
     * @brief 处理电机反馈帧，解算角度/角速度/力矩并累计多圈角度
     *
     * @param frame 收到的 FDCAN 反馈帧
     * @return 成功返回 0，ID 不匹配返回 -ENOMSG，格式错误返回 -EBADMSG
    */
    int cubemars::process_feedback(const fdcan_frame &frame)
{
    if (frame.id != feedback_id()) {
        return -ENOMSG;
    }
    // CubeMars MIT feedback is a six-byte payload: id + p16 + v12 + t12.
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC ||
        frame.length != 6) {
        return -EBADMSG;
    }

    const uint16_t position = static_cast<uint16_t>(frame.data[1]) << 8 | frame.data[2];
    const uint16_t velocity = static_cast<uint16_t>(frame.data[3]) << 4 | (frame.data[4] >> 4);
    const uint16_t torque = static_cast<uint16_t>(frame.data[4] & 0x0F) << 8 | frame.data[5];

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const float angle = decode(position, angle_max_, 16);
    data_.now_angle = angle;
    data_.now_omega = decode(velocity, omega_max_, 12);
    data_.now_torque = decode(torque, torque_max_, 12);
    if (!raw_angle_initialized_) {
        data_.now_total_angle = angle;
        raw_angle_initialized_ = true;
    } else {
        data_.now_total_angle += wrap_delta(angle - last_raw_angle_, angle_max_);
    }
    last_raw_angle_ = angle;
    k_spin_unlock(&lock_, key);
    atomic_inc(&feedback_count_);
    return 0;
}

    /**
     * @brief 获取电机当前反馈数据的快照
     *
     * @return 反馈数据快照
    */
    CubemarsData cubemars::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const CubemarsData snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

    /**
     * @brief 获取已处理的反馈帧计数
     *
     * @return 反馈帧累计次数
    */
    uint32_t cubemars::get_feedback_count() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}