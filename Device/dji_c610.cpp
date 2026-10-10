#include "dji_c610.hpp"

    /**
     * @brief 解码 8 字节原始反馈为大端序结构体
     *
     * @param data 原始反馈数据指针
     * @return 解码后的反馈结构体
    */
    C610RxData c610::decode_feedback(const uint8_t *data)
{
    const uint64_t frame = sys_get_be64(data);
    C610RxData decoded{};
    decoded.encoder = static_cast<uint16_t>((frame >> 48) & 0xFFFF);
    decoded.omega = dji_motor::signed_value(static_cast<uint16_t>((frame >> 32) & 0xFFFF));
    decoded.current = static_cast<uint16_t>((frame >> 16) & 0xFFFF);
    decoded.reserved = static_cast<uint8_t>((frame >> 8) & 0xFF);
    decoded.error = static_cast<uint8_t>(frame & 0xFF);
    return decoded;
}

    /**
     * @brief 初始化 C610 电机，绑定通道、ID 与减速比
     *
     * @param device CAN 通道编号
     * @param id 电机 ID
     * @param gear_ratio 减速比
     * @return 成功返回 0，参数非法返回负错误码
    */
    int c610::init(fdcan_device device, C610_ID id, float gear_ratio)
{
    if (!dji_motor::valid_device(device) || !dji_motor::valid_id(id) || !__builtin_isfinite(gear_ratio) || gear_ratio <= 0.0f) {
        return -EINVAL;
    }

    device_ = device;
    id_ = id;
    gear_ratio_ = gear_ratio;
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    rx_data_ = {};
    data_ = {};
    total_encoder_ = 0;
    encoder_initialized_ = false;
    k_spin_unlock(&feedback_lock_, key);
    atomic_set(&feedback_count_, 0);
    initialized_ = true;
    return 0;
}

    /**
     * @brief 设定电机的目标电流（转换为原始指令写入）
     *
     * @param current 目标电流（安培）
     * @return 成功返回 0，否则返回负错误码
    */
    int c610::set_current(float current)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (!__builtin_isfinite(current)) {
        return -EINVAL;
    }

    const int16_t command = math::current_to_raw(current, current_limit_, current_raw_limit_);
    return dji_motor::set(device_, static_cast<uint8_t>(id_), command);
}

    /**
     * @brief 构造本电机所属分组的控制报文字段
     *
     * @param frame 输出控制报文字段
     * @return 成功返回 0，否则返回负错误码
    */
    int c610::build_control_frame(fdcan_frame &frame) const
{
    if (!initialized_) {
        return -ENODEV;
    }
    return dji_motor::build_control_frame(device_, static_cast<uint8_t>(id_), frame);
}

    /**
     * @brief 更新反馈数据并连续累加计算角度与角速度
     *
     * @param data 原始反馈数据指针
    */
    void c610::unpack_feedback(const uint8_t *data)
{
    const C610RxData next = decode_feedback(data);
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);

    if (!encoder_initialized_) {
        total_encoder_ = next.encoder;
        encoder_initialized_ = true;
    } else {
        int32_t delta = static_cast<int32_t>(next.encoder) - static_cast<int32_t>(data_.pre_encoder);

        if (delta > encoder_resolution_ / 2) {
            delta -= encoder_resolution_;
            --data_.total_round;
        } else if (delta < -encoder_resolution_ / 2) {
            delta += encoder_resolution_;
            ++data_.total_round;
        }
        total_encoder_ += delta;
    }

    rx_data_ = next;
    data_.pre_encoder = next.encoder;
    data_.total_encoder = total_encoder_ > INT32_MAX ? INT32_MAX : total_encoder_ < INT32_MIN ? INT32_MIN
                                                                                              : static_cast<int32_t>(total_encoder_);

    data_.now_current = math::raw_to_current(dji_motor::signed_value(next.current), current_raw_limit_, current_limit_);
    data_.now_omega = math::rpm_to_radian_per_second(next.omega, gear_ratio_);
    data_.now_angle = math::encoder_to_radian(total_encoder_, encoder_resolution_, gear_ratio_);
    k_spin_unlock(&feedback_lock_, key);
}

    /**
     * @brief 处理一帧反馈报文，校验后更新内部状态
     *
     * @param frame 收到的反馈报文
     * @return 成功返回 0，否则返回负错误码
    */
    int c610::process_feedback(const fdcan_frame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    const int result = dji_motor::validate_feedback(frame, static_cast<uint8_t>(id_));
    if (result != 0) {
        return result;
    }

    unpack_feedback(frame.data);
    atomic_inc(&feedback_count_);
    return 0;
}

    /**
     * @brief 获取反馈报文统计计数
     *
     * @return 已处理的反馈次数
    */
    uint32_t c610::get_feedback_count() const
    {
        return static_cast<uint32_t>(atomic_get(&feedback_count_));
    }

    /**
     * @brief 线程安全地获取最近一次原始反馈数据
     *
     * @return 反馈快照结构体
    */
    C610RxData c610::get_rx_data() const
    {
        const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
        const C610RxData snapshot = rx_data_;
        k_spin_unlock(&feedback_lock_, key);
        return snapshot;
    }

    /**
     * @brief 线程安全地获取处理后的数据快照
     *
     * @return 数据快照结构体
    */
    C610Data c610::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    const C610Data snapshot = data_;
    k_spin_unlock(&feedback_lock_, key);
    return snapshot;
}