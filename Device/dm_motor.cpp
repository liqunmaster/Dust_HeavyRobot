#include "dm_motor.hpp"

namespace
{
    constexpr uint16_t RegisterCommandId = 0x7FF;
    constexpr uint8_t ModeRegister = 0x0A;
    constexpr uint16_t TwelveBitMax = 0x0FFF;
    constexpr uint16_t SixteenBitMax = 0xFFFF;

    /**
     * @brief 判断浮点值是否为有限数
     *
     * @param value 待判断值
     * @return 有限返回 true 否则返回 false
     */
    bool finite(float value)
    {
        return __builtin_isfinite(value);
    }
}

/**
 * @brief 校验控制模式是否合法
 *
 * @param mode 控制模式
 * @return 合法返回 true 否则返回 false
 */
bool dm_motor::valid_mode(dm_control_mode mode)
{
    return mode >= dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT && mode <= dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT;
}

/**
 * @brief 根据收发基址与控制模式计算控制报文 ID
 *
 * @param tx_id_base 发送 ID 基址
 * @param mode 控制模式
 * @return 控制报文 ID
 */
uint16_t dm_motor::control_id(uint16_t tx_id_base, dm_control_mode mode)
{
    return tx_id_base + ((static_cast<uint16_t>(mode) - 1) << 8);
}

/**
 * @brief 将物理值编码为指定量程内的原始整数
 *
 * @param value 物理值
 * @param minimum 量程下限
 * @param maximum 量程上限
 * @param maximum_raw 原始值最大值
 * @return 编码后的原始整数
 */
uint16_t dm_motor::encode(float value, float minimum, float maximum, uint16_t maximum_raw)
{
    return static_cast<uint16_t>((value - minimum) * maximum_raw / (maximum - minimum));
}

/**
 * @brief 将原始整数解码为物理值
 *
 * @param raw 原始整数
 * @param limit 量程绝对值
 * @param maximum_raw 原始值最大值
 * @return 解码后的物理值
 */
float dm_motor::decode(uint16_t raw, float limit, uint16_t maximum_raw)
{
    return static_cast<float>(raw) * (2.0f * limit) / maximum_raw - limit;
}

/**
 * @brief 以小端序写入 32 位浮点数据
 *
 * @param dst 目标缓冲区
 * @param value 待写入浮点值
 */
void dm_motor::write_f32(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (uint8_t i = 0; i < 4; ++i) {
        dst[i] = static_cast<uint8_t>(bits >> (8 * i));
    }
}

/**
 * @brief 以小端序写入 16 位无符号数据
 *
 * @param dst 目标缓冲区
 * @param value 待写入的 16 位值
 */
void dm_motor::write_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8);
}

/**
 * @brief 初始化达妙电机 绑定通道、ID、模式与量程
 *
 * @param device CAN 通道编号
 * @param motor_id 电机 ID
 * @param mode 控制模式
 * @param limits MIT 量程限制
 * @param protocol 反馈协议类型
 * @param master_id 主机ID
 * @param tx_id_base 发送 ID 基址
 * @return 成功返回 0 参数非法返回负错误码
 */
int dm_motor::init(fdcan_device device, uint8_t motor_id, dm_control_mode mode, dm_mit_limits limits, fdcan_protocol protocol, uint16_t master_id, uint16_t tx_id_base)
{
    if (device < FDCAN_DEVICE_CAN0 || device >= FDCAN_DEVICE_COUNT || motor_id == 0 || motor_id > 15 || master_id > 0x7FF || tx_id_base > 0x4FF || (protocol != FDCAN_PROTOCOL_CLASSIC && protocol != FDCAN_PROTOCOL_FD) ||
    !valid_mode(mode) || !finite(limits.position) || limits.position <= 0.0f || !finite(limits.velocity) || limits.velocity <= 0.0f || !finite(limits.torque) || limits.torque <= 0.0f) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    device_ = device;
    motor_id_ = motor_id;
    master_id_ = master_id;
    tx_id_base_ = tx_id_base == 0 ? motor_id : tx_id_base;
    mode_ = mode;
    protocol_ = protocol;
    requested_mode_ = mode;
    mode_change_pending_ = false;
    limits_ = limits;
    memset(tx_data_, 0, sizeof(tx_data_));
    tx_length_ = 0;
    rx_data_ = {};
    data_ = {};
    initialized_ = true;
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    return 0;
}

/**
 * @brief 设置 MIT 模式下的目标位置、速度与刚度参数
 *
 * @param position 目标位置
 * @param velocity 目标速度
 * @param kp 位置刚度
 * @param kd 阻尼系数
 * @param torque 目标力矩
 * @return 成功返回 0 否则返回负错误码
 */
int dm_motor::set_mit(float position, float velocity, float kp, float kd, float torque)
{
    if (!finite(position) || !finite(velocity) || !finite(kp) || !finite(kd) || !finite(torque)) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    if (position < -limits_.position || position > limits_.position || velocity < -limits_.velocity || velocity > limits_.velocity || torque < -limits_.torque || torque > limits_.torque || kp < 0.0f || kp > 500.0f || kd < 0.0f || kd > 5.0f) {
        k_spin_unlock(&lock_, key);
        return -ERANGE;
    }

    const uint16_t p = encode(position, -limits_.position, limits_.position, SixteenBitMax);
    const uint16_t v = encode(velocity, -limits_.velocity, limits_.velocity, TwelveBitMax);
    const uint16_t k_p = encode(kp, 0.0f, 500.0f, TwelveBitMax);
    const uint16_t k_d = encode(kd, 0.0f, 5.0f, TwelveBitMax);
    const uint16_t t = encode(torque, -limits_.torque, limits_.torque, TwelveBitMax);

    const uint64_t payload = (static_cast<uint64_t>(p) << 48) | (static_cast<uint64_t>(v) << 36) | (static_cast<uint64_t>(k_p) << 24) | (static_cast<uint64_t>(k_d) << 12) | static_cast<uint64_t>(t);
    sys_put_be64(payload, tx_data_);
    tx_length_ = 8;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 设置角度-角速度模式下的目标位置与限速
 *
 * @param position 目标位置
 * @param max_velocity 最大速度
 * @return 成功返回 0 否则返回负错误码
 */
int dm_motor::set_position_velocity(float position, float max_velocity)
{
    if (!finite(position) || !finite(max_velocity) || max_velocity < 0.0f) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_f32(&tx_data_[4], max_velocity);
    tx_length_ = 8;
    k_spin_unlock(&lock_, key);
    return 0;
}

/**
 * @brief 设置速度模式下的目标速度
 *
 * @param velocity 目标速度
 * @return 成功返回 0 否则返回负错误码
 */
int dm_motor::set_velocity(float velocity)
{
    if (!finite(velocity)) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], velocity);
    memset(&tx_data_[4], 0, 4);
    tx_length_ = 4;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::set_position_torque(float position, float max_velocity, float current_ratio)
{
    if (!finite(position) || !finite(max_velocity) || !finite(current_ratio) || max_velocity < 0.0f || max_velocity > 100.0f || current_ratio < 0.0f || current_ratio > 1.0f) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_u16(&tx_data_[4], static_cast<uint16_t>(max_velocity * 100.0f));
    write_u16(&tx_data_[6], static_cast<uint16_t>(current_ratio * 10000.0f));
    tx_length_ = 8;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::make_frame(uint16_t id, const uint8_t *data, uint8_t length, fdcan_frame &frame)
{
    if (data == nullptr || length > 8) {
        return -EINVAL;
    }
    frame = {};
    frame.id = id;
    frame.id_type = FDCAN_ID_STANDARD;
    frame.protocol = FDCAN_PROTOCOL_CLASSIC;
    frame.bitrate_switch = FDCAN_BRS_DISABLED;
    frame.length = length;
    memcpy(frame.data, data, length);
    return 0;
}

int dm_motor::build_control_frame(fdcan_frame &frame) const
{
    uint8_t payload[8];
    uint8_t length;
    uint16_t id;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_change_pending_ || tx_length_ == 0) {
        const int result = !initialized_ ? -ENODEV : mode_change_pending_ ? -EBUSY : -ENODATA;
        k_spin_unlock(&lock_, key);
        return result;
    }
    id = control_id(tx_id_base_, mode_);
    length = tx_length_;
    memcpy(payload, tx_data_, length);
    k_spin_unlock(&lock_, key);
    return make_frame(id, payload, length, frame);
}

int dm_motor::build_command_frame(uint8_t command, fdcan_frame &frame) const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const uint16_t id = control_id(tx_id_base_, mode_);
    k_spin_unlock(&lock_, key);
    uint8_t payload[8];
    memset(payload, 0xFF, sizeof(payload));
    payload[7] = command;
    return make_frame(id, payload, sizeof(payload), frame);
}

int dm_motor::build_enable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFC, frame);
}

int dm_motor::build_disable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFD, frame);
}

int dm_motor::build_clear_error_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFB, frame);
}

int dm_motor::build_save_zero_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFE, frame);
}

int dm_motor::build_mode_frame(dm_control_mode mode, fdcan_frame &frame) const
{
    if (!valid_mode(mode)) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || (mode_change_pending_ && requested_mode_ != mode)) {
        const int result = !initialized_ ? -ENODEV : -EBUSY;
        k_spin_unlock(&lock_, key);
        return result;
    }
    if (mode == mode_ && !mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return -EALREADY;
    }
    const uint8_t motor_id = motor_id_;
    k_spin_unlock(&lock_, key);

    const uint8_t payload[8] = {motor_id, 0, 0x55, ModeRegister, static_cast<uint8_t>(mode), 0, 0, 0};
    return make_frame(RegisterCommandId, payload, sizeof(payload), frame);
}

int dm_motor::mark_mode_request(dm_control_mode mode)
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || (mode_change_pending_ && requested_mode_ != mode)) {
        const int result = !initialized_ ? -ENODEV : -EBUSY;
        k_spin_unlock(&lock_, key);
        return result;
    }
    requested_mode_ = mode;
    mode_change_pending_ = true;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::process_feedback(const fdcan_frame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (frame.id != master_id_) {
        return -ENOMSG;
    }
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != protocol_ || frame.length != 8) {
        return -EBADMSG;
    }

    if (frame.data[0] == motor_id_ && frame.data[1] == 0 && frame.data[2] == 0x55 && frame.data[3] == ModeRegister) {
        const k_spinlock_key_t key = k_spin_lock(&lock_);
        if (mode_change_pending_ && frame.data[4] == static_cast<uint8_t>(requested_mode_) && frame.data[5] == 0 && frame.data[6] == 0 && frame.data[7] == 0) {
            mode_ = requested_mode_;
            mode_change_pending_ = false;
            tx_length_ = 0;
            k_spin_unlock(&lock_, key);
            atomic_inc(&feedback_count_);
            return 0;
        }
        k_spin_unlock(&lock_, key);
        return -ENOMSG;
    }

    const uint64_t feedback = sys_get_be64(frame.data);
    const uint8_t id = static_cast<uint8_t>((feedback >> 56) & 0x0F);
    if (id != motor_id_) {
        return -ENOMSG;
    }
    dm_rx_data raw{};
    raw.motor_id = id;
    raw.status = static_cast<uint8_t>((feedback >> 60) & 0x0F);
    raw.position = static_cast<uint16_t>((feedback >> 40) & 0xFFFF);
    raw.velocity = static_cast<uint16_t>((feedback >> 28) & 0x0FFF);
    raw.torque = static_cast<uint16_t>((feedback >> 16) & 0x0FFF);
    raw.mos_temperature = static_cast<uint8_t>((feedback >> 8) & 0xFF);
    raw.rotor_temperature = static_cast<uint8_t>(feedback & 0xFF);
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    rx_data_ = raw;
    data_ = {
        decode(raw.position, limits_.position, SixteenBitMax),
        decode(raw.velocity, limits_.velocity, TwelveBitMax),
        decode(raw.torque, limits_.torque, TwelveBitMax),
        static_cast<float>(raw.mos_temperature),
        static_cast<float>(raw.rotor_temperature),
        raw.status,
    };
    k_spin_unlock(&lock_, key);
    atomic_inc(&feedback_count_);
    return 0;
}

uint32_t dm_motor::get_feedback_count() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

dm_rx_data dm_motor::get_rx_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const dm_rx_data snapshot = rx_data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

dm_data dm_motor::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const dm_data snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

dm_control_mode dm_motor::get_mode() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const dm_control_mode snapshot = mode_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

uint32_t dm_motor::control_frame_id() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const uint32_t id = initialized_ ? control_id(tx_id_base_, mode_) : 0;
    k_spin_unlock(&lock_, key);
    return id;
}
