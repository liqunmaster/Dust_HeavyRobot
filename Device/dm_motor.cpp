#include "dm_motor.hpp"

namespace
{
    constexpr uint16_t RegisterCommandId = 0x7FFU;
    constexpr uint8_t ModeRegister = 0x0AU;
    constexpr uint16_t TwelveBitMax = 0x0FFFU;
    constexpr uint16_t SixteenBitMax = 0xFFFFU;

    bool finite(float value)
    {
        return __builtin_isfinite(value);
    }
}

bool dm_motor::valid_mode(DmControlMode mode)
{
    return mode >= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT && mode <= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT;
}

uint16_t dm_motor::control_id(uint8_t motor_id, DmControlMode mode)
{
    return static_cast<uint16_t>(motor_id) + ((static_cast<uint16_t>(mode) - 1U) << 8U);
}

uint16_t dm_motor::encode(float value, float minimum, float maximum, uint16_t maximum_raw)
{
    return static_cast<uint16_t>((value - minimum) * maximum_raw / (maximum - minimum));
}

float dm_motor::decode(uint16_t raw, float limit, uint16_t maximum_raw)
{
    return static_cast<float>(raw) * (2.0F * limit) / maximum_raw - limit;
}

void dm_motor::write_f32(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (uint8_t i = 0U; i < 4U; ++i) {
        dst[i] = static_cast<uint8_t>(bits >> (8U * i));
    }
}

void dm_motor::write_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8U);
}

int dm_motor::init(fdcan_device device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits, fdcan_protocol protocol, uint16_t master_id)
{
    if (device < FDCAN_DEVICE_CAN0 || device >= FDCAN_DEVICE_COUNT || motor_id == 0U || motor_id > 15U || master_id > 0x7FFU || (protocol != FDCAN_PROTOCOL_CLASSIC && protocol != FDCAN_PROTOCOL_FD) ||
        !valid_mode(mode) || !finite(limits.position) || limits.position <= 0.0F || !finite(limits.velocity) || limits.velocity <= 0.0F || !finite(limits.torque) || limits.torque <= 0.0F) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    device_ = device;
    motor_id_ = motor_id;
    master_id_ = master_id;
    mode_ = mode;
    protocol_ = protocol;
    requested_mode_ = mode;
    mode_change_pending_ = false;
    limits_ = limits;
    memset(tx_data_, 0, sizeof(tx_data_));
    tx_length_ = 0U;
    rx_data_ = {};
    data_ = {};
    initialized_ = true;
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    return 0;
}

int dm_motor::set_mit(float position, float velocity, float kp, float kd, float torque)
{
    if (!finite(position) || !finite(velocity) || !finite(kp) || !finite(kd) || !finite(torque)) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    if (position < -limits_.position || position > limits_.position || velocity < -limits_.velocity || velocity > limits_.velocity || torque < -limits_.torque || torque > limits_.torque || kp < 0.0F || kp > 500.0F || kd < 0.0F || kd > 5.0F) {
        k_spin_unlock(&lock_, key);
        return -ERANGE;
    }

    const uint16_t p = encode(position, -limits_.position, limits_.position, SixteenBitMax);
    const uint16_t v = encode(velocity, -limits_.velocity, limits_.velocity, TwelveBitMax);
    const uint16_t k_p = encode(kp, 0.0F, 500.0F, TwelveBitMax);
    const uint16_t k_d = encode(kd, 0.0F, 5.0F, TwelveBitMax);
    const uint16_t t = encode(torque, -limits_.torque, limits_.torque, TwelveBitMax);

    const uint64_t payload = (static_cast<uint64_t>(p) << 48U) | (static_cast<uint64_t>(v) << 36U) | (static_cast<uint64_t>(k_p) << 24U) | (static_cast<uint64_t>(k_d) << 12U) | static_cast<uint64_t>(t);
    sys_put_be64(payload, tx_data_);
    tx_length_ = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::set_position_velocity(float position, float max_velocity)
{
    if (!finite(position) || !finite(max_velocity) || max_velocity < 0.0F) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_f32(&tx_data_[4], max_velocity);
    tx_length_ = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::set_velocity(float velocity)
{
    if (!finite(velocity)) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], velocity);
    memset(&tx_data_[4], 0, 4U);
    tx_length_ = 4U;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::set_position_torque(float position, float max_velocity, float current_ratio)
{
    if (!finite(position) || !finite(max_velocity) || !finite(current_ratio) || max_velocity < 0.0F || max_velocity > 100.0F || current_ratio < 0.0F || current_ratio > 1.0F) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_ || mode_ != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT || mode_change_pending_) {
        k_spin_unlock(&lock_, key);
        return !initialized_ ? -ENODEV : -EPERM;
    }
    write_f32(&tx_data_[0], position);
    write_u16(&tx_data_[4], static_cast<uint16_t>(max_velocity * 100.0F));
    write_u16(&tx_data_[6], static_cast<uint16_t>(current_ratio * 10000.0F));
    tx_length_ = 8U;
    k_spin_unlock(&lock_, key);
    return 0;
}

int dm_motor::make_frame(uint16_t id, const uint8_t *data, uint8_t length, fdcan_frame &frame)
{
    if (data == nullptr || length > 8U) {
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
    if (!initialized_ || mode_change_pending_ || tx_length_ == 0U) {
        const int result = !initialized_ ? -ENODEV : mode_change_pending_ ? -EBUSY
                                                                          : -ENODATA;
        k_spin_unlock(&lock_, key);
        return result;
    }
    id = control_id(motor_id_, mode_);
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
    const uint16_t id = control_id(motor_id_, mode_);
    k_spin_unlock(&lock_, key);
    uint8_t payload[8];
    memset(payload, 0xFF, sizeof(payload));
    payload[7] = command;
    return make_frame(id, payload, sizeof(payload), frame);
}

int dm_motor::build_enable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFCU, frame);
}

int dm_motor::build_disable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFDU, frame);
}

int dm_motor::build_clear_error_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFBU, frame);
}

int dm_motor::build_save_zero_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFEU, frame);
}

int dm_motor::build_mode_frame(DmControlMode mode, fdcan_frame &frame) const
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

    const uint8_t payload[8] = {motor_id, 0U, 0x55U, ModeRegister, static_cast<uint8_t>(mode), 0U, 0U, 0U};
    return make_frame(RegisterCommandId, payload, sizeof(payload), frame);
}

int dm_motor::mark_mode_request(DmControlMode mode)
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
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != protocol_ || frame.length != 8U) {
        return -EBADMSG;
    }

    if (frame.data[0] == motor_id_ && frame.data[1] == 0U && frame.data[2] == 0x55U && frame.data[3] == ModeRegister) {
        const k_spinlock_key_t key = k_spin_lock(&lock_);
        if (mode_change_pending_ && frame.data[4] == static_cast<uint8_t>(requested_mode_) && frame.data[5] == 0U && frame.data[6] == 0U && frame.data[7] == 0U) {
            mode_ = requested_mode_;
            mode_change_pending_ = false;
            tx_length_ = 0U;
            k_spin_unlock(&lock_, key);
            atomic_inc(&feedback_count_);
            return 0;
        }
        k_spin_unlock(&lock_, key);
        return -ENOMSG;
    }

    const uint64_t feedback = sys_get_be64(frame.data);
    const uint8_t id = static_cast<uint8_t>((feedback >> 56U) & 0x0FU);
    if (id != motor_id_) {
        return -ENOMSG;
    }
    DmRxData raw{};
    raw.motor_id = id;
    raw.status = static_cast<uint8_t>((feedback >> 60U) & 0x0FU);
    raw.position = static_cast<uint16_t>((feedback >> 40U) & 0xFFFFU);
    raw.velocity = static_cast<uint16_t>((feedback >> 28U) & 0x0FFFU);
    raw.torque = static_cast<uint16_t>((feedback >> 16U) & 0x0FFFU);
    raw.mos_temperature = static_cast<uint8_t>((feedback >> 8U) & 0xFFU);
    raw.rotor_temperature = static_cast<uint8_t>(feedback & 0xFFU);
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

DmRxData dm_motor::get_rx_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmRxData snapshot = rx_data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

DmData dm_motor::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmData snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

DmControlMode dm_motor::get_mode() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const DmControlMode snapshot = mode_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

uint32_t dm_motor::control_frame_id() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const uint32_t id = initialized_ ? control_id(motor_id_, mode_) : 0U;
    k_spin_unlock(&lock_, key);
    return id;
}
