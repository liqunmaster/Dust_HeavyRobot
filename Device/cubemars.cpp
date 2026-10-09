#include "cubemars.hpp"

namespace
{
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

uint16_t cubemars::encode(float value, float minimum, float maximum, uint8_t bits)
{
    const uint32_t maximum_raw = (1UL << bits) - 1UL;
    return static_cast<uint16_t>((clamp(value, minimum, maximum) - minimum) * maximum_raw / (maximum - minimum));
}

float cubemars::decode(uint16_t raw, float limit, uint8_t bits)
{
    const uint32_t maximum_raw = (1UL << bits) - 1UL;
    return static_cast<float>(raw) * (2.0F * limit) / maximum_raw - limit;
}

float cubemars::wrap_delta(float delta, float angle_max)
{
    if (delta > angle_max) {
        delta -= 2.0F * angle_max;
    } else if (delta < -angle_max) {
        delta += 2.0F * angle_max;
    }
    return delta;
}

int cubemars::init(fdcan_device device, float angle_max, float omega_max, float torque_max)
{
    if (device < FDCAN_DEVICE_CAN0 || device >= FDCAN_DEVICE_COUNT || !__builtin_isfinite(angle_max) || angle_max <= 0.0F || !__builtin_isfinite(omega_max) || omega_max <= 0.0F || !__builtin_isfinite(torque_max) || torque_max <= 0.0F) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    device_ = device;
    angle_max_ = angle_max;
    omega_max_ = omega_max;
    torque_max_ = torque_max;
    target_angle_ = 0.0F;
    target_omega_ = 0.0F;
    target_torque_ = 0.0F;
    target_kp_ = 0.0F;
    target_kd_ = 0.0F;
    data_ = {};
    raw_angle_initialized_ = false;
    last_raw_angle_ = 0.0F;
    initialized_ = true;
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    return 0;
}

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
    if (position < -angle_max_ || position > angle_max_ || velocity < -omega_max_ || velocity > omega_max_ || torque < -torque_max_ || torque > torque_max_ || kp < 0.0F || kp > kp_max_ || kd < 0.0F || kd > kd_max_) {
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

void cubemars::pack_mit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max)
{
    if (out == nullptr || position_max <= 0.0F || velocity_max <= 0.0F || kp_max <= 0.0F || kd_max <= 0.0F || torque_max <= 0.0F || !__builtin_isfinite(position) || !__builtin_isfinite(velocity) || !__builtin_isfinite(kp) || !__builtin_isfinite(kd) || !__builtin_isfinite(torque)) {
        return;
    }

    const uint16_t p = encode(position, -position_max, position_max, 16U);
    const uint16_t v = encode(velocity, -velocity_max, velocity_max, 12U);
    const uint16_t k_p = encode(kp, 0.0F, kp_max, 12U);
    const uint16_t k_d = encode(kd, 0.0F, kd_max, 12U);
    const uint16_t t = encode(torque, -torque_max, torque_max, 12U);

    out[0] = static_cast<uint8_t>(p >> 8U);
    out[1] = static_cast<uint8_t>(p);
    out[2] = static_cast<uint8_t>(v >> 4U);
    out[3] = static_cast<uint8_t>((v & 0x0FU) << 4U | (k_p >> 8U));
    out[4] = static_cast<uint8_t>(k_p);
    out[5] = static_cast<uint8_t>(k_d >> 4U);
    out[6] = static_cast<uint8_t>((k_d & 0x0FU) << 4U | (t >> 8U));
    out[7] = static_cast<uint8_t>(t);
}

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
    frame.length = 8U;
    pack_mit(position, velocity, kp, kd, torque, frame.data,
             position_max, velocity_max, kp_max_, kd_max_, torque_max);
    return 0;
}

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
    frame.length = 8U;
    for (uint8_t index = 0U; index < 7U; ++index) {
        frame.data[index] = 0xFFU;
    }
    frame.data[7] = tail;
    return 0;
}

int cubemars::build_enable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFCU, frame);
}

int cubemars::build_disable_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFDU, frame);
}

int cubemars::build_save_zero_frame(fdcan_frame &frame) const
{
    return build_command_frame(0xFEU, frame);
}

int cubemars::process_feedback(const fdcan_frame &frame)
{
    if (frame.id != feedback_id()) {
        return -ENOMSG;
    }
    // CubeMars MIT feedback is a six-byte payload: id + p16 + v12 + t12.
    if (frame.id_type != FDCAN_ID_STANDARD || frame.protocol != FDCAN_PROTOCOL_CLASSIC ||
        frame.length != 6U) {
        return -EBADMSG;
    }

    const uint16_t position = static_cast<uint16_t>(frame.data[1]) << 8U | frame.data[2];
    const uint16_t velocity = static_cast<uint16_t>(frame.data[3]) << 4U | (frame.data[4] >> 4U);
    const uint16_t torque = static_cast<uint16_t>(frame.data[4] & 0x0FU) << 8U | frame.data[5];

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const float angle = decode(position, angle_max_, 16U);
    data_.now_angle = angle;
    data_.now_omega = decode(velocity, omega_max_, 12U);
    data_.now_torque = decode(torque, torque_max_, 12U);
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

CubemarsData cubemars::get_data() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const CubemarsData snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

uint32_t cubemars::get_feedback_count() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}
