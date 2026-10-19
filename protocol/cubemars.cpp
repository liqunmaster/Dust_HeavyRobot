#include "cubemars.hpp"

namespace
{

    float Clamp(float value, float minimum, float maximum)
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

float Cubemars::Decode(uint16_t raw, float limit, uint8_t bits)
{
    const uint32_t maximum_raw = (1U << bits) - 1U;
    return static_cast<float>(raw) * (2.0f * limit) / maximum_raw - limit;
}

uint16_t Cubemars::Encode(float value, float minimum, float maximum, uint8_t bits)
{
    const uint32_t maximum_raw = (1U << bits) - 1U;
    return static_cast<uint16_t>((Clamp(value, minimum, maximum) - minimum) * maximum_raw / (maximum - minimum));
}

bool Cubemars::DecodeFeedback(const uint8_t data[8], uint8_t length, Feedback &decoded)
{
    if (data == nullptr || length != 6) {
        return false;
    }
    decoded.motor_id = data[0];
    decoded.position = static_cast<uint16_t>(data[1] << 8 | data[2]);
    decoded.velocity = static_cast<uint16_t>(data[3] << 4 | (data[4] >> 4));
    decoded.torque = static_cast<uint16_t>((data[4] & 0x0F) << 8 | data[5]);
    return true;
}

void Cubemars::CommandPayload(uint8_t tail, uint8_t data[8])
{
    for (uint8_t index = 0; index < 7; ++index) {
        data[index] = 0xFF;
    }
    data[7] = tail;
}

float Cubemars::WrapDelta(float delta, float angle_max)
{
    if (delta > angle_max) {
        delta -= 2.0f * angle_max;
    } else if (delta < -angle_max) {
        delta += 2.0f * angle_max;
    }
    return delta;
}

int Cubemars::Init(CanfdDevice device, float angle_max, float omega_max, float torque_max)
{
    if (device < CANFD_DEVICE_CAN0 || device >= CANFD_DEVICE_COUNT || !__builtin_isfinite(angle_max) || angle_max <= 0.0f || !__builtin_isfinite(omega_max) || omega_max <= 0.0f || !__builtin_isfinite(torque_max) || torque_max <= 0.0f) {
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

int Cubemars::SetMit(float position, float velocity, float kp, float kd, float torque)
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

void Cubemars::PackMit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max)
{
    if (out == nullptr || position_max <= 0.0f || velocity_max <= 0.0f ||
        kp_max <= 0.0f || kd_max <= 0.0f || torque_max <= 0.0f) {
        return;
    }
    const uint16_t p = Encode(position, -position_max, position_max, 16);
    const uint16_t v = Encode(velocity, -velocity_max, velocity_max, 12);
    const uint16_t k_p = Encode(kp, 0.0f, kp_max, 12);
    const uint16_t k_d = Encode(kd, 0.0f, kd_max, 12);
    const uint16_t t = Encode(torque, -torque_max, torque_max, 12);

    out[0] = static_cast<uint8_t>(p >> 8);
    out[1] = static_cast<uint8_t>(p);
    out[2] = static_cast<uint8_t>(v >> 4);
    out[3] = static_cast<uint8_t>((v & 0x0F) << 4 | (k_p >> 8));
    out[4] = static_cast<uint8_t>(k_p);
    out[5] = static_cast<uint8_t>(k_d >> 4);
    out[6] = static_cast<uint8_t>((k_d & 0x0F) << 4 | (t >> 8));
    out[7] = static_cast<uint8_t>(t);
}

int Cubemars::BuildControlFrame(CanfdFrame &frame) const
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
    frame.id = GetControlFrameId();
    frame.id_type = CANFD_ID_STANDARD;
    frame.protocol = CANFD_PROTOCOL_CLASSIC;
    frame.bitrate_switch = CANFD_BRS_DISABLED;
    frame.length = 8;
    PackMit(position, velocity, kp, kd, torque, frame.data,
            position_max, velocity_max, kp_max_, kd_max_, torque_max);
    return 0;
}

int Cubemars::BuildCommandFrame(uint8_t tail, CanfdFrame &frame) const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool ready = initialized_;
    k_spin_unlock(&lock_, key);
    if (!ready) {
        return -ENODEV;
    }

    frame = {};
    frame.id = GetControlFrameId();
    frame.id_type = CANFD_ID_STANDARD;
    frame.protocol = CANFD_PROTOCOL_CLASSIC;
    frame.bitrate_switch = CANFD_BRS_DISABLED;
    frame.length = 8;
    CommandPayload(tail, frame.data);
    return 0;
}

int Cubemars::BuildEnableFrame(CanfdFrame &frame) const
{
    return BuildCommandFrame(0xFC, frame);
}

int Cubemars::BuildDisableFrame(CanfdFrame &frame) const
{
    return BuildCommandFrame(0xFD, frame);
}

int Cubemars::BuildSaveZeroFrame(CanfdFrame &frame) const
{
    return BuildCommandFrame(0xFE, frame);
}

int Cubemars::ProcessFeedback(const CanfdFrame &frame)
{
    if (frame.id != GetFeedbackId()) {
        return -ENOMSG;
    }

    if (frame.id_type != CANFD_ID_STANDARD || frame.protocol != CANFD_PROTOCOL_CLASSIC ||
        frame.length != 6) {
        return -EBADMSG;
    }

    Feedback raw{};
    if (!DecodeFeedback(frame.data, frame.length, raw)) {
        return -EBADMSG;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!initialized_) {
        k_spin_unlock(&lock_, key);
        return -ENODEV;
    }
    const float angle = Decode(raw.position, angle_max_, 16);
    data_.now_angle = angle;
    data_.now_omega = Decode(raw.velocity, omega_max_, 12);
    data_.now_torque = Decode(raw.torque, torque_max_, 12);
    if (!raw_angle_initialized_) {
        data_.now_total_angle = angle;
        raw_angle_initialized_ = true;
    } else {
        data_.now_total_angle += WrapDelta(angle - last_raw_angle_, angle_max_);
    }
    last_raw_angle_ = angle;
    k_spin_unlock(&lock_, key);
    atomic_inc(&feedback_count_);
    return 0;
}

CubemarsData Cubemars::GetData() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const CubemarsData snapshot = data_;
    k_spin_unlock(&lock_, key);
    return snapshot;
}

uint32_t Cubemars::GetFeedbackCount() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}
