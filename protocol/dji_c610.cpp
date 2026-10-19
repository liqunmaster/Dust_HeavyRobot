#include "dji_c610.hpp"

C610RxData C610::DecodeFeedback(const uint8_t *data)
{
    const DjiMotor::Feedback frame = DjiMotor::DecodeFeedback(data);
    C610RxData decoded{};
    decoded.encoder = frame.encoder;
    decoded.omega = frame.omega;
    decoded.current = frame.current;
    decoded.reserved = frame.auxiliary_0;
    decoded.error = frame.auxiliary_1;
    return decoded;
}

int C610::Init(CanfdDevice device, C610Id id, float gear_ratio)
{
    if (!DjiMotor::ValidDevice(device) || !DjiMotor::ValidId(id) || !__builtin_isfinite(gear_ratio) || gear_ratio <= 0.0f) {
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

int C610::SetCurrent(float current)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (!__builtin_isfinite(current)) {
        return -EINVAL;
    }

    const int16_t command = math::current_to_raw(current, current_limit_, current_raw_limit_);
    return DjiMotor::Set(device_, static_cast<uint8_t>(id_), command);
}

int C610::BuildControlFrame(CanfdFrame &frame) const
{
    if (!initialized_) {
        return -ENODEV;
    }
    return DjiMotor::BuildControlFrame(device_, static_cast<uint8_t>(id_), frame);
}

void C610::UnpackFeedback(const uint8_t *data)
{
    const C610RxData next = DecodeFeedback(data);
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

    data_.now_current = math::raw_to_current(DjiMotor::SignedValue(next.current), current_raw_limit_, current_limit_);
    data_.now_omega = math::rpm_to_radian_per_second(next.omega, gear_ratio_);
    data_.now_angle = math::encoder_to_radian(total_encoder_, encoder_resolution_, gear_ratio_);
    k_spin_unlock(&feedback_lock_, key);
}

int C610::ProcessFeedback(const CanfdFrame &frame)
{
    if (!initialized_) {
        return -ENODEV;
    }
    const int result = DjiMotor::ValidateFeedback(frame, static_cast<uint8_t>(id_));
    if (result != 0) {
        return result;
    }

    UnpackFeedback(frame.data);
    atomic_inc(&feedback_count_);
    return 0;
}

uint32_t C610::GetFeedbackCount() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

C610RxData C610::GetRxData() const
{
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    const C610RxData snapshot = rx_data_;
    k_spin_unlock(&feedback_lock_, key);
    return snapshot;
}

C610Data C610::GetData() const
{
    const k_spinlock_key_t key = k_spin_lock(&feedback_lock_);
    const C610Data snapshot = data_;
    k_spin_unlock(&feedback_lock_, key);
    return snapshot;
}
