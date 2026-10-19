#include "encoder.hpp"

#include <cmath>

uint16_t Encoder::Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U) : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

int Encoder::DecodeFeedback(const uint8_t *data, size_t length, uint8_t expected_address, Feedback &decoded)
{
    if (data == nullptr) {
        return -EINVAL;
    }
    if (length != response_size) {
        return -EMSGSIZE;
    }
    if (data[0] != expected_address || data[1] != function) {
        return -ENOMSG;
    }
    const uint16_t expected_crc = Crc16(data, length - 2);
    const uint16_t received_crc = static_cast<uint16_t>(data[length - 2]) | static_cast<uint16_t>(data[length - 1] << 8);
    if (expected_crc != received_crc) {
        return -EBADMSG;
    }
    decoded.raw = static_cast<uint16_t>(data[2] << 8 | data[3]);
    decoded.position = static_cast<uint16_t>(decoded.raw & 0x3FFFU);
    decoded.status = static_cast<uint8_t>(decoded.raw >> 14);
    return 0;
}

int Encoder::Init(uint8_t address, float gear_ratio, uint16_t max_delta)
{
    if (address == 0 || !std::isfinite(gear_ratio) || gear_ratio <= 0.0f || max_delta == 0 ||
        max_delta >= resolution / 2U) {
        return -EINVAL;
    }
    address_ = address;
    gear_ratio_ = gear_ratio;
    max_delta_ = max_delta;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    position_initialized_ = false;
    previous_position_ = 0;
    multi_turn_position_ = 0;
    data_ = {};
    k_spin_unlock(&lock_, key);
    atomic_set(&feedback_count_, 0);
    atomic_set(&error_count_, 0);
    initialized_ = true;
    return 0;
}

void Encoder::BuildPoll(uint8_t data[request_size]) const
{
    if (data == nullptr) {
        return;
    }
    data[0] = address_;
    data[1] = function;
    const uint16_t crc = Crc16(data, 2);
    data[2] = static_cast<uint8_t>(crc);
    data[3] = static_cast<uint8_t>(crc >> 8);
}

int Encoder::ProcessFeedback(const uint8_t *data, size_t length)
{
    if (!initialized_) {
        return -ENODEV;
    }
    Feedback decoded{};
    int ret = DecodeFeedback(data, length, address_, decoded);
    if (ret != 0) {
        Invalidate();
        atomic_inc(&error_count_);
        return ret;
    }
    atomic_inc(&feedback_count_);
    {
        const k_spinlock_key_t key = k_spin_lock(&lock_);
        data_.raw_position = decoded.raw;
        data_.position = decoded.position;
        data_.status = decoded.status;
        data_.online = true;
        data_.timestamp_ms = k_uptime_get_32();
        k_spin_unlock(&lock_, key);
    }
    if (decoded.status != 0) {
        const k_spinlock_key_t key = k_spin_lock(&lock_);
        data_.status = decoded.status;
        data_.valid = false;
        position_initialized_ = false;
        k_spin_unlock(&lock_, key);
        atomic_inc(&error_count_);
        return -EIO;
    }

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (position_initialized_) {
        int32_t delta = static_cast<int32_t>(decoded.position) - static_cast<int32_t>(previous_position_);
        if (delta > static_cast<int32_t>(resolution / 2U)) {
            delta -= resolution;
        } else if (delta < -static_cast<int32_t>(resolution / 2U)) {
            delta += resolution;
        }
        if (std::abs(delta) > static_cast<int32_t>(max_delta_)) {
            data_.status = 0;
            data_.valid = false;

            position_initialized_ = false;
            k_spin_unlock(&lock_, key);
            atomic_inc(&error_count_);
            return -ERANGE;
        }
        multi_turn_position_ += delta;
    } else {
        multi_turn_position_ = decoded.position;
        position_initialized_ = true;
    }
    previous_position_ = decoded.position;
    data_.raw_position = decoded.raw;
    data_.position = decoded.position;
    data_.status = decoded.status;
    data_.multi_turn_position = multi_turn_position_;
    data_.angle_rad = static_cast<float>(multi_turn_position_) * two_pi / (static_cast<float>(resolution) * gear_ratio_);
    data_.valid = true;
    k_spin_unlock(&lock_, key);
    return 0;
}

void Encoder::Invalidate()
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    data_.online = false;
    data_.valid = false;
    position_initialized_ = false;
    k_spin_unlock(&lock_, key);
}

AbsoluteEncoderData Encoder::GetData() const
{
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const AbsoluteEncoderData copy = data_;
    k_spin_unlock(&lock_, key);
    return copy;
}

uint32_t Encoder::GetFeedbackCount() const
{
    return static_cast<uint32_t>(atomic_get(&feedback_count_));
}

uint32_t Encoder::GetErrorCount() const
{
    return static_cast<uint32_t>(atomic_get(&error_count_));
}
