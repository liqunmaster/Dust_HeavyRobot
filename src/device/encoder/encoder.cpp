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

	_address = address;
	_gear_ratio = gear_ratio;
	_max_delta = max_delta;
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	_position_initialized = false;
	_previous_position = 0;
	_multi_turn_position = 0;
	_data = {};
	k_spin_unlock(&_lock, key);
	atomic_set(&_feedback_count, 0);
	atomic_set(&_error_count, 0);
	_initialized = true;
	return 0;
}

void Encoder::BuildPoll(uint8_t data[request_size]) const
{
	if (data == nullptr) {
		return;
	}

	data[0] = _address;
	data[1] = function;
	const uint16_t crc = Crc16(data, 2);
	data[2] = static_cast<uint8_t>(crc);
	data[3] = static_cast<uint8_t>(crc >> 8);
}

int Encoder::ProcessFeedback(const uint8_t *data, size_t length)
{
	if (!_initialized) {
		return -ENODEV;
	}

	Feedback decoded{};
	int ret = DecodeFeedback(data, length, _address, decoded);

	if (ret != 0) {
		Invalidate();
		atomic_inc(&_error_count);
		return ret;
	}

	atomic_inc(&_feedback_count);
	{
		const k_spinlock_key_t key = k_spin_lock(&_lock);
		_data.raw_position = decoded.raw;
		_data.position = decoded.position;
		_data.status = decoded.status;
		_data.online = true;
		_data.timestamp_ms = k_uptime_get_32();
		k_spin_unlock(&_lock, key);
	}

	if (decoded.status != 0) {
		const k_spinlock_key_t key = k_spin_lock(&_lock);
		_data.status = decoded.status;
		_data.valid = false;
		_position_initialized = false;
		k_spin_unlock(&_lock, key);
		atomic_inc(&_error_count);
		return -EIO;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (_position_initialized) {
		int32_t delta = static_cast<int32_t>(decoded.position) - static_cast<int32_t>(_previous_position);

		if (delta > static_cast<int32_t>(resolution / 2U)) {
			delta -= resolution;

		} else if (delta < -static_cast<int32_t>(resolution / 2U)) {
			delta += resolution;
		}

		if (std::abs(delta) > static_cast<int32_t>(_max_delta)) {
			_data.status = 0;
			_data.valid = false;

			_position_initialized = false;
			k_spin_unlock(&_lock, key);
			atomic_inc(&_error_count);
			return -ERANGE;
		}

		_multi_turn_position += delta;

	} else {
		_multi_turn_position = decoded.position;
		_position_initialized = true;
	}

	_previous_position = decoded.position;
	_data.raw_position = decoded.raw;
	_data.position = decoded.position;
	_data.status = decoded.status;
	_data.multi_turn_position = _multi_turn_position;
	_data.angle_rad = static_cast<float>(_multi_turn_position) * two_pi / (static_cast<float>(resolution) * _gear_ratio);
	_data.valid = true;
	k_spin_unlock(&_lock, key);
	return 0;
}

void Encoder::Invalidate()
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	_data.online = false;
	_data.valid = false;
	_position_initialized = false;
	k_spin_unlock(&_lock, key);
}

AbsoluteEncoderData Encoder::GetData() const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const AbsoluteEncoderData copy = _data;
	k_spin_unlock(&_lock, key);
	return copy;
}

uint32_t Encoder::GetFeedbackCount() const
{
	return static_cast<uint32_t>(atomic_get(&_feedback_count));
}

uint32_t Encoder::GetErrorCount() const
{
	return static_cast<uint32_t>(atomic_get(&_error_count));
}
