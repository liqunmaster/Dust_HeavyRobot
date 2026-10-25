#include "dji_c620.hpp"

C620RxData C620::DecodeFeedback(const uint8_t *data)
{
	const DjiMotor::Feedback frame = DjiMotor::DecodeFeedback(data);
	C620RxData decoded{};
	decoded.encoder = frame.encoder;
	decoded.omega = frame.omega;
	decoded.current = frame.current;
	decoded.temperature = frame.auxiliary_0;
	decoded.error = frame.auxiliary_1;
	return decoded;
}

int C620::Init(CanfdDevice device, C620Id id, float gear_ratio)
{
	if (!DjiMotor::ValidDevice(device) || !DjiMotor::ValidId(id) || !__builtin_isfinite(gear_ratio) || gear_ratio <= 0.0f) {
		return -EINVAL;
	}

	_device = device;
	_id = id;
	_gear_ratio = gear_ratio;
	const k_spinlock_key_t key = k_spin_lock(&_feedback_lock);
	_rx_data = {};
	_data = {};
	_total_encoder = 0;
	_encoder_initialized = false;
	k_spin_unlock(&_feedback_lock, key);
	atomic_set(&_feedback_count, 0);
	_initialized = true;
	return 0;
}

int C620::SetCurrent(float current)
{
	if (!_initialized) {
		return -ENODEV;
	}

	if (!__builtin_isfinite(current)) {
		return -EINVAL;
	}

	const int16_t command = math::current_to_raw(current, _current_limit, _current_raw_limit);
	return DjiMotor::Set(_device, static_cast<uint8_t>(_id), command);
}

int C620::BuildControlFrame(CanfdFrame &frame) const
{
	if (!_initialized) {
		return -ENODEV;
	}

	return DjiMotor::BuildControlFrame(_device, static_cast<uint8_t>(_id), frame);
}

void C620::UnpackFeedback(const uint8_t *data)
{
	const C620RxData next = DecodeFeedback(data);
	const k_spinlock_key_t key = k_spin_lock(&_feedback_lock);

	if (!_encoder_initialized) {
		_total_encoder = next.encoder;
		_encoder_initialized = true;

	} else {
		int32_t delta = static_cast<int32_t>(next.encoder) - static_cast<int32_t>(_data.pre_encoder);

		if (delta > _encoder_resolution / 2) {
			delta -= _encoder_resolution;
			--_data.total_round;

		} else if (delta < -_encoder_resolution / 2) {
			delta += _encoder_resolution;
			++_data.total_round;
		}

		_total_encoder += delta;
	}

	_rx_data = next;
	_data.pre_encoder = next.encoder;
	_data.total_encoder = _total_encoder > INT32_MAX ? INT32_MAX : _total_encoder < INT32_MIN ? INT32_MIN
			      : static_cast<int32_t>(_total_encoder);

	const int16_t current = DjiMotor::SignedValue(next.current);
	_data.now_current = math::raw_to_current(current, _current_raw_limit, _current_limit);
	_data.now_omega = math::rpm_to_radian_per_second(next.omega, _gear_ratio);
	_data.now_angle = math::encoder_to_radian(_total_encoder, _encoder_resolution, _gear_ratio);
	_data.now_temperature = static_cast<float>(next.temperature);
	k_spin_unlock(&_feedback_lock, key);
}

int C620::ProcessFeedback(const CanfdFrame &frame)
{
	if (!_initialized) {
		return -ENODEV;
	}

	const int result = DjiMotor::ValidateFeedback(frame, static_cast<uint8_t>(_id));

	if (result != 0) {
		return result;
	}

	UnpackFeedback(frame.data);
	atomic_inc(&_feedback_count);
	return 0;
}

uint32_t C620::GetFeedbackCount() const
{
	return static_cast<uint32_t>(atomic_get(&_feedback_count));
}

C620RxData C620::GetRxData() const
{
	const k_spinlock_key_t key = k_spin_lock(&_feedback_lock);
	const C620RxData snapshot = _rx_data;
	k_spin_unlock(&_feedback_lock, key);
	return snapshot;
}

C620Data C620::GetData() const
{
	const k_spinlock_key_t key = k_spin_lock(&_feedback_lock);
	const C620Data snapshot = _data;
	k_spin_unlock(&_feedback_lock, key);
	return snapshot;
}
