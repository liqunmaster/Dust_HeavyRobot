#include "dm_motor.hpp"

namespace
{
constexpr uint16_t RegisterCommandId = 0x7FF;
constexpr uint8_t ModeRegister = 0x0A;
constexpr uint16_t TwelveBitMax = 0x0FFF;
constexpr uint16_t SixteenBitMax = 0xFFFF;

bool Finite(float value)
{
	return __builtin_isfinite(value);
}
}

bool DmMotor::ValidMode(DmControlMode mode)
{
	return mode >= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT
	       && mode <= DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT;
}

uint16_t DmMotor::ControlId(uint16_t tx_id_base, DmControlMode mode)
{
	return tx_id_base + ((static_cast<uint16_t>(mode) - 1U) << 8);
}

uint16_t DmMotor::Encode(float value, float minimum, float maximum, uint16_t maximum_raw)
{
	return static_cast<uint16_t>((value - minimum) * maximum_raw / (maximum - minimum));
}

float DmMotor::Decode(uint16_t raw, float limit, uint16_t maximum_raw)
{
	return static_cast<float>(raw) * (2.0f * limit) / maximum_raw - limit;
}

void DmMotor::WriteF32(uint8_t *dst, float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));

	for (uint8_t index = 0; index < 4; ++index) {
		dst[index] = static_cast<uint8_t>(bits >>(8 * index));
	}
}

void DmMotor::WriteU16(uint8_t *dst, uint16_t value)
{
	dst[0] = static_cast<uint8_t>(value);
	dst[1] = static_cast<uint8_t>(value >> 8);
}

void DmMotor::PackMit(float position, float velocity, float kp, float kd, float torque,
		      uint8_t data[8], const DmMitLimits &limits)
{
	const uint16_t p = Encode(position, -limits.position, limits.position, SixteenBitMax);
	const uint16_t v = Encode(velocity, -limits.velocity, limits.velocity, TwelveBitMax);
	const uint16_t k_p = Encode(kp, 0.0f, 500.0f, TwelveBitMax);
	const uint16_t k_d = Encode(kd, 0.0f, 5.0f, TwelveBitMax);
	const uint16_t t = Encode(torque, -limits.torque, limits.torque, TwelveBitMax);
	data[0] = static_cast<uint8_t>(p >> 8);
	data[1] = static_cast<uint8_t>(p);
	data[2] = static_cast<uint8_t>(v >> 4);
	data[3] = static_cast<uint8_t>((v & 0x0F) << 4 | (k_p >> 8));
	data[4] = static_cast<uint8_t>(k_p);
	data[5] = static_cast<uint8_t>(k_d >> 4);
	data[6] = static_cast<uint8_t>((k_d & 0x0F) << 4 | (t >> 8));
	data[7] = static_cast<uint8_t>(t);
}

void DmMotor::CommandPayload(uint8_t command, uint8_t data[8])
{
	for (uint8_t index = 0; index < 8; ++index) {
		data[index] = 0xFF;
	}

	data[7] = command;
}

bool DmMotor::DecodeFeedback(const uint8_t data[8], uint8_t motor_id, DmRxData &feedback)
{
	if (data == nullptr) {
		return false;
	}

	const uint64_t raw = (static_cast<uint64_t>(data[0]) << 56) |
			     (static_cast<uint64_t>(data[1]) << 48) |
			     (static_cast<uint64_t>(data[2]) << 40) |
			     (static_cast<uint64_t>(data[3]) << 32) |
			     (static_cast<uint64_t>(data[4]) << 24) |
			     (static_cast<uint64_t>(data[5]) << 16) |
			     (static_cast<uint64_t>(data[6]) << 8) |
			     static_cast<uint64_t>(data[7]);
	const uint8_t id = static_cast<uint8_t>((raw >> 56) & 0x0F);

	if (id != motor_id) {
		return false;
	}

	feedback.motor_id = id;
	feedback.status = static_cast<uint8_t>((raw >> 60) & 0x0F);
	feedback.position = static_cast<uint16_t>((raw >> 40) & 0xFFFF);
	feedback.velocity = static_cast<uint16_t>((raw >> 28) & 0x0FFF);
	feedback.torque = static_cast<uint16_t>((raw >> 16) & 0x0FFF);
	feedback.mos_temperature = static_cast<uint8_t>((raw >> 8) & 0xFF);
	feedback.rotor_temperature = static_cast<uint8_t>(raw & 0xFF);
	return true;
}

int DmMotor::Init(CanfdDevice device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits, CanfdProtocol protocol,
		  uint16_t master_id, uint16_t tx_id_base)
{
	if (device < CANFD_DEVICE_CAN0 || device >= CANFD_DEVICE_COUNT || motor_id == 0 || motor_id > 15 || master_id > 0x7FF
	    || tx_id_base > 0x4FF || (protocol != CANFD_PROTOCOL_CLASSIC && protocol != CANFD_PROTOCOL_FD) ||
	    !ValidMode(mode) || !Finite(limits.position) || limits.position <= 0.0f || !Finite(limits.velocity)
	    || limits.velocity <= 0.0f || !Finite(limits.torque) || limits.torque <= 0.0f) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);
	_device = device;
	_motor_id = motor_id;
	_master_id = master_id;
	_tx_id_base = tx_id_base == 0 ? motor_id : tx_id_base;
	_mode = mode;
	_protocol = protocol;
	_requested_mode = mode;
	_mode_change_pending = false;
	_limits = limits;
	memset(_tx_data, 0, sizeof(_tx_data));
	_tx_length = 0;
	_rx_data = {};
	_data = {};
	_initialized = true;
	k_spin_unlock(&_lock, key);
	atomic_set(&_feedback_count, 0);
	return 0;
}

int DmMotor::SetMit(float position, float velocity, float kp, float kd, float torque)
{
	if (!Finite(position) || !Finite(velocity) || !Finite(kp) || !Finite(kd) || !Finite(torque)) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || _mode != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT || _mode_change_pending) {
		k_spin_unlock(&_lock, key);
		return !_initialized ? -ENODEV : -EPERM;
	}

	if (position < -_limits.position || position > _limits.position || velocity < -_limits.velocity
	    || velocity > _limits.velocity || torque < -_limits.torque || torque > _limits.torque || kp < 0.0f || kp > 500.0f
	    || kd < 0.0f || kd > 5.0f) {
		k_spin_unlock(&_lock, key);
		return -ERANGE;
	}

	PackMit(position, velocity, kp, kd, torque, _tx_data, _limits);
	_tx_length = 8;
	k_spin_unlock(&_lock, key);
	return 0;
}

int DmMotor::SetPositionVelocity(float position, float max_velocity)
{
	if (!Finite(position) || !Finite(max_velocity) || max_velocity < 0.0f) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || _mode != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA || _mode_change_pending) {
		k_spin_unlock(&_lock, key);
		return !_initialized ? -ENODEV : -EPERM;
	}

	WriteF32(&_tx_data[0], position);
	WriteF32(&_tx_data[4], max_velocity);
	_tx_length = 8;
	k_spin_unlock(&_lock, key);
	return 0;
}

int DmMotor::SetVelocity(float velocity)
{
	if (!Finite(velocity)) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || _mode != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA || _mode_change_pending) {
		k_spin_unlock(&_lock, key);
		return !_initialized ? -ENODEV : -EPERM;
	}

	WriteF32(&_tx_data[0], velocity);
	memset(&_tx_data[4], 0, 4);
	_tx_length = 4;
	k_spin_unlock(&_lock, key);
	return 0;
}

int DmMotor::SetPositionTorque(float position, float max_velocity, float current_ratio)
{
	if (!Finite(position) || !Finite(max_velocity) || !Finite(current_ratio) || max_velocity < 0.0f
	    || max_velocity > 100.0f || current_ratio < 0.0f || current_ratio > 1.0f) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || _mode != DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT || _mode_change_pending) {
		k_spin_unlock(&_lock, key);
		return !_initialized ? -ENODEV : -EPERM;
	}

	WriteF32(&_tx_data[0], position);
	WriteU16(&_tx_data[4], static_cast<uint16_t>(max_velocity * 100.0f));
	WriteU16(&_tx_data[6], static_cast<uint16_t>(current_ratio * 10000.0f));
	_tx_length = 8;
	k_spin_unlock(&_lock, key);
	return 0;
}

int DmMotor::MakeFrame(uint16_t id, const uint8_t *data, uint8_t length, CanfdFrame &frame)
{
	if (data == nullptr || length > 8) {
		return -EINVAL;
	}

	frame = {};
	frame.id = id;
	frame.id_type = CANFD_ID_STANDARD;
	frame.protocol = CANFD_PROTOCOL_CLASSIC;
	frame.bitrate_switch = CANFD_BRS_DISABLED;
	frame.length = length;
	memcpy(frame.data, data, length);
	return 0;
}

int DmMotor::BuildControlFrame(CanfdFrame &frame) const
{
	uint8_t payload[8];
	uint8_t length;
	uint16_t id;
	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || _mode_change_pending || _tx_length == 0) {
		const int result = !_initialized ? -ENODEV : _mode_change_pending ? -EBUSY
				   : -ENODATA;
		k_spin_unlock(&_lock, key);
		return result;
	}

	id = ControlId(_tx_id_base, _mode);
	length = _tx_length;
	memcpy(payload, _tx_data, length);
	k_spin_unlock(&_lock, key);

	return MakeFrame(id, payload, length, frame);
}

int DmMotor::BuildCommandFrame(uint8_t command, CanfdFrame &frame) const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized) {
		k_spin_unlock(&_lock, key);
		return -ENODEV;
	}

	const uint16_t id = ControlId(_tx_id_base, _mode);
	k_spin_unlock(&_lock, key);
	uint8_t payload[8];
	CommandPayload(command, payload);

	return MakeFrame(id, payload, sizeof(payload), frame);
}

int DmMotor::BuildEnableFrame(CanfdFrame &frame) const
{

	return BuildCommandFrame(0xFC, frame);
}

int DmMotor::BuildDisableFrame(CanfdFrame &frame) const
{

	return BuildCommandFrame(0xFD, frame);
}

int DmMotor::BuildClearErrorFrame(CanfdFrame &frame) const
{

	return BuildCommandFrame(0xFB, frame);
}

int DmMotor::BuildSaveZeroFrame(CanfdFrame &frame) const
{

	return BuildCommandFrame(0xFE, frame);
}

int DmMotor::BuildModeFrame(DmControlMode mode, CanfdFrame &frame) const
{
	if (!ValidMode(mode)) {
		return -EINVAL;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || (_mode_change_pending && _requested_mode != mode)) {
		const int result = !_initialized ? -ENODEV : -EBUSY;
		k_spin_unlock(&_lock, key);
		return result;
	}

	if (mode == _mode && !_mode_change_pending) {
		k_spin_unlock(&_lock, key);
		return -EALREADY;
	}

	const uint8_t motor_id = _motor_id;
	k_spin_unlock(&_lock, key);

	const uint8_t payload[8] = {motor_id, 0, 0x55, ModeRegister, static_cast<uint8_t>(mode), 0, 0, 0};

	return MakeFrame(RegisterCommandId, payload, sizeof(payload), frame);
}

int DmMotor::MarkModeRequest(DmControlMode mode)
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);

	if (!_initialized || (_mode_change_pending && _requested_mode != mode)) {
		const int result = !_initialized ? -ENODEV : -EBUSY;
		k_spin_unlock(&_lock, key);
		return result;
	}

	_requested_mode = mode;
	_mode_change_pending = true;
	k_spin_unlock(&_lock, key);
	return 0;
}

int DmMotor::ProcessFeedback(const CanfdFrame &frame)
{
	if (!_initialized) {
		return -ENODEV;
	}

	if (frame.id != _master_id) {
		return -ENOMSG;
	}

	if (frame.id_type != CANFD_ID_STANDARD || frame.protocol != _protocol || frame.length != 8) {
		return -EBADMSG;
	}

	if (frame.data[0] == _motor_id && frame.data[1] == 0 && frame.data[2] == 0x55 && frame.data[3] == ModeRegister) {
		const k_spinlock_key_t key = k_spin_lock(&_lock);

		if (_mode_change_pending && frame.data[4] == static_cast<uint8_t>(_requested_mode) && frame.data[5] == 0
		    && frame.data[6] == 0 && frame.data[7] == 0) {
			_mode = _requested_mode;
			_mode_change_pending = false;
			_tx_length = 0;
			k_spin_unlock(&_lock, key);
			atomic_inc(&_feedback_count);
			return 0;
		}

		k_spin_unlock(&_lock, key);
		return -ENOMSG;
	}

	DmRxData raw{};

	if (!DecodeFeedback(frame.data, _motor_id, raw)) {
		return -ENOMSG;
	}

	const k_spinlock_key_t key = k_spin_lock(&_lock);
	_rx_data = raw;
	_data = {
		Decode(raw.position, _limits.position, SixteenBitMax),
		Decode(raw.velocity, _limits.velocity, TwelveBitMax),
		Decode(raw.torque, _limits.torque, TwelveBitMax),
		static_cast<float>(raw.mos_temperature),
		static_cast<float>(raw.rotor_temperature),
		raw.status,
	};
	k_spin_unlock(&_lock, key);
	atomic_inc(&_feedback_count);
	return 0;
}

uint32_t DmMotor::GetFeedbackCount() const
{
	return static_cast<uint32_t>(atomic_get(&_feedback_count));
}

DmRxData DmMotor::GetRxData() const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const DmRxData snapshot = _rx_data;
	k_spin_unlock(&_lock, key);
	return snapshot;
}

DmData DmMotor::GetData() const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const DmData snapshot = _data;
	k_spin_unlock(&_lock, key);
	return snapshot;
}

DmControlMode DmMotor::GetMode() const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const DmControlMode snapshot = _mode;
	k_spin_unlock(&_lock, key);
	return snapshot;
}

uint32_t DmMotor::GetControlFrameId() const
{
	const k_spinlock_key_t key = k_spin_lock(&_lock);
	const uint32_t id = _initialized ? ControlId(_tx_id_base, _mode) : 0;
	k_spin_unlock(&_lock, key);
	return id;
}
