#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "dvc_canfd.hpp"

enum DmControlMode {
	MOTOR_DM_CONTROL_METHOD_NORMAL_MIT = 1,
	MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA = 2,
	MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA = 3,
	MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT = 4,
};

struct DmMitLimits {
	float position;
	float velocity;
	float torque;
};

struct DmRxData {
	uint8_t motor_id;
	uint8_t status;
	uint16_t position;
	uint16_t velocity;
	uint16_t torque;
	uint8_t mos_temperature;
	uint8_t rotor_temperature;
};

struct DmData {
	float now_angle;
	float now_omega;
	float now_torque;
	float now_mos_temperature;
	float now_rotor_temperature;
	uint8_t status;
};

class DmMotor
{
public:
	int Init(CanfdDevice device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits,
		 CanfdProtocol protocol, uint16_t master_id = 0, uint16_t tx_id_base = 0);

	int SetMit(float position, float velocity, float kp, float kd, float torque);

	int SetPositionVelocity(float position, float max_velocity);

	int SetVelocity(float velocity);

	int SetPositionTorque(float position, float max_velocity, float current_ratio);

	int BuildControlFrame(CanfdFrame &frame) const;

	int BuildEnableFrame(CanfdFrame &frame) const;

	int BuildDisableFrame(CanfdFrame &frame) const;

	int BuildClearErrorFrame(CanfdFrame &frame) const;

	int BuildSaveZeroFrame(CanfdFrame &frame) const;

	int ProcessFeedback(const CanfdFrame &frame);

	uint32_t GetFeedbackCount() const;

	DmRxData GetRxData() const;

	DmData GetData() const;

	DmControlMode GetMode() const;

	CanfdDevice GetDevice() const { return _device; }

	uint8_t GetMotorId() const { return _motor_id; }

	uint32_t GetFeedbackId() const { return _master_id; }

	CanfdProtocol GetFeedbackProtocol() const { return _protocol; }

	uint32_t GetControlFrameId() const;

private:
	friend int canfd_port_request_mode(DmMotor &motor, DmControlMode mode);
	friend class MotorPort;

	int BuildModeFrame(DmControlMode mode, CanfdFrame &frame) const;

	int MarkModeRequest(DmControlMode mode);

	static bool ValidMode(DmControlMode mode);

	static uint16_t ControlId(uint16_t tx_id_base, DmControlMode mode);

	static uint16_t Encode(float value, float minimum, float maximum, uint16_t maximum_raw);

	static float Decode(uint16_t raw, float limit, uint16_t maximum_raw);

	static void WriteF32(uint8_t *dst, float value);

	static void WriteU16(uint8_t *dst, uint16_t value);

	static void PackMit(float position, float velocity, float kp, float kd, float torque,
			    uint8_t data[8], const DmMitLimits &limits);

	static void CommandPayload(uint8_t command, uint8_t data[8]);

	static bool DecodeFeedback(const uint8_t data[8], uint8_t motor_id, DmRxData &feedback);

	int BuildCommandFrame(uint8_t command, CanfdFrame &frame) const;

	static int MakeFrame(uint16_t id, const uint8_t *data, uint8_t length, CanfdFrame &frame);

	CanfdDevice _device = CANFD_DEVICE_COUNT;

	uint8_t _motor_id = 0;

	uint16_t _master_id = 0;

	uint16_t _tx_id_base = 0;

	DmControlMode _mode = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

	CanfdProtocol _protocol = CANFD_PROTOCOL_CLASSIC;

	DmControlMode _requested_mode = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

	bool _mode_change_pending = false;

	bool _initialized = false;

	DmMitLimits _limits{};

	uint8_t _tx_data[8] {};

	uint8_t _tx_length = 0;

	DmRxData _rx_data{};

	DmData _data{};

	mutable struct k_spinlock _lock {};

	atomic_t _feedback_count{};
};
