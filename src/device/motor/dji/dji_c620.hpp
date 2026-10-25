#pragma once

#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "dvc_canfd.hpp"
#include "dji_motor.hpp"
#include "type_math.hpp"

enum C620Id {
	C620_ID_0x201 = 1,
	C620_ID_0x202 = 2,
	C620_ID_0x203 = 3,
	C620_ID_0x204 = 4,
	C620_ID_0x205 = 5,
	C620_ID_0x206 = 6,
	C620_ID_0x207 = 7,
	C620_ID_0x208 = 8,
};

struct C620RxData {
	uint16_t encoder;
	int16_t omega;
	uint16_t current;
	uint8_t temperature;
	uint8_t error;
} __attribute__((packed));

struct C620Data {
	float now_angle;
	float now_omega;
	float now_current;
	float now_temperature;
	uint32_t pre_encoder;
	int32_t total_encoder;
	int32_t total_round;
};

class C620
{
public:
	int Init(CanfdDevice device, C620Id id, float gear_ratio = 1.0f);

	int SetCurrent(float current);

	int BuildControlFrame(CanfdFrame &frame) const;

	int ProcessFeedback(const CanfdFrame &frame);

	uint32_t GetFeedbackCount() const;

	C620RxData GetRxData() const;

	C620Data GetData() const;

	CanfdDevice GetDevice() const
	{
		return _device;
	}

	uint8_t GetMotorId() const { return static_cast<uint8_t>(_id); }

	uint32_t GetFeedbackId() const
	{
		return 0x200 + GetMotorId();
	}

	uint32_t GetCommandFrameId() const
	{
		return GetMotorId() <= 4 ? 0x200 : 0x1FF;
	}

private:
	static constexpr uint16_t _encoder_resolution = 8192;

	static constexpr int16_t _current_raw_limit = 16384;

	static constexpr float _current_limit = 20.0f;

	static C620RxData DecodeFeedback(const uint8_t *data);

	void UnpackFeedback(const uint8_t *data);

	CanfdDevice _device = CANFD_DEVICE_COUNT;

	C620Id _id = C620_ID_0x201;

	float _gear_ratio = 1.0f;

	bool _initialized = false;

	C620RxData _rx_data{};

	C620Data _data{};

	int64_t _total_encoder = 0;

	bool _encoder_initialized = false;

	mutable struct k_spinlock _feedback_lock {};

	atomic_t _feedback_count{};
};
