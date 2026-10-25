#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

struct AbsoluteEncoderData {
	uint16_t raw_position = 0;
	uint16_t position = 0;
	uint8_t status = 0;
	int64_t multi_turn_position = 0;
	float angle_rad = 0.0f;
	bool valid = false;
	bool online = false;
	uint32_t timestamp_ms = 0;
};

class Encoder
{
public:
	static constexpr uint8_t default_address = 0x01;
	static constexpr uint16_t resolution = 16384;
	static constexpr size_t request_size = 4;
	static constexpr size_t response_size = 6;

	int Init(uint8_t address = default_address,
		 float gear_ratio = 1.0f,
		 uint16_t max_delta = 1000);

	void BuildPoll(uint8_t data[request_size]) const;
	int ProcessFeedback(const uint8_t *data, size_t length);
	void Invalidate();

	AbsoluteEncoderData GetData() const;
	uint32_t GetFeedbackCount() const;
	uint32_t GetErrorCount() const;

	uint8_t Address() const { return _address; }

	bool Initialized() const { return _initialized; }

private:
	struct Feedback {
		uint16_t raw;
		uint16_t position;
		uint8_t status;
	};

	static constexpr float two_pi = 6.28318530717958647692f;
	static constexpr uint8_t function = 0x00;

	static uint16_t Crc16(const uint8_t *data, size_t length);
	static int DecodeFeedback(const uint8_t *data, size_t length,
				  uint8_t expected_address, Feedback &decoded);

	uint8_t _address = default_address;
	float _gear_ratio = 1.0f;
	uint16_t _max_delta = 1000;
	bool _initialized = false;
	bool _position_initialized = false;
	uint16_t _previous_position = 0;
	int64_t _multi_turn_position = 0;
	AbsoluteEncoderData _data{};
	mutable struct k_spinlock _lock {};
	atomic_t _feedback_count{};
	atomic_t _error_count{};
};
