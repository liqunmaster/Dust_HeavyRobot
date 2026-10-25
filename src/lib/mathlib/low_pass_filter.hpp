#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{

class FirstOrderLpf final
{
public:

	void Init(float cutoff_hz, float sample_rate_hz)
	{
		if (cutoff_hz > 0.0f && sample_rate_hz > 0.0f) {
			const float rc = 1.0f / (math::two_pi * cutoff_hz);
			const float dt = 1.0f / sample_rate_hz;
			_alpha = dt / (rc + dt);

		} else {
			_alpha = 1.0f;
		}

		Reset();
	}

	float Update(float input)
	{
		if (!_initialized) {
			_value = input;
			_initialized = true;

		} else {
			_value += _alpha * (input - _value);
		}

		return _value;
	}

	void Reset()
	{
		_value = 0.0f;
		_initialized = false;
	}

	void Reset(float initial_value)
	{
		_value = initial_value;
		_initialized = true;
	}

private:
	float _alpha = 1.0f;

	float _value = 0.0f;

	bool _initialized = false;
};

class LowPassFilter final
{
public:

	void Init(float cutoff_hz, float sample_rate_hz, uint8_t order = 1)
	{
		_order = order >= 1 && order <= max_order ? order : 1;

		for (uint8_t i = 0; i < _order; ++i) {
			_stages[i].Init(cutoff_hz, sample_rate_hz);
		}
	}

	float Update(float input)
	{
		for (uint8_t i = 0; i < _order; ++i) {
			input = _stages[i].Update(input);
		}

		return input;
	}

	void Reset()
	{
		for (uint8_t i = 0; i < _order; ++i) {
			_stages[i].Reset();
		}
	}

	void Reset(float initial_value)
	{
		for (uint8_t i = 0; i < _order; ++i) {
			_stages[i].Reset(initial_value);
		}
	}

private:
	static constexpr uint8_t max_order = 10;

	FirstOrderLpf _stages[max_order] {};

	uint8_t _order = 1;
};
}
