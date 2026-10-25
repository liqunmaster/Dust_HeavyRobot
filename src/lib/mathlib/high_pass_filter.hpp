#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{

class FirstOrderHpf final
{
public:

	void Init(float cutoff_hz, float sample_rate_hz)
	{
		_enabled = cutoff_hz > 0.0f && sample_rate_hz > 0.0f;

		if (_enabled) {
			const float rc = 1.0f / (math::two_pi * cutoff_hz);
			const float dt = 1.0f / sample_rate_hz;
			_alpha = rc / (rc + dt);
		}

		Reset();
	}

	float Update(float input)
	{
		if (!_enabled) {
			return input;
		}

		if (!_initialized) {
			_previous_input = input;
			_initialized = true;
			return 0.0f;
		}

		_value = _alpha * (_value + input - _previous_input);
		_previous_input = input;
		return _value;
	}

	void Reset()
	{
		_value = 0.0f;
		_previous_input = 0.0f;
		_initialized = false;
	}

	void Reset(float initial_input)
	{
		_value = 0.0f;
		_previous_input = initial_input;
		_initialized = true;
	}

private:
	float _alpha = 1.0f;

	float _value = 0.0f;

	float _previous_input = 0.0f;

	bool _initialized = false;

	bool _enabled = false;
};

class HighPassFilter final
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

	void Reset(float initial_input)
	{
		Reset();
		Update(initial_input);
	}

private:
	static constexpr uint8_t max_order = 10;

	FirstOrderHpf _stages[max_order] {};

	uint8_t _order = 1;
};
}
