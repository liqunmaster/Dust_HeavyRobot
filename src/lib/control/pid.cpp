#include "pid.hpp"

namespace alg
{
namespace
{

float PositiveOr(float value, float fallback)
{
	return std::isfinite(value) && value > 0.0f ? value : fallback;
}
}

float Pid::ClampAbs(float value, float limit)
{
	if (std::isfinite(limit) && limit > 0.0f) {
		value = std::clamp(value, -limit, limit);
	}

	return value;
}

float Pid::WrapAngle(float value)
{
	return std::remainder(value, 2.0f * 3.14159265358979323846f);
}

void Pid::Configure(const PIDConfig &config)
{
	_config = config;
	_config.dt = PositiveOr(_config.dt, 0.001f);

	if (!std::isfinite(_config.derivative_filter_tau) || _config.derivative_filter_tau < 0.0f) {
		_config.derivative_filter_tau = 0.0f;
	}

	if (!std::isfinite(_config.dead_zone) || _config.dead_zone < 0.0f) {
		_config.dead_zone = 0.0f;
	}

	if (!std::isfinite(_config.variable_speed_a) || _config.variable_speed_a < 0.0f) {
		_config.variable_speed_a = 0.0f;
	}

	if (!std::isfinite(_config.variable_speed_b) || _config.variable_speed_b < _config.variable_speed_a) {
		_config.variable_speed_b = 0.0f;
	}

	if (!std::isfinite(_config.integral_separation) || _config.integral_separation < 0.0f) {
		_config.integral_separation = 0.0f;
	}

	Reset();
}

void Pid::Reset()
{
	_target = 0.0f;
	_measurement = 0.0f;
	_previous_target = 0.0f;
	_previous_measurement = 0.0f;
	_previous_error = 0.0f;
	_integral_error = 0.0f;
	_derivative_output = 0.0f;
	_output = 0.0f;
	_initialized = false;
}

float Pid::Calculate(float target, float measurement, float dt, bool angle)
{
	if (!std::isfinite(target) || !std::isfinite(measurement)) {
		return _output;
	}

	dt = PositiveOr(dt, _config.dt);
	_target = target;
	_measurement = measurement;

	float error = angle ? WrapAngle(target - measurement) : target - measurement;
	const float absolute_error = std::fabs(error);

	if (_config.dead_zone > 0.0f) {
		if (absolute_error <= _config.dead_zone) {
			error = 0.0f;

		} else {
			error -= std::copysign(_config.dead_zone, error);
		}
	}

	const float p_output = _config.kp * error;
	float integral_rate = 1.0f;

	if (_config.variable_speed_b > _config.variable_speed_a && absolute_error > _config.variable_speed_a) {
		if (absolute_error >= _config.variable_speed_b) {
			integral_rate = 0.0f;

		} else {
			integral_rate = (_config.variable_speed_b - absolute_error) /
					(_config.variable_speed_b - _config.variable_speed_a);
		}
	}

	if (_config.integral_separation > 0.0f && absolute_error >= _config.integral_separation) {
		integral_rate = 0.0f;
	}

	_integral_error += integral_rate * error * dt;

	if (_config.ki != 0.0f && _config.integral_limit > 0.0f) {
		_integral_error = ClampAbs(_integral_error, _config.integral_limit / std::fabs(_config.ki));
	}

	const float i_output = _config.ki * _integral_error;

	float d_raw = 0.0f;

	if (_initialized && _config.kd != 0.0f) {
		if (_config.derivative_on_measurement == DFirst::Enable) {
			d_raw = -_config.kd * (measurement - _previous_measurement) / dt;

		} else {
			const float derivative_error = angle ? WrapAngle(error - _previous_error) : error - _previous_error;
			d_raw = _config.kd * derivative_error / dt;
		}
	}

	if (_config.derivative_filter_tau > 0.0f) {
		const float alpha = _config.derivative_filter_tau /
				    (_config.derivative_filter_tau + dt);
		_derivative_output = alpha * _derivative_output + (1.0f - alpha) * d_raw;

	} else {
		_derivative_output = d_raw;
	}

	const float feedforward = _initialized && _config.kf != 0.0f ? _config.kf * (target - _previous_target) / dt : 0.0f;
	const float unclamped_output = p_output + i_output + _derivative_output + feedforward;
	_output = ClampAbs(unclamped_output, _config.output_limit);

	if (_output != unclamped_output && _config.ki != 0.0f && integral_rate > 0.0f) {
		const float without_integral = p_output + _derivative_output + feedforward;
		const float required_integral = _output - without_integral;
		_integral_error = ClampAbs(required_integral / _config.ki,
					   _config.integral_limit > 0.0f ? _config.integral_limit / std::fabs(_config.ki) :
					   std::numeric_limits<float>::infinity());
	}

	_previous_target = target;
	_previous_measurement = measurement;
	_previous_error = error;
	_initialized = true;
	return _output;
}

float Pid::Update(float target, float measurement)
{
	return Calculate(target, measurement, _config.dt, false);
}

float Pid::Update(float target, float measurement, float dt)
{
	return Calculate(target, measurement, dt, false);
}

float Pid::UpdateAngle(float target, float measurement)
{
	return Calculate(target, measurement, _config.dt, true);
}

float Pid::UpdateAngle(float target, float measurement, float dt)
{
	return Calculate(target, measurement, dt, true);
}

void Pid::Init(float kp, float ki, float kd, float kf, float integral_limit, float output_limit,
	       float dt, float dead_zone, float variable_speed_a, float variable_speed_b,
	       float integral_separation, DFirst derivative_on_measurement, float derivative_filter_tau)
{
	Configure(PIDConfig{kp, ki, kd, kf, integral_limit, output_limit, dt, dead_zone,
			    variable_speed_a, variable_speed_b, integral_separation, derivative_on_measurement,
			    derivative_filter_tau});
}

void Pid::SetKp(float kp) { _config.kp = kp; }

void Pid::SetKi(float ki) { _config.ki = ki; }

void Pid::SetKd(float kd) { _config.kd = kd; }

void Pid::SetKf(float kf) { _config.kf = kf; }

void Pid::SetIOutMax(float limit) { _config.integral_limit = std::fabs(limit); }

void Pid::SetOutMax(float limit) { _config.output_limit = std::fabs(limit); }

void Pid::SetIVariableSpeedA(float value) { _config.variable_speed_a = std::max(0.0f, value); }

void Pid::SetIVariableSpeedB(float value) { _config.variable_speed_b = std::max(0.0f, value); }

void Pid::SetISeparateThreshold(float value) { _config.integral_separation = std::max(0.0f, value); }

void Pid::SetTarget(float target) { _target = target; }

void Pid::SetNow(float measurement) { _measurement = measurement; }

void Pid::SetIntegralError(float integral_error) { _integral_error = integral_error; }

void Pid::CalculatePeriodElapsedCallback()
{
	Update(_target, _measurement);
}

void Pid::CalculateAnglePid()
{
	UpdateAngle(_target, _measurement);
}
}
