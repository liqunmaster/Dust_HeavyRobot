#include "adrc.hpp"

namespace alg
{
namespace
{

float FiniteOr(float value, float fallback)
{
	return std::isfinite(value) ? value : fallback;
}
}

float ADRC::ClampAbs(float value, float limit)
{
	if (std::isfinite(limit) && limit > 0.0f) {
		return std::clamp(value, -limit, limit);
	}

	return value;
}

float ADRC::Fal(float error, float alpha, float delta)
{
	const float magnitude = std::fabs(error);

	if (magnitude <= delta) {
		return error / std::pow(delta, 1.0f - alpha);
	}

	return std::copysign(std::pow(magnitude, alpha), error);
}

void ADRC::Configure(const ADRCConfig &config)
{
	_config = config;
	_config.b0 = FiniteOr(_config.b0, 1.0f);

	if (std::fabs(_config.b0) < 1.0e-6f) {
		_config.b0 = 1.0f;
	}

	_config.tracking_rate = std::max(0.0f, FiniteOr(_config.tracking_rate, 0.0f));
	_config.tracking_acceleration = std::max(0.0f, FiniteOr(_config.tracking_acceleration, 0.0f));
	_config.kp = FiniteOr(_config.kp, 0.0f);
	_config.kd = FiniteOr(_config.kd, 0.0f);

	ESOConfig eso_config;
	eso_config.beta1 = _config.beta1;
	eso_config.beta2 = _config.beta2;
	eso_config.beta3 = _config.beta3;
	eso_config.b0 = _config.b0;
	eso_config.alpha1 = _config.alpha1;
	eso_config.alpha2 = _config.alpha2;
	eso_config.delta = _config.fal_delta;
	eso_config.nonlinear = _config.nonlinear_observer;
	_eso.Configure(eso_config);
	Reset();
}

void ADRC::Reset(float state)
{
	_tracking_state = FiniteOr(state, 0.0f);
	_tracking_derivative = 0.0f;
	_output = 0.0f;
	_initialized = false;
	_eso.Reset(_tracking_state);
}

float ADRC::Update(float reference, float measurement, float dt)
{
	if (!std::isfinite(reference) || !std::isfinite(measurement) || !std::isfinite(dt) || dt <= 0.0f) {
		return _output;
	}

	if (!_initialized) {
		_tracking_state = reference;
		_tracking_derivative = 0.0f;
		_eso.Reset(measurement);
		_initialized = true;

	} else {
		const float tracking_error = _tracking_state - reference;
		const float max_acceleration = _config.tracking_acceleration;
		float acceleration = -_config.tracking_rate * tracking_error - 2.0f * std::sqrt(std::max(0.0f,
				     _config.tracking_rate)) * _tracking_derivative;

		if (max_acceleration > 0.0f) {
			acceleration = ClampAbs(acceleration, max_acceleration);
		}

		_tracking_derivative += acceleration * dt;
		_tracking_state += _tracking_derivative * dt;
	}

	_eso.Update(measurement, _output, dt);
	const float error = _tracking_state - _eso.State();
	const float error_derivative = _tracking_derivative - _eso.StateDerivative();
	const float proportional = _config.nonlinear_observer ? _config.kp * Fal(error, _config.alpha1,
				   std::max(_config.fal_delta, 1.0e-6f)) : _config.kp * error;
	const float derivative = _config.nonlinear_observer ? _config.kd * Fal(error_derivative, _config.alpha2,
				 std::max(_config.fal_delta, 1.0e-6f)) : _config.kd * error_derivative;
	_output = (proportional + derivative - _eso.Disturbance()) / _config.b0;
	_output = ClampAbs(_output, _config.output_limit);
	return _output;
}
}
