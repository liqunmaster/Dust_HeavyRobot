#include "eso.hpp"

namespace alg
{
namespace
{

float FiniteOr(float value, float fallback)
{
	return std::isfinite(value) ? value : fallback;
}
}

float ESO::Fal(float error, float alpha, float delta)
{
	const float magnitude = std::fabs(error);

	if (magnitude <= delta) {
		return error / std::pow(delta, 1.0f - alpha);
	}

	return std::copysign(std::pow(magnitude, alpha), error);
}

void ESO::Configure(const ESOConfig &config)
{
	_config = config;
	_config.beta1 = std::max(0.0f, FiniteOr(_config.beta1, 0.0f));
	_config.beta2 = std::max(0.0f, FiniteOr(_config.beta2, 0.0f));
	_config.beta3 = std::max(0.0f, FiniteOr(_config.beta3, 0.0f));
	_config.b0 = FiniteOr(_config.b0, 1.0f);

	if (std::fabs(_config.b0) < 1.0e-6f) {
		_config.b0 = 1.0f;
	}

	_config.alpha1 = std::clamp(FiniteOr(_config.alpha1, 0.5f), 0.0f, 1.0f);
	_config.alpha2 = std::clamp(FiniteOr(_config.alpha2, 0.25f), 0.0f, 1.0f);
	_config.delta = std::max(1.0e-6f, FiniteOr(_config.delta, 0.01f));
	Reset();
}

void ESO::Reset(float state)
{
	_z1 = FiniteOr(state, 0.0f);
	_z2 = 0.0f;
	_z3 = 0.0f;
}

void ESO::Update(float measurement, float input, float dt)
{
	if (!std::isfinite(measurement) || !std::isfinite(input) || !std::isfinite(dt) || dt <= 0.0f) {
		return;
	}

	const float error = _z1 - measurement;
	const float feedback_1 = _config.nonlinear ? Fal(error, _config.alpha1, _config.delta) : error;
	const float feedback_2 = _config.nonlinear ? Fal(error, _config.alpha2, _config.delta) : error;
	const float dz1 = _z2 - _config.beta1 * error;
	const float dz2 = _z3 - _config.beta2 * feedback_1 + _config.b0 * input;
	const float dz3 = -_config.beta3 * feedback_2;

	_z1 += dt * dz1;
	_z2 += dt * dz2;
	_z3 += dt * dz3;

	if (!std::isfinite(_z1) || !std::isfinite(_z2) || !std::isfinite(_z3)) {
		Reset(measurement);
	}
}
}
