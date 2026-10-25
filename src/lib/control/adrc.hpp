#pragma once

#include "eso.hpp"

namespace alg
{

struct ADRCConfig {
	float b0 = 1.0f;
	float tracking_rate = 30.0f;
	float tracking_acceleration = 300.0f;
	float kp = 100.0f;
	float kd = 20.0f;
	float beta1 = 30.0f;
	float beta2 = 300.0f;
	float beta3 = 1000.0f;
	float alpha1 = 0.5f;
	float alpha2 = 0.25f;
	float fal_delta = 0.01f;
	float output_limit = 0.0f;
	bool nonlinear_observer = true;
};

class ADRC final
{
public:
	ADRC() = default;

	explicit ADRC(const ADRCConfig &config)
	{
		Configure(config);
	}

	void Configure(const ADRCConfig &config);
	void Reset(float state = 0.0f);
	float Update(float reference, float measurement, float dt);

	float GetOutput() const noexcept { return _output; }

	float TrackingState() const noexcept { return _tracking_state; }

	float TrackingDerivative() const noexcept { return _tracking_derivative; }

	float EstimatedState() const noexcept { return _eso.State(); }

	float EstimatedDerivative() const noexcept { return _eso.StateDerivative(); }

	float EstimatedDisturbance() const noexcept { return _eso.Disturbance(); }

	const ADRCConfig &GetConfig() const noexcept { return _config; }

	ESO &Observer() noexcept { return _eso; }

	const ESO &Observer() const noexcept { return _eso; }

private:
	static float ClampAbs(float value, float limit);
	static float Fal(float error, float alpha, float delta);

	ADRCConfig _config{};
	ESO _eso{};
	float _tracking_state = 0.0f;
	float _tracking_derivative = 0.0f;
	float _output = 0.0f;
	bool _initialized = false;
};

using ADRC = ADRC;

namespace control
{
using ADRC = ::alg::ADRC;
using ADRC = ::alg::ADRC;
using ADRCConfig = ::alg::ADRCConfig;
}
}

using ADRC = alg::ADRC;
using ADRC = alg::ADRC;
using ADRCConfig = alg::ADRCConfig;
