#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <cstdint>

namespace alg
{

enum class DFirst : uint8_t {
	Disable = 0,
	Enable = 1,
};

constexpr DFirst PID_D_First_DISABLE = DFirst::Disable;
constexpr DFirst PID_D_First_ENABLE = DFirst::Enable;

struct PIDConfig {
	float kp = 0.0f;
	float ki = 0.0f;
	float kd = 0.0f;
	float kf = 0.0f;
	float integral_limit = 0.0f;
	float output_limit = 0.0f;
	float dt = 0.001f;
	float dead_zone = 0.0f;
	float variable_speed_a = 0.0f;
	float variable_speed_b = 0.0f;
	float integral_separation = 0.0f;
	DFirst derivative_on_measurement = DFirst::Disable;
	float derivative_filter_tau = 0.0f;
};

class Pid final
{
public:
	Pid() = default;

	explicit Pid(const PIDConfig &config)
	{
		Configure(config);
	}

	void Configure(const PIDConfig &config);

	void Reset();

	float Update(float target, float measurement);

	float Update(float target, float measurement, float dt);

	float UpdateAngle(float target, float measurement);

	float UpdateAngle(float target, float measurement, float dt);

	void Init(float kp, float ki, float kd, float kf = 0.0f, float integral_limit = 0.0f,

		  float output_limit = 0.0f, float dt = 0.001f, float dead_zone = 0.0f,

		  float variable_speed_a = 0.0f, float variable_speed_b = 0.0f,

		  float integral_separation = 0.0f, DFirst derivative_on_measurement = DFirst::Disable,

		  float derivative_filter_tau = 0.0f);

	void SetKp(float kp);

	void SetKi(float ki);

	void SetKd(float kd);

	void SetKf(float kf);

	void SetIOutMax(float limit);

	void SetOutMax(float limit);

	void SetIVariableSpeedA(float value);

	void SetIVariableSpeedB(float value);

	void SetISeparateThreshold(float value);

	void SetTarget(float target);

	void SetNow(float measurement);

	void SetIntegralError(float integral_error);

	void CalculatePeriodElapsedCallback();

	void CalculateAnglePid();

	float GetOut() const noexcept { return _output; }

	float GetIntegralError() const noexcept { return _integral_error; }

	float GetOutput() const noexcept { return _output; }

	const PIDConfig &GetConfig() const noexcept { return _config; }

private:
	float Calculate(float target, float measurement, float dt, bool angle);

	static float ClampAbs(float value, float limit);

	static float WrapAngle(float value);

	PIDConfig _config{};

	float _target = 0.0f;

	float _measurement = 0.0f;

	float _previous_target = 0.0f;

	float _previous_measurement = 0.0f;

	float _previous_error = 0.0f;

	float _integral_error = 0.0f;

	float _derivative_output = 0.0f;

	float _output = 0.0f;

	bool _initialized = false;
};

using PID = Pid;
}

using Pid = alg::Pid;
using PID = alg::PID;
using PIDConfig = alg::PIDConfig;
using DFirst = alg::DFirst;
using alg::PID_D_First_DISABLE;
using alg::PID_D_First_ENABLE;
