#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace alg
{

struct ESOConfig {
	float beta1 = 30.0f;
	float beta2 = 300.0f;
	float beta3 = 1000.0f;
	float b0 = 1.0f;
	float alpha1 = 0.5f;
	float alpha2 = 0.25f;
	float delta = 0.01f;
	bool nonlinear = true;
};

class ESO final
{
public:
	ESO() = default;

	explicit ESO(const ESOConfig &config)
	{
		Configure(config);
	}

	void Configure(const ESOConfig &config);

	void Reset(float state = 0.0f);

	void Update(float measurement, float input, float dt);

	float State() const noexcept { return _z1; }

	float StateDerivative() const noexcept { return _z2; }

	float Disturbance() const noexcept { return _z3; }

	float Z1() const noexcept { return _z1; }

	float Z2() const noexcept { return _z2; }

	float Z3() const noexcept { return _z3; }

	const ESOConfig &GetConfig() const noexcept { return _config; }

private:
	static float Fal(float error, float alpha, float delta);

	ESOConfig _config{};
	float _z1 = 0.0f;
	float _z2 = 0.0f;
	float _z3 = 0.0f;
};

namespace control
{
using ESO = ::alg::ESO;
using ESOConfig = ::alg::ESOConfig;
}
}

using ESO = alg::ESO;
using ESOConfig = alg::ESOConfig;
