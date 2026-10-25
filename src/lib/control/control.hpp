#pragma once

#include "pid.hpp"

class SpeedLoop
{
public:
	void Configure(const alg::PIDConfig &config);

	void Reset();

	float Update(float target_rad_s, float measured_rad_s, float dt_s);

private:
	alg::Pid _pid{};
};

class PositionSpeedLoop
{
public:
	void Configure(const alg::PIDConfig &position, const alg::PIDConfig &speed);

	void Reset();

	float Update(float target_rad, float measured_rad, float measured_rad_s, float dt_s);

private:
	alg::Pid _position{};
	alg::Pid _speed{};
};
