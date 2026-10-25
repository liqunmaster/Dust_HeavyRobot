#include "control.hpp"

void SpeedLoop::Configure(const alg::PIDConfig &config)
{
	_pid.Configure(config);
}

void SpeedLoop::Reset()
{
	_pid.Reset();
}

float SpeedLoop::Update(float target_rad_s, float measured_rad_s, float dt_s)
{
	return _pid.Update(target_rad_s, measured_rad_s, dt_s);
}

void PositionSpeedLoop::Configure(const alg::PIDConfig &position, const alg::PIDConfig &speed)
{
	_position.Configure(position);
	_speed.Configure(speed);
}

void PositionSpeedLoop::Reset()
{
	_position.Reset();
	_speed.Reset();
}

float PositionSpeedLoop::Update(float target_rad, float measured_rad, float measured_rad_s, float dt_s)
{
	const float target_speed = _position.Update(target_rad, measured_rad, dt_s);
	return _speed.Update(target_speed, measured_rad_s, dt_s);
}
