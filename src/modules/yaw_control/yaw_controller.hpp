#pragma once
#include "pid.hpp"
class YawController
{
public:
	void configure(const alg::PIDConfig &position, const alg::PIDConfig &speed) {_position.Configure(position); _speed.Configure(speed);}
	float update(bool active, float target, float angle, float speed, float dt)
	{
		if (!active) {_position.Reset(); _speed.Reset(); return 0.0f;}

		return _speed.Update(_position.UpdateAngle(target, angle, dt), speed, dt);
	}
private:
	alg::Pid _position{};
	alg::Pid _speed{};
};
