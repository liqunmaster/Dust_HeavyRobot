#pragma once
#include "control.hpp"
#include <cmath>
struct ChassisGeometry {
	float rotation_coefficient{0.0f};
	float wheel_radius_m{0.075f};
	float max_wheel_rad_s{0.0f};
};
class ChassisController
{
public:
	void configure(const ChassisGeometry &geometry, const alg::PIDConfig &pid)
	{
		_geometry = geometry;

		for (auto &loop : _loops) {loop.Configure(pid);}
	}
	void inverse(float vx, float vy, float yaw, float targets[4]) const;
	float update(unsigned wheel, float target, float speed, bool fresh, float dt)
	{
		if (wheel >= 4) {return 0.0f;}

		if (!fresh) {_loops[wheel].Reset(); return 0.0f;}

		return _loops[wheel].Update(target, speed, dt);
	}
	void reset() {for (auto &loop : _loops) {loop.Reset();}}
private:
	ChassisGeometry _geometry{};
	SpeedLoop _loops[4] {};
};
