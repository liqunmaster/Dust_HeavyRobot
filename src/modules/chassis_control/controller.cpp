#include "chassis_controller.hpp"
void ChassisController::inverse(float vx_m_s, float vy_m_s, float yaw_rad_s, float wheel_rad_s[4]) const
{
	const float arm = _geometry.rotation_coefficient;
	const float scale = 1.0f / _geometry.wheel_radius_m;
	wheel_rad_s[0] = (vx_m_s - vy_m_s - arm * yaw_rad_s) * scale;
	wheel_rad_s[1] = (vx_m_s + vy_m_s + arm * yaw_rad_s) * scale;
	wheel_rad_s[2] = (vx_m_s + vy_m_s - arm * yaw_rad_s) * scale;
	wheel_rad_s[3] = (vx_m_s - vy_m_s + arm * yaw_rad_s) * scale;
	float largest = 0.0f;

	for (int i = 0; i < 4; ++i) {largest = std::fmax(largest, std::fabs(wheel_rad_s[i]));}

	if (largest > _geometry.max_wheel_rad_s) {
		const float scale = _geometry.max_wheel_rad_s / largest;

		for (int i = 0; i < 4; ++i) {wheel_rad_s[i] *= scale;}
	}
}
