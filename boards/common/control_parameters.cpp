#include "control_parameters.hpp"
namespace board
{
ManualControlConfig manual_config() { return {}; }
AttitudeEstimatorConfig attitude_config()
{
	AttitudeEstimatorConfig c{};
	c.ekf.qq = 0.1f;
	c.ekf.qb = 0.0001f;
	c.ekf.r = 0.02f;
	c.ekf.lambda = 1.0f;
	c.ekf.alpha = 0.0f;
	c.ekf.chi2_th = 16.0f;
	c.ekf.w_stable_th = 0.05f;
	c.ekf.a_ref = math::standard_gravity_mps2;
	c.ekf.a_tol = 0.5f;
	c.accel_m_s2_per_count = math::standard_gravity_mps2 / 2048.0f;
	c.gyro_rad_s_per_count = math::pi / (180.0f * 16.4f);
	return c;
}
#if defined(ROBOT_BOARD_H)
namespace
{
CanMotorDevice<C620> yaw_motor{{CANFD_DEVICE_CAN1, 0, 2, 3591.0f / 187.0f}};
}
YawControlConfig yaw_config()
{
	YawControlConfig c{};
	c.motor = &yaw_motor;
	c.position_pid.kp = 7.5f;
	c.position_pid.kd = 0.20f;
	c.position_pid.output_limit = 20.0f;
	c.position_pid.dt = 0.001f;
	c.position_pid.derivative_on_measurement = alg::DFirst::Enable;
	c.position_pid.derivative_filter_tau = 0.01f;
	c.speed_pid.kp = 3.75f;
	c.speed_pid.output_limit = 20.0f;
	c.speed_pid.dt = 0.001f;
	return c;
}
#else
namespace
{
CanMotorDevice<C620> chassis_motors[4] = {
	{{CANFD_DEVICE_CAN0, 0, 1, 3591.0f / 187.0f}}, {{CANFD_DEVICE_CAN0, 1, 2, 3591.0f / 187.0f}},
	{{CANFD_DEVICE_CAN0, 2, 3, 3591.0f / 187.0f}}, {{CANFD_DEVICE_CAN0, 3, 4, 3591.0f / 187.0f}}
};
CanMotorDevice<Cubemars> lift_motors[4] = {
	{{CANFD_DEVICE_CAN0, 4, 1, 1.0f, 0, 0, {12.5f, 50.0f, 65.0f}}},
	{{CANFD_DEVICE_CAN1, 5, 1, 1.0f, 0, 0, {12.5f, 50.0f, 65.0f}}},
	{{CANFD_DEVICE_CAN2, 6, 1, 1.0f, 0, 0, {12.5f, 50.0f, 65.0f}}},
	{{CANFD_DEVICE_CAN3, 7, 1, 1.0f, 0, 0, {12.5f, 50.0f, 65.0f}}}
};
CanMotorDevice<DmMotor> booster_motor{{CANFD_DEVICE_CAN2, 8, 7, 1.0f, 8, 7, {12.5f, 30.0f, 10.0f}}};
}
ChassisControlConfig chassis_config()
{
	ChassisControlConfig c{};

	for (int i = 0; i < 4; ++i) {c.motors[i] = &chassis_motors[i];}

	c.speed_pid.kp = 3.0f;
	c.speed_pid.integral_limit = 10.0f;
	c.speed_pid.output_limit = 20.0f;
	c.speed_pid.dt = 0.001f;
	// 架构迁移期间保留现有固件的系数。
	// 现有 409/234 尺寸疑似以毫米为单位；修正单位需要单独调整控制逻辑。
	c.rotation_coefficient = 409.0f / 2 + 234.0f / 2;
	c.wheel_radius_m = 0.075f;
	c.max_wheel_rad_s = 32767.0f * 6.28318530718f / (60.0f * (3591.0f / 187.0f));
	return c;
}
LiftControlConfig lift_config()
{
	LiftControlConfig c{};

	for (int i = 0; i < 4; ++i) {c.motors[i] = &lift_motors[i];}

	c.position_pid.kp = 20.0f;
	c.position_pid.output_limit = 50.0f;
	c.position_pid.dt = 0.002f;
	c.speed_pid.kp = 2.0f;
	c.speed_pid.integral_limit = 32.5f;
	c.speed_pid.output_limit = 65.0f;
	c.speed_pid.dt = 0.002f;
	return c;
}
BoosterControlConfig booster_config()
{
	BoosterControlConfig c{};
	c.motor = &booster_motor;
	c.angle_pid.kp = 8.0f;
	c.angle_pid.ki = 1.0f;
	c.angle_pid.kd = 0.3f;
	c.angle_pid.output_limit = 5.0f;
	c.angle_pid.dt = 0.001f;
	c.angle_pid.derivative_on_measurement = alg::DFirst::Enable;
	c.angle_pid.derivative_filter_tau = 0.01f;
	return c;
}
#endif
}
