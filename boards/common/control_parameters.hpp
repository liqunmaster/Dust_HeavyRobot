#pragma once
#include "manual_control.hpp"
#include "attitude_estimator.hpp"
#include "CanMotorDevice.hpp"
#if defined(ROBOT_BOARD_H)
#include "yaw_control.hpp"
#else
#include "chassis_control.hpp"
#include "lift_control.hpp"
#include "booster_control.hpp"
#endif
namespace board
{
ManualControlConfig manual_config();
AttitudeEstimatorConfig attitude_config();
#if defined(ROBOT_BOARD_H)
YawControlConfig yaw_config();
#else
ChassisControlConfig chassis_config();
LiftControlConfig lift_config();
BoosterControlConfig booster_config();
#endif
}
