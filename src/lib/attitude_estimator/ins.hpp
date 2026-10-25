#pragma once

#include <cmath>
#include <errno.h>
#include <stdint.h>
struct AttitudeEstimate {
	float roll_rad{0.0f};
	float pitch_rad{0.0f};
	float yaw_rad{0.0f};
	float yaw_unwrapped_rad{0.0f};
	uint64_t timestamp_sample{0};
	bool valid{false};
};
#include "quaternion.hpp"
#include "type_math.hpp"

class Ins
{
public:
	void Init(const alg::attitude::QuaternionEKF::Config &config);

	bool Process(const float accel_sensor[3], const float gyro_sensor[3], float dt_seconds, uint64_t timestamp_sample);

	void SetAxisRotation(const float rotation[3][3]);

	void ResetAxisRotation();

	bool GetEulerAngles(AttitudeEstimate &angles);

private:
	bool FiniteMatrix(const float matrix[3][3]);

	float Determinant(const float matrix[3][3]);

	bool IsRotationMatrix(const float matrix[3][3]);
	alg::attitude::QuaternionEKF _estimator{};
	alg::attitude::QuaternionEKF::State _estimator_state{};
	float _axis_rotation[3][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};
	bool _axis_rotation_identity = true;
	float _startup_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
	uint32_t _startup_bias_samples = 0;
	uint64_t _last_update_us = 0;
};
