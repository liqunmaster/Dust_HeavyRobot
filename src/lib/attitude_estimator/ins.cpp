#include "ins.hpp"

bool Ins::FiniteMatrix(const float matrix[3][3])
{
	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			if (!std::isfinite(matrix[row][column])) {
				return false;
			}
		}
	}

	return true;
}

float Ins::Determinant(const float matrix[3][3])
{
	return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) - matrix[0][1] *
	       (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1]
			       * matrix[2][0]);
}

bool Ins::IsRotationMatrix(const float matrix[3][3])
{
	for (int row = 0; row < 3; ++row) {
		float norm = 0.0f;

		for (int column = 0; column < 3; ++column) {
			norm += matrix[row][column] * matrix[row][column];
		}

		if (std::fabs(norm - 1.0f) > 0.05f) {
			return false;
		}
	}

	for (int row = 0; row < 3; ++row) {
		for (int other = row + 1; other < 3; ++other) {
			float dot = 0.0f;

			for (int column = 0; column < 3; ++column) {
				dot += matrix[row][column] * matrix[other][column];
			}

			if (std::fabs(dot) > 0.05f) {
				return false;
			}
		}
	}

	return true;
}

void Ins::Init(const alg::attitude::QuaternionEKF::Config &config)
{
	_estimator.Init(config);
	_estimator_state = {};
	_startup_gyro_bias[0] = 0.0f;
	_startup_gyro_bias[1] = 0.0f;
	_startup_gyro_bias[2] = 0.0f;
	_startup_bias_samples = 0;
	ResetAxisRotation();
}

bool Ins::Process(const float accel_sensor[3], const float gyro_sensor[3], float dt_seconds, uint64_t timestamp_sample)
{
	if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0f || dt_seconds > 0.1f) {
		return false;
	}

	for (int axis = 0; axis < 3; ++axis) {
		if (!std::isfinite(accel_sensor[axis]) || !std::isfinite(gyro_sensor[axis])) { return false; }
	}

	const float accel_norm = math::sqrt(accel_sensor[0] * accel_sensor[0] + accel_sensor[1] * accel_sensor[1] +
					    accel_sensor[2] * accel_sensor[2]);
	const float gyro_norm = math::sqrt(gyro_sensor[0] * gyro_sensor[0] + gyro_sensor[1] * gyro_sensor[1] + gyro_sensor[2] *
					   gyro_sensor[2]);

	if (_startup_bias_samples < 400 && std::fabs(accel_norm - math::standard_gravity_mps2) < 1.5f && gyro_norm < 0.2f) {
		constexpr float bias_alpha = 1.0f / 200.0f;

		for (int axis = 0; axis < 3; ++axis) {
			_startup_gyro_bias[axis] += bias_alpha * (gyro_sensor[axis] - _startup_gyro_bias[axis]);
		}

		++_startup_bias_samples;
	}

	alg::attitude::Sample sample{};
	sample.dt = dt_seconds;
	float rotation[3][3];
	bool rotation_identity;

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			rotation[row][column] = _axis_rotation[row][column];
		}
	}

	rotation_identity = _axis_rotation_identity;

	for (int row = 0; row < 3; ++row) {
		sample.accel[row] = accel_sensor[row];
		sample.gyro[row] = gyro_sensor[row] - _startup_gyro_bias[row];

		if (!rotation_identity) {
			sample.accel[row] = 0.0f;
			sample.gyro[row] = 0.0f;

			for (int column = 0; column < 3; ++column) {
				sample.accel[row] += rotation[row][column] * accel_sensor[column];
				sample.gyro[row] += rotation[row][column] * (gyro_sensor[column] - _startup_gyro_bias[column]);
			}
		}
	}

	_estimator.Update(sample);
	const auto updated_state = _estimator.GetState();
	_estimator_state = updated_state;
	_last_update_us = timestamp_sample;
	return updated_state.init;
}

void Ins::SetAxisRotation(const float rotation[3][3])
{
	if (rotation == nullptr || !FiniteMatrix(rotation)) {
		return;
	}

	const float det = Determinant(rotation);

	if (!std::isfinite(det) || std::fabs(det) < 0.5f || std::fabs(det) > 1.5f || !IsRotationMatrix(rotation)) {
		return;
	}

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			_axis_rotation[row][column] = rotation[row][column];
		}
	}

	_axis_rotation_identity = true;

	for (int row = 0; row < 3; ++row) {
		for (int column = 0; column < 3; ++column) {
			_axis_rotation_identity = _axis_rotation_identity && rotation[row][column] == (row == column ? 1.0f : 0.0f);
		}
	}
}

void Ins::ResetAxisRotation()
{
	constexpr float identity[3][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f},
	};
	SetAxisRotation(identity);
}

bool Ins::GetEulerAngles(AttitudeEstimate &angles)
{
	alg::attitude::QuaternionEKF::State state;
	state = _estimator_state;
	angles.timestamp_sample = _last_update_us;

	if (!state.init || !std::isfinite(state.roll) || !std::isfinite(state.pitch) || !std::isfinite(state.yaw)
	    || !std::isfinite(state.yaw_sum)) {
		return false;
	}

	angles.valid = true;
	angles.roll_rad = math::degrees_to_radians(state.roll);
	angles.pitch_rad = math::degrees_to_radians(state.pitch);
	angles.yaw_rad = math::degrees_to_radians(state.yaw);
	angles.yaw_unwrapped_rad = math::degrees_to_radians(state.yaw_sum);

	return true;
}
