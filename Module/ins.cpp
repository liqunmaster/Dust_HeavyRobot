#include "ins.hpp"

namespace
{
    Ins ins;
}

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
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
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

void Ins::Init()
{
    alg::attitude::QuaternionEKF::Config config{};
    config.qq = 0.1f;
    config.qb = 0.0001f;
    config.r = 0.02f;
    config.lambda = 1.0f;
    config.alpha = 0.0f;
    config.chi2_th = 16.0f;
    config.w_stable_th = 0.05f;
    config.a_ref = math::standard_gravity_mps2;
    config.a_tol = 0.5f;
    estimator_.Init(config);
    estimator_state_ = {};
    startup_gyro_bias_[0] = 0.0f;
    startup_gyro_bias_[1] = 0.0f;
    startup_gyro_bias_[2] = 0.0f;
    startup_bias_samples_ = 0;
    ins_reset_axis_rotation();
}

bool Ins::Process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds)
{
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0f || dt_seconds > 0.1f) {
        return false;
    }

    const float accel_sensor[3] = {
        math::accel_raw_to_mps2(ax_raw, 2048.0f),
        math::accel_raw_to_mps2(ay_raw, 2048.0f),
        math::accel_raw_to_mps2(az_raw, 2048.0f),
    };
    const float gyro_sensor[3] = {
        math::gyro_raw_to_radians_per_second(gx_raw, 16.4f),
        math::gyro_raw_to_radians_per_second(gy_raw, 16.4f),
        math::gyro_raw_to_radians_per_second(gz_raw, 16.4f),
    };

    const float accel_norm = math::sqrt(accel_sensor[0] * accel_sensor[0] + accel_sensor[1] * accel_sensor[1] + accel_sensor[2] * accel_sensor[2]);
    const float gyro_norm = math::sqrt(gyro_sensor[0] * gyro_sensor[0] + gyro_sensor[1] * gyro_sensor[1] + gyro_sensor[2] * gyro_sensor[2]);
    if (startup_bias_samples_ < 400 && std::fabs(accel_norm - math::standard_gravity_mps2) < 1.5f && gyro_norm < 0.2f) {
        constexpr float bias_alpha = 1.0f / 200.0f;
        for (int axis = 0; axis < 3; ++axis) {
            startup_gyro_bias_[axis] += bias_alpha * (gyro_sensor[axis] - startup_gyro_bias_[axis]);
        }
        ++startup_bias_samples_;
    }

    alg::attitude::Sample sample{};
    sample.dt = dt_seconds;
    float rotation[3][3];
    bool rotation_identity;
    k_spinlock_key_t key = k_spin_lock(&ins_lock_);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            rotation[row][column] = axis_rotation_[row][column];
        }
    }
    rotation_identity = axis_rotation_identity_;
    k_spin_unlock(&ins_lock_, key);

    for (int row = 0; row < 3; ++row) {
        sample.accel[row] = accel_sensor[row];
        sample.gyro[row] = gyro_sensor[row] - startup_gyro_bias_[row];
        if (!rotation_identity) {
            sample.accel[row] = 0.0f;
            sample.gyro[row] = 0.0f;
            for (int column = 0; column < 3; ++column) {
                sample.accel[row] += rotation[row][column] * accel_sensor[column];
                sample.gyro[row] += rotation[row][column] * (gyro_sensor[column] - startup_gyro_bias_[column]);
            }
        }
    }

    estimator_.Update(sample);
    const auto updated_state = estimator_.GetState();
    key = k_spin_lock(&ins_lock_);
    estimator_state_ = updated_state;
    last_update_ms_ = k_uptime_get_32();
    k_spin_unlock(&ins_lock_, key);
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

    const k_spinlock_key_t key = k_spin_lock(&ins_lock_);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            axis_rotation_[row][column] = rotation[row][column];
        }
    }

    axis_rotation_identity_ = true;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            axis_rotation_identity_ = axis_rotation_identity_ && rotation[row][column] == (row == column ? 1.0f : 0.0f);
        }
    }
    k_spin_unlock(&ins_lock_, key);
}

void Ins::ResetAxisRotation()
{
    constexpr float identity[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    ins_set_axis_rotation(identity);
}

bool Ins::GetEulerAngles(InsEulerAngles &angles)
{
    alg::attitude::QuaternionEKF::State state;
    const k_spinlock_key_t key = k_spin_lock(&ins_lock_);
    state = estimator_state_;
    angles.timestamp_ms = last_update_ms_;
    k_spin_unlock(&ins_lock_, key);

    if (!state.init || !std::isfinite(state.roll) || !std::isfinite(state.pitch) || !std::isfinite(state.yaw) || !std::isfinite(state.yaw_sum)) {
        return false;
    }

    angles.roll_rad = math::degrees_to_radians(state.roll);
    angles.pitch_rad = math::degrees_to_radians(state.pitch);
    angles.yaw_rad = math::degrees_to_radians(state.yaw);
    angles.yaw_unwrapped_rad = math::degrees_to_radians(state.yaw_sum);

    return true;
}

void ins_init()
{
    ins.Init();
}

bool ins_process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds)
{
    return ins.Process(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, dt_seconds);
}

void ins_set_axis_rotation(const float rotation[3][3])
{
    ins.SetAxisRotation(rotation);
}

void ins_reset_axis_rotation()
{
    ins.ResetAxisRotation();
}

bool ins_get_euler_angles(InsEulerAngles &angles)
{
    return ins.GetEulerAngles(angles);
}
