#include "ins.hpp"

namespace
{
    constexpr float accel_lsb_per_g = 2048.0F;

    constexpr float gyro_lsb_per_dps = 16.4F;

    alg::attitude::QuaternionEkf estimator{};
    alg::attitude::QuaternionEkf::State estimator_state{};

    float axis_rotation[3][3] = {
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
    };

    bool axis_rotation_identity = true;

    struct k_spinlock ins_lock;

    float startup_gyro_bias[3] = {0.0F, 0.0F, 0.0F};

    uint32_t startup_bias_samples = 0U;

    bool finite_matrix(const float matrix[3][3])
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

    float determinant(const float matrix[3][3])
    {
        return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }

    bool is_rotation_matrix(const float matrix[3][3])
    {
        for (int row = 0; row < 3; ++row) {
            float norm = 0.0F;
            for (int column = 0; column < 3; ++column) {
                norm += matrix[row][column] * matrix[row][column];
            }
            if (std::fabs(norm - 1.0F) > 0.05F) {
                return false;
            }
        }

        for (int row = 0; row < 3; ++row) {
            for (int other = row + 1; other < 3; ++other) {
                float dot = 0.0F;
                for (int column = 0; column < 3; ++column) {
                    dot += matrix[row][column] * matrix[other][column];
                }
                if (std::fabs(dot) > 0.05F) {
                    return false;
                }
            }
        }
        return true;
    }
}

int ins_init()
{
    alg::attitude::QuaternionEkf::Config config{};
    config.qq = 0.1F;
    config.qb = 0.0001F;
    config.r = 0.02F;
    config.lambda = 1.0F;
    config.alpha = 0.0F;
    config.chi2_th = 16.0F;
    config.w_stable_th = 0.05F;
    config.a_ref = math::standard_gravity_mps2;
    config.a_tol = 0.5F;
    estimator.init(config);
    estimator_state = {};
    startup_gyro_bias[0] = 0.0F;
    startup_gyro_bias[1] = 0.0F;
    startup_gyro_bias[2] = 0.0F;
    startup_bias_samples = 0U;
    return ins_reset_axis_rotation();
}

bool ins_process(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw, float dt_seconds)
{
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0F || dt_seconds > 0.1F) {
        return false;
    }

    const float accel_sensor[3] = {
        math::accel_raw_to_mps2(ax_raw, accel_lsb_per_g),
        math::accel_raw_to_mps2(ay_raw, accel_lsb_per_g),
        math::accel_raw_to_mps2(az_raw, accel_lsb_per_g),
    };
    const float gyro_sensor[3] = {
        math::gyro_raw_to_radians_per_second(gx_raw, gyro_lsb_per_dps),
        math::gyro_raw_to_radians_per_second(gy_raw, gyro_lsb_per_dps),
        math::gyro_raw_to_radians_per_second(gz_raw, gyro_lsb_per_dps),
    };

    const float accel_norm = math::sqrt(accel_sensor[0] * accel_sensor[0] + accel_sensor[1] * accel_sensor[1] + accel_sensor[2] * accel_sensor[2]);
    const float gyro_norm = math::sqrt(gyro_sensor[0] * gyro_sensor[0] + gyro_sensor[1] * gyro_sensor[1] + gyro_sensor[2] * gyro_sensor[2]);
    if (startup_bias_samples < 400U && std::fabs(accel_norm - math::standard_gravity_mps2) < 1.5F && gyro_norm < 0.2F) {
        constexpr float bias_alpha = 1.0F / 200.0F;
        for (int axis = 0; axis < 3; ++axis) {
            startup_gyro_bias[axis] += bias_alpha * (gyro_sensor[axis] - startup_gyro_bias[axis]);
        }
        ++startup_bias_samples;
    }

    alg::attitude::Sample sample{};
    sample.dt = dt_seconds;
    float rotation[3][3];
    bool rotation_identity;
    k_spinlock_key_t key = k_spin_lock(&ins_lock);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            rotation[row][column] = axis_rotation[row][column];
        }
    }
    rotation_identity = axis_rotation_identity;
    k_spin_unlock(&ins_lock, key);

    for (int row = 0; row < 3; ++row) {
        sample.accel[row] = accel_sensor[row];
        sample.gyro[row] = gyro_sensor[row] - startup_gyro_bias[row];
        if (!rotation_identity) {
            sample.accel[row] = 0.0F;
            sample.gyro[row] = 0.0F;
            for (int column = 0; column < 3; ++column) {
                sample.accel[row] += rotation[row][column] * accel_sensor[column];
                sample.gyro[row] += rotation[row][column] * (gyro_sensor[column] - startup_gyro_bias[column]);
            }
        }
    }

    estimator.update(sample);
    const auto updated_state = estimator.get_state();
    key = k_spin_lock(&ins_lock);
    estimator_state = updated_state;
    k_spin_unlock(&ins_lock, key);
    return updated_state.init;
}

int ins_set_axis_rotation(const float rotation[3][3])
{
    if (rotation == nullptr || !finite_matrix(rotation)) {
        return -EINVAL;
    }

    const float det = determinant(rotation);
    if (!std::isfinite(det) || std::fabs(det) < 0.5F || std::fabs(det) > 1.5F || !is_rotation_matrix(rotation)) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&ins_lock);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            axis_rotation[row][column] = rotation[row][column];
        }
    }

    axis_rotation_identity = true;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            axis_rotation_identity = axis_rotation_identity && rotation[row][column] == (row == column ? 1.0F : 0.0F);
        }
    }
    k_spin_unlock(&ins_lock, key);
    return 0;
}

int ins_reset_axis_rotation()
{
    constexpr float identity[3][3] = {
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
    };
    return ins_set_axis_rotation(identity);
}

bool ins_get_euler_angles(ins_euler_angles &angles)
{
    alg::attitude::QuaternionEkf::State state;
    const k_spinlock_key_t key = k_spin_lock(&ins_lock);
    state = estimator_state;
    k_spin_unlock(&ins_lock, key);

    if (!state.init || !std::isfinite(state.roll) || !std::isfinite(state.pitch) || !std::isfinite(state.yaw) || !std::isfinite(state.yaw_sum)) {
        return false;
    }

    angles.roll_rad = math::degrees_to_radians(state.roll);
    angles.pitch_rad = math::degrees_to_radians(state.pitch);
    angles.yaw_rad = math::degrees_to_radians(state.yaw);
    angles.yaw_unwrapped_rad = math::degrees_to_radians(state.yaw_sum);
    return true;
}
