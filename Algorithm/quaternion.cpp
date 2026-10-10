#include "quaternion.hpp"

namespace
{

    /**
     * @brief 将角度折叠到 (-180°, 180°]
     *
     * @param value 输入角度（度）
     * @return 折叠后的角度（度）
    */
    float wrap_degrees(float value)
    {
        value = std::remainder(value, 360.0f);
        return value == -180.0f ? 180.0f : value;
    }
}

namespace alg::attitude
{

    /**
     * @brief 初始化四元数 EKF 姿态估计器
     *
     * @param config 配置结构体
    */
    void QuaternionEkf::init(const Config &config)
    {
        config_ = config;
        config_.lambda = std::clamp(config_.lambda, 0.01f, 1.0f);
        state_ = {};
        state_.init = true;
        static_count_ = 0;
        static_z_sum_ = 0.0f;
        static_z_count_ = 0;
        previous_yaw_ = 0.0f;
        yaw_turns_ = 0;
        previous_q_[0] = 1.0f;
        previous_q_[1] = previous_q_[2] = previous_q_[3] = 0.0f;
        have_previous_q_ = false;

        EKF::State initial = EKF::State::Zero();
        initial(0) = 1.0f;
        EKF::Cov covariance = EKF::Cov::Identity();
        covariance.topLeftCorner<4, 4>() *= 1e-3f;
        covariance.bottomRightCorner<2, 2>() *= 0.1f;
        ekf_.init(initial, covariance);
        ekf_.set_system_func(system_func);
        ekf_.set_observe_func(observe_func);
        ekf_.set_gain_row_scale(3, 0.0f);
    }

    /**
     * @brief 依据加速度计测量值初始化姿态（横滚/俯仰到四元数）
     *
     * @param sample 一次传感器采样
    */
    void QuaternionEkf::init_from_accel(const Sample &sample)
    {
        const float norm = math::sqrt(sample.accel[0] * sample.accel[0] + sample.accel[1] * sample.accel[1] + sample.accel[2] * sample.accel[2]);
        if (!std::isfinite(norm) || norm < 1e-3f) {
            return;
        }

        const float roll = math::atan2(sample.accel[1], sample.accel[2]);
        const float pitch = -math::asin_clamped(sample.accel[0] / norm);
        const float cr = math::cos(0.5f * roll), sr = math::sin(0.5f * roll);
        const float cp = math::cos(0.5f * pitch), sp = math::sin(0.5f * pitch);
        EKF::State initial = EKF::State::Zero();
        initial << cp * cr, cp * sr, sp * cr, -sp * sr, 0.0f, 0.0f;
        ekf_.set_state(initial);
        for (int axis = 0; axis < 3; ++axis) {
            state_.a[axis] = sample.accel[axis];
        }
    }

    /**
     * @brief 输入一次 IMU 采样并更新姿态估计（含零偏漂移补偿与收敛检测）
     *
     * @param sample 一次传感器采样
    */
    void QuaternionEkf::update(const Sample &sample)
    {
        if (!state_.init) {
            init(Config{});
        }
        if (!std::isfinite(sample.dt) || sample.dt <= 0.0f || sample.dt > 0.1f) {
            return;
        }
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(sample.gyro[axis]) || !std::isfinite(sample.accel[axis])) {
                return;
            }
        }

        state_.dt = sample.dt;
        if (state_.upd_cnt == 0) {
            init_from_accel(sample);
        }
        const float accel_weight = sample.dt / (sample.dt + std::max(config_.alpha, 0.0f));
        for (int axis = 0; axis < 3; ++axis) {
            state_.a[axis] += accel_weight * (sample.accel[axis] - state_.a[axis]);
            state_.w[axis] = sample.gyro[axis] - state_.bg[axis];
        }

        state_.a_norm = math::sqrt(state_.a[0] * state_.a[0] + state_.a[1] * state_.a[1] + state_.a[2] * state_.a[2]);
        state_.w_norm = math::sqrt(state_.w[0] * state_.w[0] + state_.w[1] * state_.w[1] + state_.w[2] * state_.w[2]);
        state_.stable = state_.w_norm < config_.w_stable_th && std::fabs(state_.a_norm - config_.a_ref) < config_.a_tol;

        if (state_.stable && std::fabs(state_.w[2]) < 0.02f) {
            if (static_count_ < 100) {
                ++static_count_;
            }
            if (static_count_ == 100) {
                static_z_sum_ += sample.gyro[2];
                if (++static_z_count_ == 500) {
                    state_.bg[2] = static_z_sum_ / static_z_count_;
                    static_z_sum_ = 0.0f;
                    static_z_count_ = 0;
                }
            }
        } else {
            static_count_ = 0;
            static_z_sum_ = 0.0f;
            static_z_count_ = 0;
        }
        state_.w[2] = sample.gyro[2] - state_.bg[2];

        if (state_.upd_cnt > 0) {
            EKF::Ctrl control;
            control << state_.w[0] * sample.dt, state_.w[1] * sample.dt,
                state_.w[2] * sample.dt, sample.dt;
            EKF::Cov process_noise = EKF::Cov::Zero();
            process_noise.diagonal() << config_.qq * sample.dt, config_.qq * sample.dt,
                config_.qq * sample.dt, config_.qq * sample.dt,
                config_.qb * sample.dt, config_.qb * sample.dt;
            if (!ekf_.predict(control, process_noise)) {
                return;
            }

            ekf_.fade_predicted_variance(4, config_.lambda, config_.pb_limit);
            ekf_.fade_predicted_variance(5, config_.lambda, config_.pb_limit);
            ekf_.set_correction_limit(4, config_.bias_limit * sample.dt);
            ekf_.set_correction_limit(5, config_.bias_limit * sample.dt);

            if (std::isfinite(state_.a_norm) && state_.a_norm > 1e-3f && std::fabs(state_.a_norm - config_.a_ref) < config_.a_tol) {
                EKF::Obs gravity;
                gravity << state_.a[0] / state_.a_norm,
                    state_.a[1] / state_.a_norm,
                    state_.a[2] / state_.a_norm;
                const EKF::ObsCov measurement_noise = EKF::ObsCov::Identity() * config_.r;
                const float gate = state_.converg ? config_.chi2_th : std::numeric_limits<float>::infinity();
                bool corrected = ekf_.update(gravity, measurement_noise, gate);
                state_.chi2 = ekf_.get_chi2();
                if (corrected) {
                    state_.converg = true;
                    state_.err_cnt = 0;
                } else if (state_.converg && state_.stable && std::isfinite(state_.chi2) && state_.chi2 > config_.chi2_th) {
                    if (++state_.err_cnt > config_.div_limit) {
                        state_.converg = false;
                        state_.err_cnt = 0;
                    }
                }
            }
        }

        EKF::State corrected = ekf_.get_x();
        if (!normalize(corrected)) {
            return;
        }
        ekf_.set_state(corrected);
        for (int axis = 0; axis < 4; ++axis) {
            state_.q[axis] = corrected(axis);
        }
        state_.bg[0] = corrected(4);
        state_.bg[1] = corrected(5);
        update_angles();
        ++state_.upd_cnt;
    }

    /**
     * @brief 系统模型函数：四元数姿态运动学递推及其雅可比
     *
     * @param state 当前状态向量
     * @param control 控制输入
     * @param predicted 输出的预测状态
     * @param jacobian 输出的雅可比矩阵
    */
    void QuaternionEkf::system_func(const EKF::State &state, const EKF::Ctrl &control, EKF::State &predicted, EKF::Cov &jacobian)
    {
        const float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);
        const float hx = 0.5f * control(0), hy = 0.5f * control(1);
        const float hz = 0.5f * control(2), hdt = 0.5f * control(3);
        predicted = state;
        predicted(0) = q0 - q1 * hx - q2 * hy - q3 * hz;
        predicted(1) = q1 + q0 * hx + q2 * hz - q3 * hy;
        predicted(2) = q2 + q0 * hy - q1 * hz + q3 * hx;
        predicted(3) = q3 + q0 * hz + q1 * hy - q2 * hx;
        normalize(predicted);

        jacobian.setIdentity();
        jacobian(0, 1) = -hx;
        jacobian(0, 2) = -hy;
        jacobian(0, 3) = -hz;
        jacobian(1, 0) = hx;
        jacobian(1, 2) = hz;
        jacobian(1, 3) = -hy;
        jacobian(2, 0) = hy;
        jacobian(2, 1) = -hz;
        jacobian(2, 3) = hx;
        jacobian(3, 0) = hz;
        jacobian(3, 1) = hy;
        jacobian(3, 2) = -hx;
        jacobian(0, 4) = q1 * hdt;
        jacobian(0, 5) = q2 * hdt;
        jacobian(1, 4) = -q0 * hdt;
        jacobian(1, 5) = q3 * hdt;
        jacobian(2, 4) = -q3 * hdt;
        jacobian(2, 5) = -q0 * hdt;
        jacobian(3, 4) = q2 * hdt;
        jacobian(3, 5) = -q1 * hdt;
    }

    /**
     * @brief 观测模型函数：由四元数预测重力加速度及其雅可比
     *
     * @param state 当前状态向量
     * @param predicted 输出的预测观测值
     * @param jacobian 输出的雅可比矩阵
    */
    void QuaternionEkf::observe_func(const EKF::State &state, EKF::Obs &predicted, EKF::ObsMat &jacobian)
    {
        const float q0 = state(0), q1 = state(1), q2 = state(2), q3 = state(3);
        predicted << 2.0f * (q1 * q3 - q0 * q2),
            2.0f * (q0 * q1 + q2 * q3),
            q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        jacobian.setZero();
        jacobian(0, 0) = -2.0f * q2;
        jacobian(0, 1) = 2.0f * q3;
        jacobian(0, 2) = -2.0f * q0;
        jacobian(0, 3) = 2.0f * q1;
        jacobian(1, 0) = 2.0f * q1;
        jacobian(1, 1) = 2.0f * q0;
        jacobian(1, 2) = 2.0f * q3;
        jacobian(1, 3) = 2.0f * q2;
        jacobian(2, 0) = 2.0f * q0;
        jacobian(2, 1) = -2.0f * q1;
        jacobian(2, 2) = -2.0f * q2;
        jacobian(2, 3) = 2.0f * q3;
    }

    /**
     * @brief 对四元数部分进行归一化
     *
     * @param state 待归一化的状态向量
     * @return 归一化是否成功
    */
    bool QuaternionEkf::normalize(EKF::State &state)
    {
        const float length = math::sqrt(state.head<4>().squaredNorm());
        if (!std::isfinite(length) || length < 1e-6f) {
            return false;
        }
        state.head<4>() /= length;
        return true;
    }

    /**
     * @brief 由四元数解算横滚/俯仰/航向并处理多圈航向累积
    */
    void QuaternionEkf::update_angles()
    {
        const float q0 = state_.q[0], q1 = state_.q[1];
        const float q2 = state_.q[2], q3 = state_.q[3];
        const float r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        const float r21 = 2.0f * (q1 * q2 + q0 * q3);
        const float r31 = 2.0f * (q1 * q3 - q0 * q2);
        const float r32 = 2.0f * (q2 * q3 + q0 * q1);
        const float r33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

        const float yaw = wrap_degrees(math::radians_to_degrees(math::atan2(r21, r11)));
        const float pitch = math::radians_to_degrees(math::asin_clamped(-r31));
        const float roll = wrap_degrees(math::radians_to_degrees(math::atan2(r32, r33)));
        const float alternate_yaw = wrap_degrees(yaw + 180.0f);
        const float alternate_pitch = pitch >= 0.0f ? 180.0f - pitch : -180.0f - pitch;
        const float alternate_roll = wrap_degrees(roll + 180.0f);

        bool use_alternate = false;
        float current_q[4]{q0, q1, q2, q3};
        if (have_previous_q_) {
            float dot = 0.0f;
            for (int axis = 0; axis < 4; ++axis) {
                dot += previous_q_[axis] * current_q[axis];
            }
            if (dot < 0.0f) {
                for (float &component : current_q) {
                    component = -component;
                }
                dot = -dot;
            }
            const float delta_y = previous_q_[0] * current_q[2] + previous_q_[1] * current_q[3] - previous_q_[2] * current_q[0] - previous_q_[3] * current_q[1];
            const float expected_pitch = wrap_degrees(state_.pitch + math::radians_to_degrees(2.0f * math::atan2(delta_y, dot)));
            const float direct_error = std::fabs(wrap_degrees(pitch - expected_pitch));
            const float alternate_error = std::fabs(wrap_degrees(alternate_pitch - expected_pitch));
            if (std::fabs(direct_error - alternate_error) > 0.05f) {
                use_alternate = alternate_error < direct_error;
            } else {
                const float direct_yaw = wrap_degrees(yaw - state_.yaw);
                const float direct_roll = wrap_degrees(roll - state_.roll);
                const float other_yaw = wrap_degrees(alternate_yaw - state_.yaw);
                const float other_roll = wrap_degrees(alternate_roll - state_.roll);
                use_alternate = other_yaw * other_yaw + other_roll * other_roll < direct_yaw * direct_yaw + direct_roll * direct_roll;
            }
        }
        state_.yaw = use_alternate ? alternate_yaw : yaw;
        state_.pitch = use_alternate ? alternate_pitch : pitch;
        state_.roll = use_alternate ? alternate_roll : roll;
        for (int axis = 0; axis < 4; ++axis) {
            previous_q_[axis] = current_q[axis];
        }
        have_previous_q_ = true;

        const float difference = state_.yaw - previous_yaw_;
        if (difference > 180.0f) {
            --yaw_turns_;
        }
        if (difference < -180.0f) {
            ++yaw_turns_;
        }
        state_.yaw_sum = 360.0f * static_cast<float>(yaw_turns_) + state_.yaw;
        previous_yaw_ = state_.yaw;
    }
}