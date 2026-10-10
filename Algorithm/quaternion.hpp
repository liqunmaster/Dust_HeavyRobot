#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "kalman.hpp"
#include "type_math.hpp"

namespace alg::attitude
{

    // 一次 IMU 采样数据
    struct Sample
    {
        float gyro[3]{0.0f, 0.0f, 0.0f};
        float accel[3]{0.0f, 0.0f, 0.0f};
        float temp{0.0f};
        float dt{0.005f};
    };

    // 基于四元数扩展卡尔曼滤波的姿态估计器
    class QuaternionEkf final
    {
        public:
        // 姿态估计器的参数配置
        struct Config
        {
            float qq{10.0f};
            float qb{0.001f};
            float r{1000000.0f};
            float lambda{1.0f};
            float alpha{0.0f};
            float chi2_th{16.0f};
            uint32_t div_limit{50};
            float w_stable_th{0.3f};
            float a_ref{9.80665f};
            float a_tol{0.5f};
            float bias_limit{0.01f};
            float pb_limit{10000.0f};
        };

        // 姿态估计的运行状态
        struct State
        {
            bool init{false};
            bool converg{false};
            bool stable{false};
            uint64_t err_cnt{0};
            uint64_t upd_cnt{0};
            float q[4]{1.0f, 0.0f, 0.0f, 0.0f};
            float bg[3]{0.0f, 0.0f, 0.0f};
            float w[3]{0.0f, 0.0f, 0.0f};
            float a[3]{0.0f, 0.0f, 0.0f};
            float w_norm{0.0f};
            float a_norm{0.0f};
            float roll{0.0f};
            float pitch{0.0f};
            float yaw{0.0f};
            float yaw_sum{0.0f};
            float chi2{0.0f};
            float dt{0.0f};
        };

        void init(const Config &config);

        void init_from_accel(const Sample &sample);

        void update(const Sample &sample);

        /**
         * @brief 获取姿态估计的运行状态
         *
         * @return 状态结构体引用
        */
        const State &get_state() const { return state_; }

        private:
        using EKF = alg::filter::ExtendedKalman<6, 3, 4>;

        static void system_func(const EKF::State &state, const EKF::Ctrl &control, EKF::State &predicted, EKF::Cov &jacobian);

        static void observe_func(const EKF::State &state, EKF::Obs &predicted, EKF::ObsMat &jacobian);

        static bool normalize(EKF::State &state);

        void update_angles();

        Config config_{};

        State state_{};

        EKF ekf_{};

        uint16_t static_count_{0};

        float static_z_sum_{0.0f};

        uint16_t static_z_count_{0};

        float previous_yaw_{0.0f};

        int32_t yaw_turns_{0};

        float previous_q_[4]{1.0f, 0.0f, 0.0f, 0.0f};
        
        bool have_previous_q_{false};
    };

}