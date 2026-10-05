#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "kalman.hpp"
#include "type_math.hpp"

namespace alg::attitude
{

    struct Sample
    {
        float gyro[3]{0.0F, 0.0F, 0.0F};
        float accel[3]{0.0F, 0.0F, 0.0F};
        float temp{0.0F};
        float dt{0.005F};
    };

    class QuaternionEkf final
    {
        public:
        struct Config
        {
            float qq{10.0F};
            float qb{0.001F};
            float r{1000000.0F};
            float lambda{1.0F};
            float alpha{0.0F};
            float chi2_th{16.0F};
            uint32_t div_limit{50};
            float w_stable_th{0.3F};
            float a_ref{9.80665F};
            float a_tol{0.5F};
            float bias_limit{0.01F};
            float pb_limit{10000.0F};
        };

        struct State
        {
            bool init{false};
            bool converg{false};
            bool stable{false};
            uint64_t err_cnt{0};
            uint64_t upd_cnt{0};
            float q[4]{1.0F, 0.0F, 0.0F, 0.0F};
            float bg[3]{0.0F, 0.0F, 0.0F};
            float w[3]{0.0F, 0.0F, 0.0F};
            float a[3]{0.0F, 0.0F, 0.0F};
            float w_norm{0.0F};
            float a_norm{0.0F};
            float roll{0.0F};
            float pitch{0.0F};
            float yaw{0.0F};
            float yaw_sum{0.0F};
            float chi2{0.0F};
            float dt{0.0F};
        };

        void init(const Config &config);

        void init_from_accel(const Sample &sample);

        void update(const Sample &sample);

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

        float static_z_sum_{0.0F};

        uint16_t static_z_count_{0};

        float previous_yaw_{0.0F};

        int32_t yaw_turns_{0};

        float previous_q_[4]{1.0F, 0.0F, 0.0F, 0.0F};
        
        bool have_previous_q_{false};
    };

}
