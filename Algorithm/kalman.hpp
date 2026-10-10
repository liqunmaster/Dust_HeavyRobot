#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>

// 单轴一维卡尔曼滤波器
class kalman_axis final
{
    public:
    kalman_axis() = default;

    kalman_axis(float process_noise, float measurement_noise);

    void configure(float process_noise, float measurement_noise);

    void reset(float value = 0.0f);

    float update(float measurement);

    private:
    float process_noise_{0.001f};

    float measurement_noise_{0.05f};

    float estimate_{0.0f};

    float covariance_{1.0f};

    bool initialized_{false};
};

// 三轴卡尔曼滤波器
class kalman_vector3 final
{
    public:
    kalman_vector3() = default;

    kalman_vector3(float process_noise, float measurement_noise);

    void configure(float process_noise, float measurement_noise);

    void reset(const float value[3] = nullptr);

    void update(const float measurement[3], float filtered[3]);

    private:
    kalman_axis axis_[3];
};

namespace alg::filter
{

    // 模板化扩展卡尔曼滤波器：NX 状态维、NZ 观测维、NU 控制维
    template <int NX, int NZ, int NU>
    class extended_kalman final
    {
        public:
        using State = Eigen::Matrix<float, NX, 1>;

        using Cov = Eigen::Matrix<float, NX, NX>;

        using Obs = Eigen::Matrix<float, NZ, 1>;

        using ObsCov = Eigen::Matrix<float, NZ, NZ>;

        using Ctrl = Eigen::Matrix<float, NU, 1>;

        using ObsMat = Eigen::Matrix<float, NZ, NX>;

        using SystemFunc = void (*)(const State &, const Ctrl &, State &, Cov &);
        
        using ObserveFunc = void (*)(const State &, Obs &, ObsMat &);

        void init(const State &initial_state, const Cov &initial_covariance);

        /**
         * @brief 注册系统模型函数
         *
         * @param function 系统函数指针
         */
        void set_system_func(SystemFunc function)
        {
            system_func_ = function;
        }

        /**
         * @brief 注册观测模型函数
         *
         * @param function 观测函数指针
         */
        void set_observe_func(ObserveFunc function)
        {
            observe_func_ = function;
        }

        /**
         * @brief 直接设置状态并同步预测状态
         *
         * @param state 状态向量
         */
        void set_state(const State &state)
        {
            state_ = state;
            predicted_state_ = state;
        }

        /**
         * @brief 直接设置协方差并同步预测协方差
         *
         * @param covariance 协方差矩阵
         */
        void set_covariance(const Cov &covariance)
        {
            covariance_ = covariance;
            predicted_covariance_ = covariance;
        }

        void set_gain_row_scale(int row, float scale);
        void set_correction_limit(int row, float limit);

        bool predict(const Ctrl &control, const Cov &process_noise);

        bool update(const Obs &measurement, const ObsCov &measurement_noise, float max_chi2 = std::numeric_limits<float>::infinity(), float gain_scale = 1.0f);

        void use_prediction();

        void fade_predicted_variance(int index, float lambda, float max_variance);

        /**
         * @brief 获取当前状态向量估计
         *
         * @return 状态向量
         */
        const State &get_x() const
        {
            return state_;
        }

        /**
         * @brief 获取当前协方差矩阵
         *
         * @return 协方差矩阵
         */
        const Cov &get_p() const
        {
            return covariance_;
        }

        /**
         * @brief 获取最近一次观测更新的卡方值
         *
         * @return 卡方归一化新息
         */
        float get_chi2() const
        {
            return chi2_;
        }

        private:
        SystemFunc system_func_{nullptr};

        ObserveFunc observe_func_{nullptr};

        State state_{State::Zero()};

        State predicted_state_{State::Zero()};

        Cov covariance_{Cov::Identity()};

        Cov predicted_covariance_{Cov::Identity()};

        State gain_row_scale_{State::Ones()};

        State correction_limit_{State::Constant(std::numeric_limits<float>::infinity())};

        float chi2_{0.0f};
    };

    extern template class extended_kalman<6, 3, 4>;

}
