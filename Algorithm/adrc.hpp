#pragma once

#include "eso.hpp"

namespace alg
{
    // ADRC 参数配置结构体
    struct adrc_config
    {
        float b0 = 1.0f;
        float tracking_rate = 30.0f;
        float tracking_acceleration = 300.0f;
        float kp = 100.0f;
        float kd = 20.0f;
        float beta1 = 30.0f;
        float beta2 = 300.0f;
        float beta3 = 1000.0f;
        float alpha1 = 0.5f;
        float alpha2 = 0.25f;
        float fal_delta = 0.01f;
        float output_limit = 0.0f;
        bool nonlinear_observer = true;
    };

    // 自抗扰控制器：跟踪微分器 + 扩张状态观测器 + 非线性组合
    class adrc final
    {
        public:
        adrc() = default;

        /**
         * @brief 根据配置构造 ADRC 控制器
         *
         * @param config 配置结构体
         */
        explicit adrc(const adrc_config &config)
        {
            configure(config);
        }

        void configure(const adrc_config &config);
        void reset(float state = 0.0f);
        float update(float reference, float measurement, float dt);

        /**
         * @brief 获取当前控制输出
         *
         * @return 控制输出量
         */
        float output() const noexcept { return output_; }

        /**
         * @brief 获取跟踪状态
         *
         * @return 跟踪状态值
         */
        float tracking_state() const noexcept { return tracking_state_; }

        /**
         * @brief 获取跟踪状态的微分
         *
         * @return 跟踪微分值
         */
        float tracking_derivative() const noexcept { return tracking_derivative_; }

        /**
         * @brief 获取 ESO 估计的状态
         *
         * @return ESO 状态估计值
         */
        float estimated_state() const noexcept { return eso_.state(); }

        /**
         * @brief 获取 ESO 估计的状态微分
         *
         * @return ESO 微分估计值
         */
        float estimated_derivative() const noexcept { return eso_.state_derivative(); }

        /**
         * @brief 获取 ESO 估计的总扰动
         *
         * @return 扰动估计值
         */
        float estimated_disturbance() const noexcept { return eso_.disturbance(); }

        /**
         * @brief 获取当前配置
         *
         * @return 配置结构体
         */
        const adrc_config &config() const noexcept { return config_; }

        /**
         * @brief 获取内部 ESO 观测器引用
         *
         * @return ESO 引用
         */
        eso &observer() noexcept { return eso_; }

        /**
         * @brief 获取内部 ESO 观测器引用
         *
         * @return ESO 常量引用
         */
        const eso &observer() const noexcept { return eso_; }

        private:
        static float clamp_abs(float value, float limit);
        static float fal(float error, float alpha, float delta);

        adrc_config config_{};
        eso eso_{};
        float tracking_state_ = 0.0f;
        float tracking_derivative_ = 0.0f;
        float output_ = 0.0f;
        bool initialized_ = false;
    };

    using ADRC = adrc;

    namespace control
    {
        using adrc = ::alg::adrc;
        using ADRC = ::alg::ADRC;
        using adrc_config = ::alg::adrc_config;
    }

}

using adrc = alg::adrc;
using ADRC = alg::ADRC;
using adrc_config = alg::adrc_config;
