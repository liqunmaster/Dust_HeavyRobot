#pragma once

#include "eso.hpp"

namespace alg
{

    // ADRC 参数配置结构体
    struct AdrcConfig
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

    // 自抗扰控制器（ADRC）：跟踪微分器 + 扩张状态观测器 + 非线性组合
    class Adrc final
    {
        public:
        Adrc() = default;

        /**
         * @brief 根据配置构造 ADRC 控制器
         *
         * @param config 配置结构体
        */
        explicit Adrc(const AdrcConfig &config)
        {
            configure(config);
        }

        void configure(const AdrcConfig &config);
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
        const AdrcConfig &config() const noexcept { return config_; }

        /**
         * @brief 获取内部 ESO 观测器引用（可读写）
         *
         * @return ESO 引用
        */
        Eso &observer() noexcept { return eso_; }

        /**
         * @brief 获取内部 ESO 观测器引用（只读）
         *
         * @return ESO 常量引用
        */
        const Eso &observer() const noexcept { return eso_; }

        private:
        static float clamp_abs(float value, float limit);
        static float fal(float error, float alpha, float delta);

        AdrcConfig config_{};
        Eso eso_{};
        float tracking_state_ = 0.0f;
        float tracking_derivative_ = 0.0f;
        float output_ = 0.0f;
        bool initialized_ = false;
    };

    using ADRC = Adrc;

    namespace control
    {
        using Adrc = ::alg::Adrc;
        using ADRC = ::alg::ADRC;
        using AdrcConfig = ::alg::AdrcConfig;
    }

}

using Adrc = alg::Adrc;
using ADRC = alg::ADRC;
using AdrcConfig = alg::AdrcConfig;