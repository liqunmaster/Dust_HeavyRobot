#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace alg
{
    // 扩张状态观测器（ESO）参数配置结构体
    struct EsoConfig
    {
        float beta1 = 30.0f;
        float beta2 = 300.0f;
        float beta3 = 1000.0f;
        float b0 = 1.0f;
        float alpha1 = 0.5f;
        float alpha2 = 0.25f;
        float delta = 0.01f;
        bool nonlinear = true;
    };

    // 扩张状态观测器（ESO）：估计状态、状态微分与总扰动
    class Eso final
    {
        public:
        Eso() = default;

        /**
         * @brief 根据配置构造 ESO 观测器
         *
         * @param config 配置结构体
        */
        explicit Eso(const EsoConfig &config)
        {
            configure(config);
        }

        void configure(const EsoConfig &config);

        void reset(float state = 0.0f);

        void update(float measurement, float input, float dt);

        /**
         * @brief 获取状态估计值 z1
         *
         * @return 状态估计
        */
        float state() const noexcept { return z1_; }

        /**
         * @brief 获取状态微分估计值 z2
         *
         * @return 微分估计
        */
        float state_derivative() const noexcept { return z2_; }

        /**
         * @brief 获取总扰动估计值 z3
         *
         * @return 扰动估计
        */
        float disturbance() const noexcept { return z3_; }

        /**
         * @brief 获取状态估计值 z1
         *
         * @return z1
        */
        float z1() const noexcept { return z1_; }

        /**
         * @brief 获取状态微分估计值 z2
         *
         * @return z2
        */
        float z2() const noexcept { return z2_; }

        /**
         * @brief 获取总扰动估计值 z3
         *
         * @return z3
        */
        float z3() const noexcept { return z3_; }

        /**
         * @brief 获取当前配置
         *
         * @return 配置结构体
        */
        const EsoConfig &config() const noexcept { return config_; }

        private:
        static float fal(float error, float alpha, float delta);

        EsoConfig config_{};
        float z1_ = 0.0f;
        float z2_ = 0.0f;
        float z3_ = 0.0f;
    };

    namespace control
    {
        using Eso = ::alg::Eso;
        using EsoConfig = ::alg::EsoConfig;
    }
}

using Eso = alg::Eso;
using EsoConfig = alg::EsoConfig;