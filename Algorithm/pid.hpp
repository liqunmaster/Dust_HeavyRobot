#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <cstdint>

namespace alg
{
    // 微分作用取用方式的枚举
    enum class DFirst : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };

    constexpr DFirst PID_D_First_DISABLE = DFirst::Disable;
    constexpr DFirst PID_D_First_ENABLE = DFirst::Enable;

    // PID 参数配置结构体
    struct PidConfig
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float kf = 0.0f;
        float integral_limit = 0.0f;
        float output_limit = 0.0f;
        float dt = 0.001f;
        float dead_zone = 0.0f;
        float variable_speed_a = 0.0f;
        float variable_speed_b = 0.0f;
        float integral_separation = 0.0f;
        DFirst derivative_on_measurement = DFirst::Disable;
        float derivative_filter_tau = 0.0f;
    };

    // PID 控制器：支持变积分、积分分离、微分滤波、死区、前馈与角度环
    class Pid final
    {
        public:
        Pid() = default;

        /**
         * @brief 根据配置构造 PID 控制器
         *
         * @param config 配置结构体
        */
        explicit Pid(const PidConfig &config)
        {
            configure(config);
        }

        void configure(const PidConfig &config);

        void reset();

        float update(float target, float measurement);

        float update(float target, float measurement, float dt);

        float update_angle(float target, float measurement);

        float update_angle(float target, float measurement, float dt);

        void Init(float kp, float ki, float kd, float kf = 0.0f, float integral_limit = 0.0f,

                  float output_limit = 0.0f, float dt = 0.001f, float dead_zone = 0.0f,

                  float variable_speed_a = 0.0f, float variable_speed_b = 0.0f,

                  float integral_separation = 0.0f, DFirst derivative_on_measurement = DFirst::Disable,

                  float derivative_filter_tau = 0.0f);

        void SetKp(float kp);

        void SetKi(float ki);

        void SetKd(float kd);

        void SetKf(float kf);

        void SetIOutMax(float limit);

        void SetOutMax(float limit);

        void SetIVariableSpeedA(float value);

        void SetIVariableSpeedB(float value);

        void SetISeparateThreshold(float value);

        void SetTarget(float target);

        void SetNow(float measurement);

        void SetIntegralError(float integral_error);

        void CalculatePeriodElapsedCallback();

        void CalculateAnglePid();

        /**
         * @brief 获取 PID 输出
         *
         * @return 输出量
        */
        float GetOut() const noexcept { return output_; }

        /**
         * @brief 获取积分累计误差
         *
         * @return 积分误差
        */
        float GetIntegralError() const noexcept { return integral_error_; }

        /**
         * @brief 获取 PID 输出
         *
         * @return 输出量
        */
        float output() const noexcept { return output_; }

        /**
         * @brief 获取当前配置
         *
         * @return 配置结构体
        */
        const PidConfig &config() const noexcept { return config_; }

        private:
        float calculate(float target, float measurement, float dt, bool angle);

        static float clamp_abs(float value, float limit);

        static float wrap_angle(float value);

        PidConfig config_{};

        float target_ = 0.0f;

        float measurement_ = 0.0f;

        float previous_target_ = 0.0f;

        float previous_measurement_ = 0.0f;

        float previous_error_ = 0.0f;

        float integral_error_ = 0.0f;

        float derivative_output_ = 0.0f;

        float output_ = 0.0f;

        bool initialized_ = false;
    };

    using PID = Pid;

} // namespace alg

using Pid = alg::Pid;
using PID = alg::PID;
using PidConfig = alg::PidConfig;
using DFirst = alg::DFirst;
using alg::PID_D_First_DISABLE;
using alg::PID_D_First_ENABLE;