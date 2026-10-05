#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <cstdint>

namespace alg
{
    enum class DFirst : uint8_t
    {
        Disable = 0,
        Enable = 1,
    };

    constexpr DFirst PID_D_First_DISABLE = DFirst::Disable;
    constexpr DFirst PID_D_First_ENABLE = DFirst::Enable;

    struct PidConfig
    {
        float kp = 0.0F;
        float ki = 0.0F;
        float kd = 0.0F;
        float kf = 0.0F;
        float integral_limit = 0.0F;
        float output_limit = 0.0F;
        float dt = 0.001F;
        float dead_zone = 0.0F;
        float variable_speed_a = 0.0F;
        float variable_speed_b = 0.0F;
        float integral_separation = 0.0F;
        DFirst derivative_on_measurement = DFirst::Disable;
        float derivative_filter_tau = 0.0F;
    };

    class Pid final
    {
        public:
        Pid() = default;

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

        void Init(float kp, float ki, float kd, float kf = 0.0F, float integral_limit = 0.0F,

                  float output_limit = 0.0F, float dt = 0.001F, float dead_zone = 0.0F,

                  float variable_speed_a = 0.0F, float variable_speed_b = 0.0F,

                  float integral_separation = 0.0F, DFirst derivative_on_measurement = DFirst::Disable,

                  float derivative_filter_tau = 0.0F);

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

        float GetOut() const noexcept { return output_; }

        float GetIntegralError() const noexcept { return integral_error_; }

        float output() const noexcept { return output_; }

        const PidConfig &config() const noexcept { return config_; }

        private:
        float calculate(float target, float measurement, float dt, bool angle);

        static float clamp_abs(float value, float limit);

        static float wrap_angle(float value);

        PidConfig config_{};

        float target_ = 0.0F;

        float measurement_ = 0.0F;

        float previous_target_ = 0.0F;

        float previous_measurement_ = 0.0F;

        float previous_error_ = 0.0F;

        float integral_error_ = 0.0F;

        float derivative_output_ = 0.0F;

        float output_ = 0.0F;

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
