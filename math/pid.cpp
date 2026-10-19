#include "pid.hpp"

namespace alg
{
    namespace
    {

        float PositiveOr(float value, float fallback)
        {
            return std::isfinite(value) && value > 0.0f ? value : fallback;
        }
    }

    float Pid::ClampAbs(float value, float limit)
    {
        if (std::isfinite(limit) && limit > 0.0f) {
            value = std::clamp(value, -limit, limit);
        }
        return value;
    }

    float Pid::WrapAngle(float value)
    {
        return std::remainder(value, 2.0f * 3.14159265358979323846f);
    }

    void Pid::Configure(const PIDConfig &config)
    {
        config_ = config;
        config_.dt = PositiveOr(config_.dt, 0.001f);
        if (!std::isfinite(config_.derivative_filter_tau) || config_.derivative_filter_tau < 0.0f) {
            config_.derivative_filter_tau = 0.0f;
        }
        if (!std::isfinite(config_.dead_zone) || config_.dead_zone < 0.0f) {
            config_.dead_zone = 0.0f;
        }
        if (!std::isfinite(config_.variable_speed_a) || config_.variable_speed_a < 0.0f) {
            config_.variable_speed_a = 0.0f;
        }
        if (!std::isfinite(config_.variable_speed_b) || config_.variable_speed_b < config_.variable_speed_a) {
            config_.variable_speed_b = 0.0f;
        }
        if (!std::isfinite(config_.integral_separation) || config_.integral_separation < 0.0f) {
            config_.integral_separation = 0.0f;
        }
        Reset();
    }

    void Pid::Reset()
    {
        target_ = 0.0f;
        measurement_ = 0.0f;
        previous_target_ = 0.0f;
        previous_measurement_ = 0.0f;
        previous_error_ = 0.0f;
        integral_error_ = 0.0f;
        derivative_output_ = 0.0f;
        output_ = 0.0f;
        initialized_ = false;
    }

    float Pid::Calculate(float target, float measurement, float dt, bool angle)
    {
        if (!std::isfinite(target) || !std::isfinite(measurement)) {
            return output_;
        }
        dt = PositiveOr(dt, config_.dt);
        target_ = target;
        measurement_ = measurement;

        float error = angle ? WrapAngle(target - measurement) : target - measurement;
        const float absolute_error = std::fabs(error);
        if (config_.dead_zone > 0.0f) {
            if (absolute_error <= config_.dead_zone) {
                error = 0.0f;
            } else {
                error -= std::copysign(config_.dead_zone, error);
            }
        }

        const float p_output = config_.kp * error;
        float integral_rate = 1.0f;
        if (config_.variable_speed_b > config_.variable_speed_a && absolute_error > config_.variable_speed_a) {
            if (absolute_error >= config_.variable_speed_b) {
                integral_rate = 0.0f;
            } else {
                integral_rate = (config_.variable_speed_b - absolute_error) /
                                (config_.variable_speed_b - config_.variable_speed_a);
            }
        }
        if (config_.integral_separation > 0.0f && absolute_error >= config_.integral_separation) {
            integral_rate = 0.0f;
        }
        integral_error_ += integral_rate * error * dt;
        if (config_.ki != 0.0f && config_.integral_limit > 0.0f) {
            integral_error_ = ClampAbs(integral_error_, config_.integral_limit / std::fabs(config_.ki));
        }
        const float i_output = config_.ki * integral_error_;

        float d_raw = 0.0f;
        if (initialized_ && config_.kd != 0.0f) {
            if (config_.derivative_on_measurement == DFirst::Enable) {
                d_raw = -config_.kd * (measurement - previous_measurement_) / dt;
            } else {
                const float derivative_error = angle ? WrapAngle(error - previous_error_) : error - previous_error_;
                d_raw = config_.kd * derivative_error / dt;
            }
        }
        if (config_.derivative_filter_tau > 0.0f) {
            const float alpha = config_.derivative_filter_tau /
                                (config_.derivative_filter_tau + dt);
            derivative_output_ = alpha * derivative_output_ + (1.0f - alpha) * d_raw;
        } else {
            derivative_output_ = d_raw;
        }

        const float feedforward = initialized_ && config_.kf != 0.0f ? config_.kf * (target - previous_target_) / dt : 0.0f;
        const float unclamped_output = p_output + i_output + derivative_output_ + feedforward;
        output_ = ClampAbs(unclamped_output, config_.output_limit);
        if (output_ != unclamped_output && config_.ki != 0.0f && integral_rate > 0.0f) {
            const float without_integral = p_output + derivative_output_ + feedforward;
            const float required_integral = output_ - without_integral;
            integral_error_ = ClampAbs(required_integral / config_.ki,
                                       config_.integral_limit > 0.0f ? config_.integral_limit / std::fabs(config_.ki) : std::numeric_limits<float>::infinity());
        }

        previous_target_ = target;
        previous_measurement_ = measurement;
        previous_error_ = error;
        initialized_ = true;
        return output_;
    }

    float Pid::Update(float target, float measurement)
    {
        return Calculate(target, measurement, config_.dt, false);
    }

    float Pid::Update(float target, float measurement, float dt)
    {
        return Calculate(target, measurement, dt, false);
    }

    float Pid::UpdateAngle(float target, float measurement)
    {
        return Calculate(target, measurement, config_.dt, true);
    }

    float Pid::UpdateAngle(float target, float measurement, float dt)
    {
        return Calculate(target, measurement, dt, true);
    }

    void Pid::Init(float kp, float ki, float kd, float kf, float integral_limit, float output_limit,
                   float dt, float dead_zone, float variable_speed_a, float variable_speed_b,
                   float integral_separation, DFirst derivative_on_measurement, float derivative_filter_tau)
    {
        Configure(PIDConfig{kp, ki, kd, kf, integral_limit, output_limit, dt, dead_zone,
                            variable_speed_a, variable_speed_b, integral_separation, derivative_on_measurement,
                            derivative_filter_tau});
    }

    void Pid::SetKp(float kp) { config_.kp = kp; }

    void Pid::SetKi(float ki) { config_.ki = ki; }

    void Pid::SetKd(float kd) { config_.kd = kd; }

    void Pid::SetKf(float kf) { config_.kf = kf; }

    void Pid::SetIOutMax(float limit) { config_.integral_limit = std::fabs(limit); }

    void Pid::SetOutMax(float limit) { config_.output_limit = std::fabs(limit); }

    void Pid::SetIVariableSpeedA(float value) { config_.variable_speed_a = std::max(0.0f, value); }

    void Pid::SetIVariableSpeedB(float value) { config_.variable_speed_b = std::max(0.0f, value); }

    void Pid::SetISeparateThreshold(float value) { config_.integral_separation = std::max(0.0f, value); }

    void Pid::SetTarget(float target) { target_ = target; }

    void Pid::SetNow(float measurement) { measurement_ = measurement; }

    void Pid::SetIntegralError(float integral_error) { integral_error_ = integral_error; }

    void Pid::CalculatePeriodElapsedCallback()
    {
        Update(target_, measurement_);
    }

    void Pid::CalculateAnglePid()
    {
        UpdateAngle(target_, measurement_);
    }
}
