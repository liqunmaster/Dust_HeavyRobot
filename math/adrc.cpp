#include "adrc.hpp"

namespace alg
{
    namespace
    {

        float FiniteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    float ADRC::ClampAbs(float value, float limit)
    {
        if (std::isfinite(limit) && limit > 0.0f) {
            return std::clamp(value, -limit, limit);
        }
        return value;
    }

    float ADRC::Fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0f - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    void ADRC::Configure(const ADRCConfig &config)
    {
        config_ = config;
        config_.b0 = FiniteOr(config_.b0, 1.0f);
        if (std::fabs(config_.b0) < 1.0e-6f) {
            config_.b0 = 1.0f;
        }
        config_.tracking_rate = std::max(0.0f, FiniteOr(config_.tracking_rate, 0.0f));
        config_.tracking_acceleration = std::max(0.0f, FiniteOr(config_.tracking_acceleration, 0.0f));
        config_.kp = FiniteOr(config_.kp, 0.0f);
        config_.kd = FiniteOr(config_.kd, 0.0f);

        ESOConfig eso_config;
        eso_config.beta1 = config_.beta1;
        eso_config.beta2 = config_.beta2;
        eso_config.beta3 = config_.beta3;
        eso_config.b0 = config_.b0;
        eso_config.alpha1 = config_.alpha1;
        eso_config.alpha2 = config_.alpha2;
        eso_config.delta = config_.fal_delta;
        eso_config.nonlinear = config_.nonlinear_observer;
        eso_.Configure(eso_config);
        Reset();
    }

    void ADRC::Reset(float state)
    {
        tracking_state_ = FiniteOr(state, 0.0f);
        tracking_derivative_ = 0.0f;
        output_ = 0.0f;
        initialized_ = false;
        eso_.Reset(tracking_state_);
    }

    float ADRC::Update(float reference, float measurement, float dt)
    {
        if (!std::isfinite(reference) || !std::isfinite(measurement) || !std::isfinite(dt) || dt <= 0.0f) {
            return output_;
        }

        if (!initialized_) {
            tracking_state_ = reference;
            tracking_derivative_ = 0.0f;
            eso_.Reset(measurement);
            initialized_ = true;
        } else {
            const float tracking_error = tracking_state_ - reference;
            const float max_acceleration = config_.tracking_acceleration;
            float acceleration = -config_.tracking_rate * tracking_error - 2.0f * std::sqrt(std::max(0.0f, config_.tracking_rate)) * tracking_derivative_;
            if (max_acceleration > 0.0f) {
                acceleration = ClampAbs(acceleration, max_acceleration);
            }
            tracking_derivative_ += acceleration * dt;
            tracking_state_ += tracking_derivative_ * dt;
        }

        eso_.Update(measurement, output_, dt);
        const float error = tracking_state_ - eso_.State();
        const float error_derivative = tracking_derivative_ - eso_.StateDerivative();
        const float proportional = config_.nonlinear_observer ? config_.kp * Fal(error, config_.alpha1, std::max(config_.fal_delta, 1.0e-6f)) : config_.kp * error;
        const float derivative = config_.nonlinear_observer ? config_.kd * Fal(error_derivative, config_.alpha2, std::max(config_.fal_delta, 1.0e-6f)) : config_.kd * error_derivative;
        output_ = (proportional + derivative - eso_.Disturbance()) / config_.b0;
        output_ = ClampAbs(output_, config_.output_limit);
        return output_;
    }
}
