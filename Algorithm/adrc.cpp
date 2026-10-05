#include "adrc.hpp"

namespace alg
{

    namespace
    {
        float finite_or(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    float Adrc::clamp_abs(float value, float limit)
    {
        if (std::isfinite(limit) && limit > 0.0F) {
            return std::clamp(value, -limit, limit);
        }
        return value;
    }

    float Adrc::fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0F - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    void Adrc::configure(const AdrcConfig &config)
    {
        config_ = config;
        config_.b0 = finite_or(config_.b0, 1.0F);
        if (std::fabs(config_.b0) < 1.0e-6F) {
            config_.b0 = 1.0F;
        }
        config_.tracking_rate = std::max(0.0F, finite_or(config_.tracking_rate, 0.0F));
        config_.tracking_acceleration = std::max(0.0F, finite_or(config_.tracking_acceleration, 0.0F));
        config_.kp = finite_or(config_.kp, 0.0F);
        config_.kd = finite_or(config_.kd, 0.0F);

        EsoConfig eso_config;
        eso_config.beta1 = config_.beta1;
        eso_config.beta2 = config_.beta2;
        eso_config.beta3 = config_.beta3;
        eso_config.b0 = config_.b0;
        eso_config.alpha1 = config_.alpha1;
        eso_config.alpha2 = config_.alpha2;
        eso_config.delta = config_.fal_delta;
        eso_config.nonlinear = config_.nonlinear_observer;
        eso_.configure(eso_config);
        reset();
    }

    void Adrc::reset(float state)
    {
        tracking_state_ = finite_or(state, 0.0F);
        tracking_derivative_ = 0.0F;
        output_ = 0.0F;
        initialized_ = false;
        eso_.reset(tracking_state_);
    }

    float Adrc::update(float reference, float measurement, float dt)
    {
        if (!std::isfinite(reference) || !std::isfinite(measurement) || !std::isfinite(dt) || dt <= 0.0F) {
            return output_;
        }

        if (!initialized_) {
            tracking_state_ = reference;
            tracking_derivative_ = 0.0F;
            eso_.reset(measurement);
            initialized_ = true;
        } else {
            const float tracking_error = tracking_state_ - reference;
            const float max_acceleration = config_.tracking_acceleration;
            float acceleration = -config_.tracking_rate * tracking_error -
                                 2.0F * std::sqrt(std::max(0.0F, config_.tracking_rate)) * tracking_derivative_;
            if (max_acceleration > 0.0F) {
                acceleration = clamp_abs(acceleration, max_acceleration);
            }
            tracking_derivative_ += acceleration * dt;
            tracking_state_ += tracking_derivative_ * dt;
        }

        eso_.update(measurement, output_, dt);
        const float error = tracking_state_ - eso_.state();
        const float error_derivative = tracking_derivative_ - eso_.state_derivative();
        const float proportional = config_.nonlinear_observer ? config_.kp * fal(error, config_.alpha1, std::max(config_.fal_delta, 1.0e-6F)) : config_.kp * error;
        const float derivative = config_.nonlinear_observer ? config_.kd * fal(error_derivative, config_.alpha2, std::max(config_.fal_delta, 1.0e-6F)) : config_.kd * error_derivative;
        output_ = (proportional + derivative - eso_.disturbance()) / config_.b0;
        output_ = clamp_abs(output_, config_.output_limit);
        return output_;
    }

}
