#include "eso.hpp"

namespace alg
{
    namespace
    {

        float FiniteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    float ESO::Fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0f - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    void ESO::Configure(const ESOConfig &config)
    {
        config_ = config;
        config_.beta1 = std::max(0.0f, FiniteOr(config_.beta1, 0.0f));
        config_.beta2 = std::max(0.0f, FiniteOr(config_.beta2, 0.0f));
        config_.beta3 = std::max(0.0f, FiniteOr(config_.beta3, 0.0f));
        config_.b0 = FiniteOr(config_.b0, 1.0f);
        if (std::fabs(config_.b0) < 1.0e-6f) {
            config_.b0 = 1.0f;
        }
        config_.alpha1 = std::clamp(FiniteOr(config_.alpha1, 0.5f), 0.0f, 1.0f);
        config_.alpha2 = std::clamp(FiniteOr(config_.alpha2, 0.25f), 0.0f, 1.0f);
        config_.delta = std::max(1.0e-6f, FiniteOr(config_.delta, 0.01f));
        Reset();
    }

    void ESO::Reset(float state)
    {
        z1_ = FiniteOr(state, 0.0f);
        z2_ = 0.0f;
        z3_ = 0.0f;
    }

    void ESO::Update(float measurement, float input, float dt)
    {
        if (!std::isfinite(measurement) || !std::isfinite(input) || !std::isfinite(dt) || dt <= 0.0f) {
            return;
        }
        const float error = z1_ - measurement;
        const float feedback_1 = config_.nonlinear ? Fal(error, config_.alpha1, config_.delta) : error;
        const float feedback_2 = config_.nonlinear ? Fal(error, config_.alpha2, config_.delta) : error;
        const float dz1 = z2_ - config_.beta1 * error;
        const float dz2 = z3_ - config_.beta2 * feedback_1 + config_.b0 * input;
        const float dz3 = -config_.beta3 * feedback_2;

        z1_ += dt * dz1;
        z2_ += dt * dz2;
        z3_ += dt * dz3;
        if (!std::isfinite(z1_) || !std::isfinite(z2_) || !std::isfinite(z3_)) {
            Reset(measurement);
        }
    }
}
