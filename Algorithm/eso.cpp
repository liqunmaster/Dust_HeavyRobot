#include "eso.hpp"

namespace alg
{

    namespace
    {
        float finite_or(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    } // namespace

    float Eso::fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0F - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    void Eso::configure(const EsoConfig &config)
    {
        config_ = config;
        config_.beta1 = std::max(0.0F, finite_or(config_.beta1, 0.0F));
        config_.beta2 = std::max(0.0F, finite_or(config_.beta2, 0.0F));
        config_.beta3 = std::max(0.0F, finite_or(config_.beta3, 0.0F));
        config_.b0 = finite_or(config_.b0, 1.0F);
        if (std::fabs(config_.b0) < 1.0e-6F) {
            config_.b0 = 1.0F;
        }
        config_.alpha1 = std::clamp(finite_or(config_.alpha1, 0.5F), 0.0F, 1.0F);
        config_.alpha2 = std::clamp(finite_or(config_.alpha2, 0.25F), 0.0F, 1.0F);
        config_.delta = std::max(1.0e-6F, finite_or(config_.delta, 0.01F));
        reset();
    }

    void Eso::reset(float state)
    {
        z1_ = finite_or(state, 0.0F);
        z2_ = 0.0F;
        z3_ = 0.0F;
    }

    void Eso::update(float measurement, float input, float dt)
    {
        if (!std::isfinite(measurement) || !std::isfinite(input) || !std::isfinite(dt) || dt <= 0.0F) {
            return;
        }
        const float error = z1_ - measurement;
        const float feedback_1 = config_.nonlinear ? fal(error, config_.alpha1, config_.delta) : error;
        const float feedback_2 = config_.nonlinear ? fal(error, config_.alpha2, config_.delta) : error;
        const float dz1 = z2_ - config_.beta1 * error;
        const float dz2 = z3_ - config_.beta2 * feedback_1 + config_.b0 * input;
        const float dz3 = -config_.beta3 * feedback_2;

        z1_ += dt * dz1;
        z2_ += dt * dz2;
        z3_ += dt * dz3;
        if (!std::isfinite(z1_) || !std::isfinite(z2_) || !std::isfinite(z3_)) {
            reset(measurement);
        }
    }

} // namespace alg
