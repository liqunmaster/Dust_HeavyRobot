#pragma once

#include "eso.hpp"

namespace alg
{

    struct AdrcConfig
    {
        float b0 = 1.0F;
        float tracking_rate = 30.0F;
        float tracking_acceleration = 300.0F;
        float kp = 100.0F;
        float kd = 20.0F;
        float beta1 = 30.0F;
        float beta2 = 300.0F;
        float beta3 = 1000.0F;
        float alpha1 = 0.5F;
        float alpha2 = 0.25F;
        float fal_delta = 0.01F;
        float output_limit = 0.0F;
        bool nonlinear_observer = true;
    };

    class Adrc final
    {
        public:
        Adrc() = default;

        explicit Adrc(const AdrcConfig &config)
        {
            configure(config);
        }

        void configure(const AdrcConfig &config);
        void reset(float state = 0.0F);
        float update(float reference, float measurement, float dt);

        float output() const noexcept { return output_; }

        float tracking_state() const noexcept { return tracking_state_; }

        float tracking_derivative() const noexcept { return tracking_derivative_; }

        float estimated_state() const noexcept { return eso_.state(); }

        float estimated_derivative() const noexcept { return eso_.state_derivative(); }

        float estimated_disturbance() const noexcept { return eso_.disturbance(); }

        const AdrcConfig &config() const noexcept { return config_; }

        Eso &observer() noexcept { return eso_; }

        const Eso &observer() const noexcept { return eso_; }

        private:
        static float clamp_abs(float value, float limit);
        static float fal(float error, float alpha, float delta);

        AdrcConfig config_{};
        Eso eso_{};
        float tracking_state_ = 0.0F;
        float tracking_derivative_ = 0.0F;
        float output_ = 0.0F;
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
