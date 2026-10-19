#pragma once

#include "eso.hpp"

namespace alg
{

    struct ADRCConfig
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

    class ADRC final
    {
        public:
        ADRC() = default;

        explicit ADRC(const ADRCConfig &config)
        {
            Configure(config);
        }

        void Configure(const ADRCConfig &config);
        void Reset(float state = 0.0f);
        float Update(float reference, float measurement, float dt);

        float GetOutput() const noexcept { return output_; }

        float TrackingState() const noexcept { return tracking_state_; }

        float TrackingDerivative() const noexcept { return tracking_derivative_; }

        float EstimatedState() const noexcept { return eso_.State(); }

        float EstimatedDerivative() const noexcept { return eso_.StateDerivative(); }

        float EstimatedDisturbance() const noexcept { return eso_.Disturbance(); }

        const ADRCConfig &GetConfig() const noexcept { return config_; }

        ESO &Observer() noexcept { return eso_; }

        const ESO &Observer() const noexcept { return eso_; }

        private:
        static float ClampAbs(float value, float limit);
        static float Fal(float error, float alpha, float delta);

        ADRCConfig config_{};
        ESO eso_{};
        float tracking_state_ = 0.0f;
        float tracking_derivative_ = 0.0f;
        float output_ = 0.0f;
        bool initialized_ = false;
    };

    using ADRC = ADRC;

    namespace control
    {
        using ADRC = ::alg::ADRC;
        using ADRC = ::alg::ADRC;
        using ADRCConfig = ::alg::ADRCConfig;
    }
}

using ADRC = alg::ADRC;
using ADRC = alg::ADRC;
using ADRCConfig = alg::ADRCConfig;
