#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace alg
{
    struct EsoConfig
    {
        float beta1 = 30.0F;
        float beta2 = 300.0F;
        float beta3 = 1000.0F;
        float b0 = 1.0F;
        float alpha1 = 0.5F;
        float alpha2 = 0.25F;
        float delta = 0.01F;
        bool nonlinear = true;
    };

    class Eso final
    {
        public:
        Eso() = default;

        explicit Eso(const EsoConfig &config)
        {
            configure(config);
        }

        void configure(const EsoConfig &config);

        void reset(float state = 0.0F);

        void update(float measurement, float input, float dt);

        float state() const noexcept { return z1_; }

        float state_derivative() const noexcept { return z2_; }

        float disturbance() const noexcept { return z3_; }

        float z1() const noexcept { return z1_; }

        float z2() const noexcept { return z2_; }

        float z3() const noexcept { return z3_; }

        const EsoConfig &config() const noexcept { return config_; }

        private:
        static float fal(float error, float alpha, float delta);

        EsoConfig config_{};
        float z1_ = 0.0F;
        float z2_ = 0.0F;
        float z3_ = 0.0F;
    };

    namespace control
    {
        using Eso = ::alg::Eso;
        using EsoConfig = ::alg::EsoConfig;
    }
}

using Eso = alg::Eso;
using EsoConfig = alg::EsoConfig;
