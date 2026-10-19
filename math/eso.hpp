#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace alg
{

    struct ESOConfig
    {
        float beta1 = 30.0f;
        float beta2 = 300.0f;
        float beta3 = 1000.0f;
        float b0 = 1.0f;
        float alpha1 = 0.5f;
        float alpha2 = 0.25f;
        float delta = 0.01f;
        bool nonlinear = true;
    };

    class ESO final
    {
        public:
        ESO() = default;

        explicit ESO(const ESOConfig &config)
        {
            Configure(config);
        }

        void Configure(const ESOConfig &config);

        void Reset(float state = 0.0f);

        void Update(float measurement, float input, float dt);

        float State() const noexcept { return z1_; }

        float StateDerivative() const noexcept { return z2_; }

        float Disturbance() const noexcept { return z3_; }

        float Z1() const noexcept { return z1_; }

        float Z2() const noexcept { return z2_; }

        float Z3() const noexcept { return z3_; }

        const ESOConfig &GetConfig() const noexcept { return config_; }

        private:
        static float Fal(float error, float alpha, float delta);

        ESOConfig config_{};
        float z1_ = 0.0f;
        float z2_ = 0.0f;
        float z3_ = 0.0f;
    };

    namespace control
    {
        using ESO = ::alg::ESO;
        using ESOConfig = ::alg::ESOConfig;
    }
}

using ESO = alg::ESO;
using ESOConfig = alg::ESOConfig;
