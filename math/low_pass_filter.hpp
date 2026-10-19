#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{

    class FirstOrderLpf final
    {
        public:

        void Init(float cutoff_hz, float sample_rate_hz)
        {
            if (cutoff_hz > 0.0f && sample_rate_hz > 0.0f) {
                const float rc = 1.0f / (math::two_pi * cutoff_hz);
                const float dt = 1.0f / sample_rate_hz;
                alpha_ = dt / (rc + dt);
            } else {
                alpha_ = 1.0f;
            }
            Reset();
        }

        float Update(float input)
        {
            if (!initialized_) {
                value_ = input;
                initialized_ = true;
            } else {
                value_ += alpha_ * (input - value_);
            }
            return value_;
        }

        void Reset()
        {
            value_ = 0.0f;
            initialized_ = false;
        }

        void Reset(float initial_value)
        {
            value_ = initial_value;
            initialized_ = true;
        }

        private:
        float alpha_ = 1.0f;

        float value_ = 0.0f;

        bool initialized_ = false;
    };

    class LowPassFilter final
    {
        public:

        void Init(float cutoff_hz, float sample_rate_hz, uint8_t order = 1)
        {
            order_ = order >= 1 && order <= max_order ? order : 1;
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Init(cutoff_hz, sample_rate_hz);
            }
        }

        float Update(float input)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                input = stages_[i].Update(input);
            }
            return input;
        }

        void Reset()
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Reset();
            }
        }

        void Reset(float initial_value)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].Reset(initial_value);
            }
        }

        private:
        static constexpr uint8_t max_order = 10;

        FirstOrderLpf stages_[max_order]{};

        uint8_t order_ = 1;
    };
}
