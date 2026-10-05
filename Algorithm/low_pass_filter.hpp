#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{
    class FirstOrderLpf final
    {
        public:
        void init(float cutoff_hz, float sample_rate_hz)
        {
            if (cutoff_hz > 0.0F && sample_rate_hz > 0.0F) {
                const float rc = 1.0F / (math::two_pi * cutoff_hz);
                const float dt = 1.0F / sample_rate_hz;
                alpha_ = dt / (rc + dt);
            } else {
                alpha_ = 1.0F;
            }
            reset();
        }

        float update(float input)
        {
            if (!initialized_) {
                value_ = input;
                initialized_ = true;
            } else {
                value_ += alpha_ * (input - value_);
            }
            return value_;
        }

        void reset()
        {
            value_ = 0.0F;
            initialized_ = false;
        }

        void reset(float initial_value)
        {
            value_ = initial_value;
            initialized_ = true;
        }

        private:
        float alpha_ = 1.0F;

        float value_ = 0.0F;

        bool initialized_ = false;
    };

    class LowPassFilter final
    {
        public:
        void init(float cutoff_hz, float sample_rate_hz, uint8_t order = 1U)
        {
            order_ = order >= 1U && order <= max_order ? order : 1U;
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].init(cutoff_hz, sample_rate_hz);
            }
        }

        float update(float input)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                input = stages_[i].update(input);
            }
            return input;
        }

        void reset()
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].reset();
            }
        }

        void reset(float initial_value)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].reset(initial_value);
            }
        }

        private:
        static constexpr uint8_t max_order = 10U;

        FirstOrderLpf stages_[max_order]{};

        uint8_t order_ = 1U;
    };
}
