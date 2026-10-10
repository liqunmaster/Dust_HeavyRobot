#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{
    // 一阶高通滤波器（RC 型）
    class FirstOrderHpf final
    {
        public:
        /**
         * @brief 根据截止频率与采样率初始化滤波器
         *
         * @param cutoff_hz 截止频率（Hz）
         * @param sample_rate_hz 采样率（Hz）
        */
        void init(float cutoff_hz, float sample_rate_hz)
        {
            enabled_ = cutoff_hz > 0.0f && sample_rate_hz > 0.0f;
            if (enabled_) {
                const float rc = 1.0f / (math::two_pi * cutoff_hz);
                const float dt = 1.0f / sample_rate_hz;
                alpha_ = rc / (rc + dt);
            }
            reset();
        }

        /**
         * @brief 输入一个采样值并返回高通滤波结果
         *
         * @param input 输入采样值
         * @return 滤波后的值
        */
        float update(float input)
        {
            if (!enabled_) {
                return input;
            }
            if (!initialized_) {
                previous_input_ = input;
                initialized_ = true;
                return 0.0f;
            }
            value_ = alpha_ * (value_ + input - previous_input_);
            previous_input_ = input;
            return value_;
        }

        /**
         * @brief 复位滤波器输出为 0
        */
        void reset()
        {
            value_ = 0.0f;
            previous_input_ = 0.0f;
            initialized_ = false;
        }

        /**
         * @brief 复位滤波器并设置初始输入值
         *
         * @param initial_input 初始输入值
        */
        void reset(float initial_input)
        {
            value_ = 0.0f;
            previous_input_ = initial_input;
            initialized_ = true;
        }

        private:
        float alpha_ = 1.0f;

        float value_ = 0.0f;

        float previous_input_ = 0.0f;

        bool initialized_ = false;

        bool enabled_ = false;
    };

    // 多级串联高通滤波器（一阶节级联实现高阶滤波）
    class HighPassFilter final
    {
        public:
        /**
         * @brief 初始化滤波器阶数并初始化各级一阶节
         *
         * @param cutoff_hz 截止频率（Hz）
         * @param sample_rate_hz 采样率（Hz）
         * @param order 滤波器阶数（1~10，越界回退为 1）
        */
        void init(float cutoff_hz, float sample_rate_hz, uint8_t order = 1)
        {
            order_ = order >= 1 && order <= max_order ? order : 1;
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].init(cutoff_hz, sample_rate_hz);
            }
        }

        /**
         * @brief 输入一个采样值并返回高通滤波结果
         *
         * @param input 输入采样值
         * @return 滤波后的值
        */
        float update(float input)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                input = stages_[i].update(input);
            }
            return input;
        }

        /**
         * @brief 复位所有滤波级输出为 0
        */
        void reset()
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].reset();
            }
        }

        /**
         * @brief 复位所有滤波级并按初始输入刷新
         *
         * @param initial_input 初始输入值
        */
        void reset(float initial_input)
        {
            reset();
            update(initial_input);
        }

        private:
        static constexpr uint8_t max_order = 10;

        FirstOrderHpf stages_[max_order]{};

        uint8_t order_ = 1;
    };
}