#pragma once

#include <stdint.h>

#include "type_math.hpp"

namespace alg::filter
{
    // 一阶低通滤波器（RC 型）
    class FirstOrderLpf final
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
            if (cutoff_hz > 0.0f && sample_rate_hz > 0.0f) {
                const float rc = 1.0f / (math::two_pi * cutoff_hz);
                const float dt = 1.0f / sample_rate_hz;
                alpha_ = dt / (rc + dt);
            } else {
                alpha_ = 1.0f;
            }
            reset();
        }

        /**
         * @brief 输入一个采样值并返回低通滤波结果
         *
         * @param input 输入采样值
         * @return 滤波后的值
        */
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

        /**
         * @brief 复位滤波器输出为 0
        */
        void reset()
        {
            value_ = 0.0f;
            initialized_ = false;
        }

        /**
         * @brief 复位滤波器并设置初始输出值
         *
         * @param initial_value 初始输出值
        */
        void reset(float initial_value)
        {
            value_ = initial_value;
            initialized_ = true;
        }

        private:
        float alpha_ = 1.0f;

        float value_ = 0.0f;

        bool initialized_ = false;
    };

    // 多级串联低通滤波器（一阶节级联实现高阶滤波）
    class LowPassFilter final
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
         * @brief 输入一个采样值并返回低通滤波结果
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
         * @brief 复位所有滤波级并设置初始输出值
         *
         * @param initial_value 初始输出值
        */
        void reset(float initial_value)
        {
            for (uint8_t i = 0; i < order_; ++i) {
                stages_[i].reset(initial_value);
            }
        }

        private:
        static constexpr uint8_t max_order = 10;

        FirstOrderLpf stages_[max_order]{};

        uint8_t order_ = 1;
    };
}