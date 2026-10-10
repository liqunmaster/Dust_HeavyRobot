#include "adrc.hpp"

namespace alg
{

    namespace
    {
        /**
         * @brief 若数值有限则返回原值，否则返回备用值
         *
         * @param value 待判断的数值
         * @param fallback 数值无效时的备用值
         * @return 有效值或备用值
        */
        float finite_or(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    /**
     * @brief 将数值限幅在正负 limit 之间
     *
     * @param value 输入值
     * @param limit 限幅绝对值（小于等于 0 时不限幅）
     * @return 限幅后的值
    */
    float Adrc::clamp_abs(float value, float limit)
    {
        if (std::isfinite(limit) && limit > 0.0f) {
            return std::clamp(value, -limit, limit);
        }
        return value;
    }

    /**
     * @brief 非线性误差函数 fal
     *
     * @param error 输入误差
     * @param alpha 幂指数（0~1）
     * @param delta 线性区间宽度
     * @return fal 非线性输出
    */
    float Adrc::fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0f - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    /**
     * @brief 配置 ADRC 参数并同步配置扩张状态观测器（ESO）
     *
     * @param config 配置结构体
    */
    void Adrc::configure(const AdrcConfig &config)
    {
        config_ = config;
        config_.b0 = finite_or(config_.b0, 1.0f);
        if (std::fabs(config_.b0) < 1.0e-6f) {
            config_.b0 = 1.0f;
        }
        config_.tracking_rate = std::max(0.0f, finite_or(config_.tracking_rate, 0.0f));
        config_.tracking_acceleration = std::max(0.0f, finite_or(config_.tracking_acceleration, 0.0f));
        config_.kp = finite_or(config_.kp, 0.0f);
        config_.kd = finite_or(config_.kd, 0.0f);

        EsoConfig eso_config;
        eso_config.beta1 = config_.beta1;
        eso_config.beta2 = config_.beta2;
        eso_config.beta3 = config_.beta3;
        eso_config.b0 = config_.b0;
        eso_config.alpha1 = config_.alpha1;
        eso_config.alpha2 = config_.alpha2;
        eso_config.delta = config_.fal_delta;
        eso_config.nonlinear = config_.nonlinear_observer;
        eso_.configure(eso_config);
        reset();
    }

    /**
     * @brief 复位跟踪状态、输出与 ESO 状态
     *
     * @param state 初始跟踪状态值
    */
    void Adrc::reset(float state)
    {
        tracking_state_ = finite_or(state, 0.0f);
        tracking_derivative_ = 0.0f;
        output_ = 0.0f;
        initialized_ = false;
        eso_.reset(tracking_state_);
    }

    /**
     * @brief 执行一次 ADRC 控制周期：跟踪微分 + ESO 观测 + 组合律
     *
     * @param reference 给定参考值
     * @param measurement 被控量测量值
     * @param dt 采样周期（秒）
     * @return 控制输出量
    */
    float Adrc::update(float reference, float measurement, float dt)
    {
        if (!std::isfinite(reference) || !std::isfinite(measurement) || !std::isfinite(dt) || dt <= 0.0f) {
            return output_;
        }

        if (!initialized_) {
            tracking_state_ = reference;
            tracking_derivative_ = 0.0f;
            eso_.reset(measurement);
            initialized_ = true;
        } else {
            const float tracking_error = tracking_state_ - reference;
            const float max_acceleration = config_.tracking_acceleration;
            float acceleration = -config_.tracking_rate * tracking_error -
                                 2.0f * std::sqrt(std::max(0.0f, config_.tracking_rate)) * tracking_derivative_;
            if (max_acceleration > 0.0f) {
                acceleration = clamp_abs(acceleration, max_acceleration);
            }
            tracking_derivative_ += acceleration * dt;
            tracking_state_ += tracking_derivative_ * dt;
        }

        eso_.update(measurement, output_, dt);
        const float error = tracking_state_ - eso_.state();
        const float error_derivative = tracking_derivative_ - eso_.state_derivative();
        const float proportional = config_.nonlinear_observer ? config_.kp * fal(error, config_.alpha1, std::max(config_.fal_delta, 1.0e-6f)) : config_.kp * error;
        const float derivative = config_.nonlinear_observer ? config_.kd * fal(error_derivative, config_.alpha2, std::max(config_.fal_delta, 1.0e-6f)) : config_.kd * error_derivative;
        output_ = (proportional + derivative - eso_.disturbance()) / config_.b0;
        output_ = clamp_abs(output_, config_.output_limit);
        return output_;
    }

}