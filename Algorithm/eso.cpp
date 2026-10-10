#include "eso.hpp"

namespace alg
{

    namespace
    {
        /**
         * @brief 若数值有限则返回原值 否则返回备用值
         *
         * @param value 待判断的数值
         * @param fallback 数值无效时的备用值
         * @return 有效值或备用值
         */
        float finite_or(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    } // namespace

    /**
     * @brief 非线性误差函数 fal
     *
     * @param error 输入误差
     * @param alpha 幂指数
     * @param delta 线性区间宽度
     * @return fal 非线性输出
     */
    float eso::fal(float error, float alpha, float delta)
    {
        const float magnitude = std::fabs(error);
        if (magnitude <= delta) {
            return error / std::pow(delta, 1.0f - alpha);
        }
        return std::copysign(std::pow(magnitude, alpha), error);
    }

    /**
     * @brief 配置 ESO 参数并复位内部状态
     *
     * @param config 配置结构体
     */
    void eso::configure(const eso_config &config)
    {
        config_ = config;
        config_.beta1 = std::max(0.0f, finite_or(config_.beta1, 0.0f));
        config_.beta2 = std::max(0.0f, finite_or(config_.beta2, 0.0f));
        config_.beta3 = std::max(0.0f, finite_or(config_.beta3, 0.0f));
        config_.b0 = finite_or(config_.b0, 1.0f);
        if (std::fabs(config_.b0) < 1.0e-6f) {
            config_.b0 = 1.0f;
        }
        config_.alpha1 = std::clamp(finite_or(config_.alpha1, 0.5f), 0.0f, 1.0f);
        config_.alpha2 = std::clamp(finite_or(config_.alpha2, 0.25f), 0.0f, 1.0f);
        config_.delta = std::max(1.0e-6f, finite_or(config_.delta, 0.01f));
        reset();
    }

    /**
     * @brief 复位 ESO 状态变量
     *
     * @param state 初始状态估计值
     */
    void eso::reset(float state)
    {
        z1_ = finite_or(state, 0.0f);
        z2_ = 0.0f;
        z3_ = 0.0f;
    }

    /**
     * @brief 执行一次 ESO 状态观测更新
     *
     * @param measurement 被控对象测量输出
     * @param input 控制输入量
     * @param dt 采样周期
     */
    void eso::update(float measurement, float input, float dt)
    {
        if (!std::isfinite(measurement) || !std::isfinite(input) || !std::isfinite(dt) || dt <= 0.0f) {
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

}
