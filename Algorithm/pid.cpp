#include "pid.hpp"

namespace alg
{

    namespace
    {
        /**
         * @brief 若数值为正且有限则返回原值，否则返回备用值
         *
         * @param value 待判断的数值
         * @param fallback 数值无效时的备用值
         * @return 有效值或备用值
        */
        float positive_or(float value, float fallback)
        {
            return std::isfinite(value) && value > 0.0f ? value : fallback;
        }
    }

    /**
     * @brief 将数值限幅在正负 limit 之间
     *
     * @param value 输入值
     * @param limit 限幅绝对值（小于等于 0 时不限幅）
     * @return 限幅后的值
    */
    float Pid::clamp_abs(float value, float limit)
    {
        if (std::isfinite(limit) && limit > 0.0f) {
            value = std::clamp(value, -limit, limit);
        }
        return value;
    }

    /**
     * @brief 将角度差折叠到 [-180°, 180°]（弧度制）
     *
     * @param value 待折叠的角度（弧度）
     * @return 折叠后的角度（弧度）
    */
    float Pid::wrap_angle(float value)
    {
        return std::remainder(value, 2.0f * 3.14159265358979323846f);
    }

    /**
     * @brief 配置 PID 参数并复位内部状态
     *
     * @param config 配置结构体
    */
    void Pid::configure(const PidConfig &config)
    {
        config_ = config;
        config_.dt = positive_or(config_.dt, 0.001f);
        if (!std::isfinite(config_.derivative_filter_tau) || config_.derivative_filter_tau < 0.0f) {
            config_.derivative_filter_tau = 0.0f;
        }
        if (!std::isfinite(config_.dead_zone) || config_.dead_zone < 0.0f) {
            config_.dead_zone = 0.0f;
        }
        if (!std::isfinite(config_.variable_speed_a) || config_.variable_speed_a < 0.0f) {
            config_.variable_speed_a = 0.0f;
        }
        if (!std::isfinite(config_.variable_speed_b) || config_.variable_speed_b < config_.variable_speed_a) {
            config_.variable_speed_b = 0.0f;
        }
        if (!std::isfinite(config_.integral_separation) || config_.integral_separation < 0.0f) {
            config_.integral_separation = 0.0f;
        }
        reset();
    }

    /**
     * @brief 复位 PID 内部全部状态变量
    */
    void Pid::reset()
    {
        target_ = 0.0f;
        measurement_ = 0.0f;
        previous_target_ = 0.0f;
        previous_measurement_ = 0.0f;
        previous_error_ = 0.0f;
        integral_error_ = 0.0f;
        derivative_output_ = 0.0f;
        output_ = 0.0f;
        initialized_ = false;
    }

    /**
     * @brief 执行一次 PID 核心计算（含死区、变积分、积分分离、微分滤波等）
     *
     * @param target 目标值
     * @param measurement 测量值
     * @param dt 采样周期（秒）
     * @param angle 是否为角度环（按角度差折叠计算误差）
     * @return PID 输出量
    */
    float Pid::calculate(float target, float measurement, float dt, bool angle)
    {
        if (!std::isfinite(target) || !std::isfinite(measurement)) {
            return output_;
        }
        dt = positive_or(dt, config_.dt);
        target_ = target;
        measurement_ = measurement;

        float error = angle ? wrap_angle(target - measurement) : target - measurement;
        const float absolute_error = std::fabs(error);
        if (config_.dead_zone > 0.0f) {
            if (absolute_error <= config_.dead_zone) {
                error = 0.0f;
            } else {
                error -= std::copysign(config_.dead_zone, error);
            }
        }

        const float p_output = config_.kp * error;
        float integral_rate = 1.0f;
        if (config_.variable_speed_b > config_.variable_speed_a && absolute_error > config_.variable_speed_a) {
            if (absolute_error >= config_.variable_speed_b) {
                integral_rate = 0.0f;
            } else {
                integral_rate = (config_.variable_speed_b - absolute_error) /
                                (config_.variable_speed_b - config_.variable_speed_a);
            }
        }
        if (config_.integral_separation > 0.0f && absolute_error >= config_.integral_separation) {
            integral_rate = 0.0f;
        }
        integral_error_ += integral_rate * error * dt;
        if (config_.ki != 0.0f && config_.integral_limit > 0.0f) {
            integral_error_ = clamp_abs(integral_error_, config_.integral_limit / std::fabs(config_.ki));
        }
        const float i_output = config_.ki * integral_error_;

        float d_raw = 0.0f;
        if (initialized_ && config_.kd != 0.0f) {
            if (config_.derivative_on_measurement == DFirst::Enable) {
                d_raw = -config_.kd * (measurement - previous_measurement_) / dt;
            } else {
                const float derivative_error = angle ? wrap_angle(error - previous_error_) : error - previous_error_;
                d_raw = config_.kd * derivative_error / dt;
            }
        }
        if (config_.derivative_filter_tau > 0.0f) {
            const float alpha = config_.derivative_filter_tau /
                                (config_.derivative_filter_tau + dt);
            derivative_output_ = alpha * derivative_output_ + (1.0f - alpha) * d_raw;
        } else {
            derivative_output_ = d_raw;
        }

        const float feedforward = initialized_ && config_.kf != 0.0f ?
            config_.kf * (target - previous_target_) / dt : 0.0f;
        const float unclamped_output = p_output + i_output + derivative_output_ + feedforward;
        output_ = clamp_abs(unclamped_output, config_.output_limit);
        if (output_ != unclamped_output && config_.ki != 0.0f && integral_rate > 0.0f) {
            const float without_integral = p_output + derivative_output_ + feedforward;
            const float required_integral = output_ - without_integral;
            integral_error_ = clamp_abs(required_integral / config_.ki,
                                        config_.integral_limit > 0.0f ? config_.integral_limit / std::fabs(config_.ki) : std::numeric_limits<float>::infinity());
        }

        previous_target_ = target;
        previous_measurement_ = measurement;
        previous_error_ = error;
        initialized_ = true;
        return output_;
    }

    /**
     * @brief 使用配置默认采样周期执行一次 PID 更新（非角度环）
     *
     * @param target 目标值
     * @param measurement 测量值
     * @return PID 输出量
    */
    float Pid::update(float target, float measurement)
    {
        return calculate(target, measurement, config_.dt, false);
    }

    /**
     * @brief 使用指定采样周期执行一次 PID 更新（非角度环）
     *
     * @param target 目标值
     * @param measurement 测量值
     * @param dt 采样周期（秒）
     * @return PID 输出量
    */
    float Pid::update(float target, float measurement, float dt)
    {
        return calculate(target, measurement, dt, false);
    }

    /**
     * @brief 使用配置默认采样周期执行一次 PID 更新（角度环）
     *
     * @param target 目标角度
     * @param measurement 测量角度
     * @return PID 输出量
    */
    float Pid::update_angle(float target, float measurement)
    {
        return calculate(target, measurement, config_.dt, true);
    }

    /**
     * @brief 使用指定采样周期执行一次 PID 更新（角度环）
     *
     * @param target 目标角度
     * @param measurement 测量角度
     * @param dt 采样周期（秒）
     * @return PID 输出量
    */
    float Pid::update_angle(float target, float measurement, float dt)
    {
        return calculate(target, measurement, dt, true);
    }

    /**
     * @brief 一次性配置 PID 全部参数（兼容旧式初始化接口）
     *
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param kf 前馈系数
     * @param integral_limit 积分限幅
     * @param output_limit 输出限幅
     * @param dt 采样周期（秒）
     * @param dead_zone 死区
     * @param variable_speed_a 变积分区间下限
     * @param variable_speed_b 变积分区间上限
     * @param integral_separation 积分分离阈值
     * @param derivative_on_measurement 微分作用于测量值的开关
     * @param derivative_filter_tau 微分滤波时间常数
    */
    void Pid::Init(float kp, float ki, float kd, float kf, float integral_limit, float output_limit,
                   float dt, float dead_zone, float variable_speed_a, float variable_speed_b,
                   float integral_separation, DFirst derivative_on_measurement, float derivative_filter_tau)
    {
        configure(PidConfig{kp, ki, kd, kf, integral_limit, output_limit, dt, dead_zone,
                            variable_speed_a, variable_speed_b, integral_separation, derivative_on_measurement,
                            derivative_filter_tau});
    }

    /**
     * @brief 设置比例系数 kp
     *
     * @param kp 比例系数
    */
    void Pid::SetKp(float kp) { config_.kp = kp; }

    /**
     * @brief 设置积分系数 ki
     *
     * @param ki 积分系数
    */
    void Pid::SetKi(float ki) { config_.ki = ki; }

    /**
     * @brief 设置微分系数 kd
     *
     * @param kd 微分系数
    */
    void Pid::SetKd(float kd) { config_.kd = kd; }

    /**
     * @brief 设置前馈系数 kf
     *
     * @param kf 前馈系数
    */
    void Pid::SetKf(float kf) { config_.kf = kf; }

    /**
     * @brief 设置积分限幅
     *
     * @param limit 积分限幅绝对值
    */
    void Pid::SetIOutMax(float limit) { config_.integral_limit = std::fabs(limit); }

    /**
     * @brief 设置输出限幅
     *
     * @param limit 输出限幅绝对值
    */
    void Pid::SetOutMax(float limit) { config_.output_limit = std::fabs(limit); }

    /**
     * @brief 设置变积分的开关区间下限
     *
     * @param value 区间下限
    */
    void Pid::SetIVariableSpeedA(float value) { config_.variable_speed_a = std::max(0.0f, value); }

    /**
     * @brief 设置变积分的开关区间上限
     *
     * @param value 区间上限
    */
    void Pid::SetIVariableSpeedB(float value) { config_.variable_speed_b = std::max(0.0f, value); }

    /**
     * @brief 设置积分分离阈值
     *
     * @param value 分离阈值
    */
    void Pid::SetISeparateThreshold(float value) { config_.integral_separation = std::max(0.0f, value); }

    /**
     * @brief 设置目标值
     *
     * @param target 目标值
    */
    void Pid::SetTarget(float target) { target_ = target; }

    /**
     * @brief 设置当前测量值
     *
     * @param measurement 测量值
    */
    void Pid::SetNow(float measurement) { measurement_ = measurement; }

    /**
     * @brief 直接设置积分累计误差
     *
     * @param integral_error 积分误差值
    */
    void Pid::SetIntegralError(float integral_error) { integral_error_ = integral_error; }

    /**
     * @brief 使用已保存的目标与测量值执行一次默认周期 PID 更新（回调用）
    */
    void Pid::CalculatePeriodElapsedCallback()
    {
        update(target_, measurement_);
    }

    /**
     * @brief 使用已保存的目标与测量值执行一次默认周期角度环 PID 更新
    */
    void Pid::CalculateAnglePid()
    {
        update_angle(target_, measurement_);
    }

}