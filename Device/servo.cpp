#include "servo.hpp"

namespace
{
    constexpr uint16_t min_pulse_us = 1000;
    constexpr uint16_t center_pulse_us = 1500;
    constexpr uint16_t max_pulse_us = 2000;
    constexpr uint16_t max_angle_degrees = 150;
    constexpr uint32_t period_ns = PWM_MSEC(20);

    const device *const pwm_device = DEVICE_DT_GET(DT_NODELABEL(pwm1));
}

/**
 * @brief 初始化舵机并输出中位脉冲
 *
 * @return 成功返回 0 设备未就绪或参数非法返回负错误码
 */
int servo::init()
{
    if (initialized_) {
        return 0;
    }
    if (!device_is_ready(pwm_device)) {
        return -ENODEV;
    }
    if (output_ != servo_output::p2 && output_ != servo_output::p3) {
        return -EINVAL;
    }

    const int ret = pwm_set(pwm_device, static_cast<uint32_t>(output_), period_ns, PWM_USEC(center_pulse_us), PWM_POLARITY_NORMAL);
    if (ret == 0) {
        initialized_ = true;
    }
    return ret;
}

/**
 * @brief 设置舵机脉宽
 *
 * @param pulse_us 脉宽微秒数 需在有效范围内
 * @return 成功返回 0 越界返回 -EINVAL 未初始化返回 -ENODEV
 */
int servo::set_pulse_us(uint16_t pulse_us)
{
    if (!initialized_) {
        return -ENODEV;
    }
    if (pulse_us < min_pulse_us || pulse_us > max_pulse_us) {
        return -EINVAL;
    }

    return pwm_set(pwm_device, static_cast<uint32_t>(output_), period_ns, PWM_USEC(pulse_us), PWM_POLARITY_NORMAL);
}

/**
 * @brief 按角度设置舵机转角
 *
 * @param degrees 目标角度
 * @return 成功返回 0 越界返回 -EINVAL
 */
int servo::set_angle(uint16_t degrees)
{
    if (degrees > max_angle_degrees) {
        return -EINVAL;
    }

    const uint16_t pulse_us = min_pulse_us + static_cast<uint16_t>((static_cast<uint32_t>(degrees) * (max_pulse_us - min_pulse_us) + max_angle_degrees / 2) / max_angle_degrees);

    return set_pulse_us(pulse_us);
}
