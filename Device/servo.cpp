#include "servo.hpp"

namespace
{
    constexpr uint16_t min_pulse_us = 1000U;
    constexpr uint16_t center_pulse_us = 1500U;
    constexpr uint16_t max_pulse_us = 2000U;
    constexpr uint16_t max_angle_degrees = 150U;
    constexpr uint32_t period_ns = PWM_MSEC(20U);

    const device *const pwm_device = DEVICE_DT_GET(DT_NODELABEL(pwm1));
}

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

int servo::set_angle(uint16_t degrees)
{
    if (degrees > max_angle_degrees) {
        return -EINVAL;
    }

    const uint16_t pulse_us = min_pulse_us + static_cast<uint16_t>((static_cast<uint32_t>(degrees) * (max_pulse_us - min_pulse_us) + max_angle_degrees / 2U) / max_angle_degrees);

    return set_pulse_us(pulse_us);
}
