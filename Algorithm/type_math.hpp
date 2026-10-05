#pragma once

#include <cmath>
#include <stdint.h>

#ifdef __cplusplus
namespace math
{

    constexpr float pi = 3.14159265358979323846F;
    constexpr float half_pi = pi / 2.0F;
    constexpr float two_pi = 2.0F * pi;
    constexpr float standard_gravity_mps2 = 9.80665F;
    constexpr float kelvin_offset = 273.15F;

    constexpr float degrees_to_radians(float degrees)
    {
        return degrees * (pi / 180.0F);
    }

    constexpr float radians_to_degrees(float radians)
    {
        return radians * (180.0F / pi);
    }

    constexpr float celsius_to_kelvin(float celsius)
    {
        return celsius + kelvin_offset;
    }

    constexpr float kelvin_to_celsius(float kelvin)
    {
        return kelvin - kelvin_offset;
    }

    constexpr float g_to_mps2(float acceleration_g)
    {
        return acceleration_g * standard_gravity_mps2;
    }

    constexpr float mps2_to_g(float acceleration_mps2)
    {
        return acceleration_mps2 / standard_gravity_mps2;
    }

    constexpr float accel_raw_to_mps2(int16_t raw, float lsb_per_g)
    {
        return g_to_mps2(static_cast<float>(raw) / lsb_per_g);
    }

    constexpr float gyro_raw_to_radians_per_second(int16_t raw, float lsb_per_degree_per_second)
    {
        return degrees_to_radians(static_cast<float>(raw) / lsb_per_degree_per_second);
    }

    inline float sin(float angle) noexcept
    {
        return ::sinf(angle);
    }

    inline float cos(float angle) noexcept
    {
        return ::cosf(angle);
    }

    inline float atan2(float y, float x) noexcept
    {
        return ::atan2f(y, x);
    }

    inline float asin_clamped(float value) noexcept
    {
        if (value > 1.0F) {
            value = 1.0F;
        }
        if (value < -1.0F) {
            value = -1.0F;
        }
        return ::asinf(value);
    }

    inline float sqrt(float value) noexcept
    {
        return __builtin_sqrtf(value);
    }

    constexpr float encoder_to_turns(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return static_cast<float>(encoder) / (static_cast<float>(resolution) * gear_ratio);
    }

    constexpr float encoder_to_radian(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * two_pi;
    }

    constexpr float encoder_to_degree(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0F)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * 360.0F;
    }

    constexpr float rpm_to_radian_per_second(float rpm, float gear_ratio = 1.0F)
    {
        return rpm * two_pi / (60.0F * gear_ratio);
    }

    constexpr float rpm_to_degrees_per_second(float rpm, float gear_ratio = 1.0F)
    {
        return rpm * 6.0F / gear_ratio;
    }

    constexpr float raw_to_current(int16_t raw, int16_t raw_limit, float current_limit)
    {
        return static_cast<float>(raw) * current_limit / static_cast<float>(raw_limit);
    }

    constexpr int16_t current_to_raw(float current, float current_limit, int16_t raw_limit)
    {
        const float limited = current > current_limit ? current_limit : (current < -current_limit ? -current_limit : current);
        return static_cast<int16_t>(limited * static_cast<float>(raw_limit) / current_limit);
    }

    static float invSqrt(float x)
    {
        if (!(x > 0.0F) || !std::isfinite(x)) {
            return 0.0F;
        }
        return 1.0F / math::sqrt(x);
    }

}
#endif
