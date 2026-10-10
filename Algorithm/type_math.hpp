#pragma once

#include <cmath>
#include <stdint.h>

#ifdef __cplusplus
namespace math
{

    constexpr float pi = 3.14159265358979323846f;
    constexpr float half_pi = pi / 2.0f;
    constexpr float two_pi = 2.0f * pi;
    constexpr float standard_gravity_mps2 = 9.80665f;
    constexpr float kelvin_offset = 273.15f;

    /**
     * @brief 度转弧度
     *
     * @param degrees 角度（度）
     * @return 弧度值
    */
    constexpr float degrees_to_radians(float degrees)
    {
        return degrees * (pi / 180.0f);
    }

    /**
     * @brief 弧度转度
     *
     * @param radians 角度（弧度）
     * @return 度数值
    */
    constexpr float radians_to_degrees(float radians)
    {
        return radians * (180.0f / pi);
    }

    /**
     * @brief 摄氏温度转开尔文
     *
     * @param celsius 摄氏温度（°C）
     * @return 开尔文温度
    */
    constexpr float celsius_to_kelvin(float celsius)
    {
        return celsius + kelvin_offset;
    }

    /**
     * @brief 开尔文温度转摄氏
     *
     * @param kelvin 开尔文温度
     * @return 摄氏温度（°C）
    */
    constexpr float kelvin_to_celsius(float kelvin)
    {
        return kelvin - kelvin_offset;
    }

    /**
     * @brief 加速度 g 数转米每秒平方
     *
     * @param acceleration_g 加速度（g）
     * @return 加速度（m/s²）
    */
    constexpr float g_to_mps2(float acceleration_g)
    {
        return acceleration_g * standard_gravity_mps2;
    }

    /**
     * @brief 米每秒平方加速度转 g 数
     *
     * @param acceleration_mps2 加速度（m/s²）
     * @return 加速度（g）
    */
    constexpr float mps2_to_g(float acceleration_mps2)
    {
        return acceleration_mps2 / standard_gravity_mps2;
    }

    /**
     * @brief 原始加速度计数值转 m/s²
     *
     * @param raw 原始 ADC/传感器数值
     * @param lsb_per_g 每 g 对应的 LSB 数（灵敏度）
     * @return 加速度（m/s²）
    */
    constexpr float accel_raw_to_mps2(int16_t raw, float lsb_per_g)
    {
        return g_to_mps2(static_cast<float>(raw) / lsb_per_g);
    }

    /**
     * @brief 原始陀螺仪数值转弧度每秒
     *
     * @param raw 原始 ADC/传感器数值
     * @param lsb_per_degree_per_second 每 °/s 对应的 LSB 数
     * @return 角速度（rad/s）
    */
    constexpr float gyro_raw_to_radians_per_second(int16_t raw, float lsb_per_degree_per_second)
    {
        return degrees_to_radians(static_cast<float>(raw) / lsb_per_degree_per_second);
    }

    /**
     * @brief 正弦函数封装
     *
     * @param angle 角度（弧度）
     * @return 正弦值
    */
    inline float sin(float angle) noexcept
    {
        return ::sinf(angle);
    }

    /**
     * @brief 余弦函数封装
     *
     * @param angle 角度（弧度）
     * @return 余弦值
    */
    inline float cos(float angle) noexcept
    {
        return ::cosf(angle);
    }

    /**
     * @brief 四象限反正切函数封装
     *
     * @param y 纵坐标
     * @param x 横坐标
     * @return 角度（弧度）
    */
    inline float atan2(float y, float x) noexcept
    {
        return ::atan2f(y, x);
    }

    /**
     * @brief 反正弦函数，输入自动裁剪到 [-1, 1]
     *
     * @param value 正弦值（自动限幅）
     * @return 角度（弧度）
    */
    inline float asin_clamped(float value) noexcept
    {
        if (value > 1.0f) {
            value = 1.0f;
        }
        if (value < -1.0f) {
            value = -1.0f;
        }
        return ::asinf(value);
    }

    /**
     * @brief 开平方封装
     *
     * @param value 输入值
     * @return 平方根
    */
    inline float sqrt(float value) noexcept
    {
        return __builtin_sqrtf(value);
    }

    /**
     * @brief 编码器原始计数值转圈数
     *
     * @param encoder 编码器原始计数
     * @param resolution 每圈分辨率
     * @param gear_ratio 减速比
     * @return 圈数
    */
    constexpr float encoder_to_turns(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0f)
    {
        return static_cast<float>(encoder) / (static_cast<float>(resolution) * gear_ratio);
    }

    /**
     * @brief 编码器原始计数值转弧度
     *
     * @param encoder 编码器原始计数
     * @param resolution 每圈分辨率
     * @param gear_ratio 减速比
     * @return 角度（弧度）
    */
    constexpr float encoder_to_radian(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0f)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * two_pi;
    }

    /**
     * @brief 编码器原始计数值转角度
     *
     * @param encoder 编码器原始计数
     * @param resolution 每圈分辨率
     * @param gear_ratio 减速比
     * @return 角度（度）
    */
    constexpr float encoder_to_degree(int64_t encoder, uint32_t resolution, float gear_ratio = 1.0f)
    {
        return encoder_to_turns(encoder, resolution, gear_ratio) * 360.0f;
    }

    /**
     * @brief RPM 转速转弧度每秒
     *
     * @param rpm 转速（rpm）
     * @param gear_ratio 减速比
     * @return 角速度（rad/s）
    */
    constexpr float rpm_to_radian_per_second(float rpm, float gear_ratio = 1.0f)
    {
        return rpm * two_pi / (60.0f * gear_ratio);
    }

    /**
     * @brief RPM 转速转度每秒
     *
     * @param rpm 转速（rpm）
     * @param gear_ratio 减速比
     * @return 角速度（°/s）
    */
    constexpr float rpm_to_degrees_per_second(float rpm, float gear_ratio = 1.0f)
    {
        return rpm * 6.0f / gear_ratio;
    }

    /**
     * @brief 电流原始数值按比例换算为实际电流
     *
     * @param raw 原始电流数值
     * @param raw_limit 电流原始上限
     * @param current_limit 对应实际电流上限
     * @return 电流（A）
    */
    constexpr float raw_to_current(int16_t raw, int16_t raw_limit, float current_limit)
    {
        return static_cast<float>(raw) * current_limit / static_cast<float>(raw_limit);
    }

    /**
     * @brief 实际电流换算为电流原始数值（先限幅）
     *
     * @param current 实际电流（A）
     * @param current_limit 实际电流上限
     * @param raw_limit 电流原始上限
     * @return 原始电流数值
    */
    constexpr int16_t current_to_raw(float current, float current_limit, int16_t raw_limit)
    {
        const float limited = current > current_limit ? current_limit : (current < -current_limit ? -current_limit : current);
        return static_cast<int16_t>(limited * static_cast<float>(raw_limit) / current_limit);
    }

    /**
     * @brief 快速倒数平方根（带有效性检查）
     *
     * @param x 输入值（须大于 0）
     * @return 1/sqrt(x)，输入无效时返回 0
    */
    static float invSqrt(float x)
    {
        if (!(x > 0.0f) || !std::isfinite(x)) {
            return 0.0f;
        }
        return 1.0f / math::sqrt(x);
    }

}
#endif