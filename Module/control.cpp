#include "control.hpp"

/**
 * @brief 配置速度环 PID 参数
 *
 * @param config PID 配置参数
 */
void speed_loop::configure(const alg::pid_config &config)
{
    pid_.configure(config);
}

/**
 * @brief 复位速度环内部 PID 状态
 *
 */
void speed_loop::reset()
{
    pid_.reset();
}

/**
 * @brief 执行速度环一次更新 返回需要输出的控制量
 *
 * @param target_rad_s 目标角速度
 * @param measured_rad_s 实测角速度
 * @param dt_s 控制周期
 * @return 计算得到的控制量
 */
float speed_loop::update(float target_rad_s, float measured_rad_s, float dt_s)
{
    return pid_.update(target_rad_s, measured_rad_s, dt_s);
}

/**
 * @brief 配置位置-速度串级环的位置环与速度环 PID 参数
 *
 * @param position 位置环 PID 配置
 * @param speed 速度环 PID 配置
 */
void position_speed_loop::configure(const alg::pid_config &position, const alg::pid_config &speed)
{
    position_.configure(position);
    speed_.configure(speed);
}

/**
 * @brief 复位位置-速度串级环内部 PID 状态
 *
 */
void position_speed_loop::reset()
{
    position_.reset();
    speed_.reset();
}

/**
 * @brief 执行位置-速度串级环一次更新 返回力矩控制量
 *
 * @param target_rad 目标角度
 * @param measured_rad 实测角度
 * @param measured_rad_s 实测角速度
 * @param dt_s 控制周期
 * @return 计算得到的力矩控制量
 */
float position_speed_loop::update(float target_rad, float measured_rad, float measured_rad_s, float dt_s)
{
    const float target_speed = position_.update(target_rad, measured_rad, dt_s);
    return speed_.update(target_speed, measured_rad_s, dt_s);
}
