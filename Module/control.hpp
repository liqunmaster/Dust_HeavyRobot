#pragma once

#include "pid.hpp"

// 单级速度环（封装一个 PID 速度控制器）
class SpeedLoop
{
    public:
    void configure(const alg::PidConfig &config);
    void reset();
    float update(float target_rad_s, float measured_rad_s, float dt_s);

    private:
    alg::Pid pid_{};
};

// 位置-速度串级控制环（内层 PID 为速度环）
class PositionSpeedLoop
{
    public:
    void configure(const alg::PidConfig &position, const alg::PidConfig &speed);
    void reset();
    float update(float target_rad, float measured_rad, float measured_rad_s, float dt_s);

    private:
    alg::Pid position_{};
    alg::Pid speed_{};
};
