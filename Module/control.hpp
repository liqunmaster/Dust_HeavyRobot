#pragma once

#include "pid.hpp"

// 单级速度环
class speed_loop
{
    public:
    void configure(const alg::pid_config &config);
    void reset();
    float update(float target_rad_s, float measured_rad_s, float dt_s);

    private:
    alg::pid pid_{};
};

// 位置-速度串级控制环
class position_speed_loop
{
    public:
    void configure(const alg::pid_config &position, const alg::pid_config &speed);
    void reset();
    float update(float target_rad, float measured_rad, float measured_rad_s, float dt_s);

    private:
    alg::pid position_{};
    alg::pid speed_{};
};
