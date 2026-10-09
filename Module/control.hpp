#pragma once

#include "pid.hpp"

class SpeedLoop
{
    public:
    void configure(const alg::PidConfig &config);
    void reset();
    float update(float target_rad_s, float measured_rad_s, float dt_s);

    private:
    alg::Pid pid_{};
};

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
