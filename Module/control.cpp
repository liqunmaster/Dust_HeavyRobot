#include "control.hpp"

void SpeedLoop::configure(const alg::PidConfig &config)
{
    pid_.configure(config);
}

void SpeedLoop::reset()
{
    pid_.reset();
}

float SpeedLoop::update(float target_rad_s, float measured_rad_s, float dt_s)
{
    return pid_.update(target_rad_s, measured_rad_s, dt_s);
}

void PositionSpeedLoop::configure(const alg::PidConfig &position, const alg::PidConfig &speed)
{
    position_.configure(position);
    speed_.configure(speed);
}

void PositionSpeedLoop::reset()
{
    position_.reset();
    speed_.reset();
}

float PositionSpeedLoop::update(float target_rad, float measured_rad, float measured_rad_s, float dt_s)
{
    const float target_speed = position_.update(target_rad, measured_rad, dt_s);
    return speed_.update(target_speed, measured_rad_s, dt_s);
}
