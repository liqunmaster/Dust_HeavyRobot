#include "control.hpp"

void SpeedLoop::Configure(const alg::PIDConfig &config)
{
    pid_.Configure(config);
}

void SpeedLoop::Reset()
{
    pid_.Reset();
}

float SpeedLoop::Update(float target_rad_s, float measured_rad_s, float dt_s)
{
    return pid_.Update(target_rad_s, measured_rad_s, dt_s);
}

void PositionSpeedLoop::Configure(const alg::PIDConfig &position, const alg::PIDConfig &speed)
{
    position_.Configure(position);
    speed_.Configure(speed);
}

void PositionSpeedLoop::Reset()
{
    position_.Reset();
    speed_.Reset();
}

float PositionSpeedLoop::Update(float target_rad, float measured_rad, float measured_rad_s, float dt_s)
{
    const float target_speed = position_.Update(target_rad, measured_rad, dt_s);
    return speed_.Update(target_speed, measured_rad_s, dt_s);
}
