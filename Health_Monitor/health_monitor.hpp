#pragma once

#include "cpu_usage.hpp"
#include "motor_health.hpp"

inline void health_monitor_init()
{
    motor_health_init();
}

inline void health_monitor_poll()
{
    motor_health_poll();
    cpu_usage_supervisor();
}
