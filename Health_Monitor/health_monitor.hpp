#pragma once

#include "cpu_usage.hpp"
#include "motor_health.hpp"

/**
 * @brief 初始化各健康监控子模块
 *
 */
inline void health_monitor_init()
{
    motor_health_init();
}

/**
 * @brief 周期调度健康监控巡检
 *
 */
inline void health_monitor_poll()
{
    motor_health_poll();
    cpu_usage_supervisor();
}
