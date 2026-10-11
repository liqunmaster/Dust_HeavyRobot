#pragma once

#include "cpu_usage.hpp"

/**
 * @brief 初始化各健康监控子模块
 *
 */
inline void health_monitor_init()
{
    motor_health_init();
}
