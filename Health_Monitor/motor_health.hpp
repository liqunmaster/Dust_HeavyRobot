#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "fdcan_channel.hpp"
#include "fdcan_port.hpp"

#define MOTOR_CHECK_PERIOD_MS 10
#define MOTOR_OFFLINE_TIMEOUT_MS 30
#define CUBEMARS_FEEDBACK_TIMEOUT_MS 100
#define MOTOR_RECOVERY_PERIOD_MS 1
#define CUBEMARS_RECOVERY_PERIOD_MS 50

void motor_health_init();

void motor_health_poll();

bool motor_health_is_offline(size_t index);

