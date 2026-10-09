#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "fdcan_channel.hpp"
#include "fdcan_port.hpp"

#define MOTOR_CHECK_PERIOD_MS 10U
#define MOTOR_OFFLINE_TIMEOUT_MS 30U
#define CUBEMARS_FEEDBACK_TIMEOUT_MS 100U
#define MOTOR_RECOVERY_PERIOD_MS 1U
#define CUBEMARS_RECOVERY_PERIOD_MS 50U

void motor_health_init();

void motor_health_poll();

bool motor_health_is_offline(size_t index);
