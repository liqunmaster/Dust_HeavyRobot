#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_clock_drv.h"

#ifdef __cplusplus
#undef __R
#endif

#define BSP_PWM_MAX_PULSES 24U

#define BSP_PWM_RESET_PULSES 48U

int bsp_pwm_init(uint32_t period_ns);

int bsp_pwm_write(const uint16_t *high_ns, size_t count);
