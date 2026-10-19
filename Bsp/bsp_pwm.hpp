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

inline constexpr size_t bsp_pwm_max_pulses = 24;

inline constexpr size_t bsp_pwm_reset_pulses = 48;

int bsp_pwm_init(uint32_t period_ns);

void bsp_pwm_write(const uint16_t *high_ns, size_t count);

int bsp_pwm_heater_init();

void bsp_pwm_heater_set_duty(float duty);

struct PwmDmaBuffers
{
    uint32_t words[bsp_pwm_max_pulses + bsp_pwm_reset_pulses];
};

struct PwmState
{
    uint64_t clock_hz{};
    uint32_t period_ticks{};
    uint32_t trigger_ticks{};
    uint32_t period_setting{};
    bool initialized{};
    bool heater_initialized{};
    atomic_t heater_duty_permille{};
    uint32_t heater_period_ticks{};
};
