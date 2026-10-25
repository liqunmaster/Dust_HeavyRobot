#include "bsp_pwm.hpp"
#include "hpm_clock_nowait.hpp"

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"

namespace
{
    const device *const pwm_device = DEVICE_DT_GET(DT_NODELABEL(pwm0));
    bool initialized;
    uint32_t period_ticks;

    // Call only inside a short IRQ critical section. Never stop/reset PWM0 here.
    void ApplyDuty(uint32_t permille)
    {
        if (!initialized) {
            return;
        }
        const uint32_t compare = permille != 0U
            ? period_ticks - permille * period_ticks / 1000U
            : period_ticks + 1U;
        pwm_shadow_register_unlock(HPM_PWM0);
        pwm_cmp_update_cmp_value(HPM_PWM0, 7, compare, 0);
        pwm_issue_shadow_register_lock_event(HPM_PWM0);
        if (permille != 0U) {
            pwm_disable_pwm_sw_force_output(HPM_PWM0, 7);
        } else {
            pwm_enable_pwm_sw_force_output(HPM_PWM0, 7);
            pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(7, pwm_output_0));
        }
    }

    void HeaterWatchdogHandler(k_timer *)
    {
        const unsigned int key = irq_lock();
        ApplyDuty(0);
        irq_unlock(key);
    }

    K_TIMER_DEFINE(heater_watchdog, HeaterWatchdogHandler, nullptr);
}

int bsp_pwm_heater_init()
{
    if (!device_is_ready(pwm_device)) {
        return -ENODEV;
    }
    const int clock_ret = hpm_enable_resource_nowait(sysctl_resource_mot0);
    if (clock_ret != 0) {
        return clock_ret;
    }
    uint64_t clock_hz;
    const int ret = pwm_get_cycles_per_sec(pwm_device, 7, &clock_hz);
    if (ret != 0) {
        return ret;
    }
    const uint32_t ticks = static_cast<uint32_t>((clock_hz * 50000U + 500000000ULL) / 1000000000ULL);
    if (ticks < 4U || ticks >= 0xFFFFFFU) {
        return -EINVAL;
    }

    const unsigned int key = irq_lock();
    if (initialized) {
        irq_unlock(key);
        return 0;
    }
    pwm_stop_counter(HPM_PWM0);
    pwm_set_reload(HPM_PWM0, 0, ticks);
    pwm_set_start_count(HPM_PWM0, 0, 0);
    pwm_config_t output;
    pwm_cmp_config_t compare;
    pwm_get_default_pwm_config(HPM_PWM0, &output);
    pwm_get_default_cmp_config(HPM_PWM0, &compare);
    output.enable_output = true;
    compare.cmp = ticks + 1U;
    compare.update_trigger = pwm_shadow_register_update_on_modify;
    if (pwm_setup_waveform(HPM_PWM0, 7, &output, 7, &compare, 1) != status_success) {
        irq_unlock(key);
        return -EIO;
    }
    pwm_config_force_polarity(HPM_PWM0, false);
    pwm_config_force_cmd_timing(HPM_PWM0, pwm_force_immediately);
    period_ticks = ticks;
    initialized = true;
    ApplyDuty(0);
    pwm_timer_reset(HPM_PWM0);
    pwm_start_counter(HPM_PWM0);
    irq_unlock(key);
    return 0;
}

void bsp_pwm_heater_set_duty(float duty)
{
    const float limited = duty > 0.0f ? (duty < 0.95f ? duty : 0.95f) : 0.0f;
    const unsigned int key = irq_lock();
    if (initialized) {
        if (limited > 0.0f) {
            k_timer_start(&heater_watchdog, K_MSEC(300), K_NO_WAIT);
        } else {
            k_timer_stop(&heater_watchdog);
        }
        ApplyDuty(static_cast<uint32_t>(limited * 1000.0f));
    }
    irq_unlock(key);
}
