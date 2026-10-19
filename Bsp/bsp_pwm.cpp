#include "bsp_pwm.hpp"

#define PWM_NODE DT_NODELABEL(pwm0)
#define PWM_DMA_CHANNEL DT_DMAS_CELL_BY_NAME(PWM_NODE, ws2812b, channel)
#define PWM_DMA_SOURCE DT_DMAS_CELL_BY_NAME(PWM_NODE, ws2812b, source)

static const struct device *const pwm_device = DEVICE_DT_GET(PWM_NODE);
static const struct device *const dma_device = DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(PWM_NODE, ws2812b));

static PwmDmaBuffers dma_buffers __attribute__((section("AHB_SRAM"), aligned(4)));
static PwmState pwm_state;
static K_MUTEX_DEFINE(pwm_lock);
static K_SEM_DEFINE(dma_done, 0, 1);

static void RestoreHeaterLocked()
{
    if (!pwm_state.heater_initialized) {
        return;
    }
    pwm_stop_counter(HPM_PWM0);
    if (pwm_state.initialized) {
        pwm_enable_pwm_sw_force_output(HPM_PWM0, 6);
        pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(6, pwm_output_0));
    }
    pwm_set_reload(HPM_PWM0, 0, pwm_state.heater_period_ticks);
    const uint32_t duty_permille = static_cast<uint32_t>(atomic_get(&pwm_state.heater_duty_permille));
    const uint32_t compare_ticks = duty_permille > 0
                                       ? pwm_state.heater_period_ticks - duty_permille * pwm_state.heater_period_ticks / 1000U
                                       : pwm_state.heater_period_ticks + 1;
    pwm_shadow_register_unlock(HPM_PWM0);
    pwm_cmp_update_cmp_value(HPM_PWM0, 7, compare_ticks, 0);
    pwm_issue_shadow_register_lock_event(HPM_PWM0);
    if (duty_permille > 0) {
        pwm_disable_pwm_sw_force_output(HPM_PWM0, 7);
        pwm_timer_reset(HPM_PWM0);
        pwm_start_counter(HPM_PWM0);
    } else {
        pwm_enable_pwm_sw_force_output(HPM_PWM0, 7);
        pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(7, pwm_output_0));
    }
}

static void HeaterWorkHandler(struct k_work *)
{
    k_mutex_lock(&pwm_lock, K_FOREVER);
    RestoreHeaterLocked();
    k_mutex_unlock(&pwm_lock);
}

K_WORK_DEFINE(heater_work, HeaterWorkHandler);

static void HeaterWatchdogHandler(struct k_timer *)
{
    atomic_clear(&pwm_state.heater_duty_permille);
    k_work_submit(&heater_work);
}

K_TIMER_DEFINE(heater_watchdog, HeaterWatchdogHandler, nullptr);

int bsp_pwm_heater_init()
{
    k_mutex_lock(&pwm_lock, K_FOREVER);
    if (pwm_state.heater_initialized) {
        k_mutex_unlock(&pwm_lock);
        return 0;
    }
    if (!device_is_ready(pwm_device)) {
        k_mutex_unlock(&pwm_lock);
        return -ENODEV;
    }
    clock_add_to_group(clock_mot0, 0);
    uint64_t heater_clock_hz = 0;
    const int result = pwm_get_cycles_per_sec(pwm_device, 7, &heater_clock_hz);
    if (result != 0) {
        k_mutex_unlock(&pwm_lock);
        return result;
    }
    pwm_state.heater_period_ticks = static_cast<uint32_t>((heater_clock_hz * 50000 + 500000000ULL) / 1000000000ULL);
    if (pwm_state.heater_period_ticks < 4 || pwm_state.heater_period_ticks >= 0xFFFFFF) {
        k_mutex_unlock(&pwm_lock);
        return -EINVAL;
    }

    pwm_stop_counter(HPM_PWM0);
    pwm_set_reload(HPM_PWM0, 0, pwm_state.heater_period_ticks);
    pwm_set_start_count(HPM_PWM0, 0, 0);
    pwm_config_t output;
    pwm_cmp_config_t compare;
    pwm_get_default_pwm_config(HPM_PWM0, &output);
    pwm_get_default_cmp_config(HPM_PWM0, &compare);
    output.enable_output = true;
    compare.cmp = pwm_state.heater_period_ticks + 1;
    compare.update_trigger = pwm_shadow_register_update_on_modify;
    if (pwm_setup_waveform(HPM_PWM0, 7, &output, 7, &compare, 1) != status_success) {
        k_mutex_unlock(&pwm_lock);
        return -EIO;
    }
    pwm_config_force_polarity(HPM_PWM0, false);
    pwm_config_force_cmd_timing(HPM_PWM0, pwm_force_immediately);
    atomic_clear(&pwm_state.heater_duty_permille);
    pwm_state.heater_initialized = true;
    RestoreHeaterLocked();
    k_mutex_unlock(&pwm_lock);
    return 0;
}

void bsp_pwm_heater_set_duty(float duty)
{
    const float limited = duty > 0.0f ? (duty < 0.95f ? duty : 0.95f) : 0.0f;
    atomic_set(&pwm_state.heater_duty_permille, static_cast<atomic_val_t>(limited * 1000.0f));
    if (limited > 0.0f) {
        k_timer_start(&heater_watchdog, K_MSEC(300), K_NO_WAIT);
    } else {
        k_timer_stop(&heater_watchdog);
    }
    if (k_mutex_lock(&pwm_lock, K_NO_WAIT) == 0) {
        RestoreHeaterLocked();
        k_mutex_unlock(&pwm_lock);
    } else {
        k_work_submit(&heater_work);
    }
}

static uint32_t NsToTicks(uint32_t ns)
{
    return (uint32_t)((pwm_state.clock_hz * ns + 500000000LL) / 1000000000LL);
}

static void pwm_dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
    (void)dev;
    (void)user_data;
    (void)channel;
    (void)status;
    k_sem_give(&dma_done);
}

int bsp_pwm_init(uint32_t period_ns)
{
    if (period_ns == 0) {
        return -EINVAL;
    }
    k_mutex_lock(&pwm_lock, K_FOREVER);
    if (pwm_state.initialized) {
        const int result = pwm_state.period_setting == period_ns ? 0 : -EBUSY;
        k_mutex_unlock(&pwm_lock);
        return result;
    }
    if (!device_is_ready(pwm_device) || !device_is_ready(dma_device)) {
        k_mutex_unlock(&pwm_lock);
        return -ENODEV;
    }

    clock_add_to_group(clock_mot0, 0);

    int result = pwm_get_cycles_per_sec(pwm_device, 6, &pwm_state.clock_hz);
    if (result != 0) {
        k_mutex_unlock(&pwm_lock);
        return result;
    }
    pwm_state.period_ticks = NsToTicks(period_ns);
    if (pwm_state.period_ticks < 4 || pwm_state.period_ticks > 0xFFFFFF) {
        k_mutex_unlock(&pwm_lock);
        return -EINVAL;
    }

    pwm_enable_pwm_sw_force_output(HPM_PWM0, 7);
    pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(7, pwm_output_0));
    pwm_stop_counter(HPM_PWM0);
    pwm_set_reload(HPM_PWM0, 0, pwm_state.period_ticks);
    pwm_set_start_count(HPM_PWM0, 0, 0);

    pwm_config_t output;
    pwm_cmp_config_t compare;
    pwm_get_default_pwm_config(HPM_PWM0, &output);
    pwm_get_default_cmp_config(HPM_PWM0, &compare);
    output.enable_output = true;
    output.invert_output = true;

    compare.cmp = 0;
    if (pwm_setup_waveform(HPM_PWM0, 6, &output, 6, &compare, 1) != status_success) {
        k_mutex_unlock(&pwm_lock);
        return -EIO;
    }

    pwm_state.trigger_ticks = NsToTicks(850);
    if (pwm_state.trigger_ticks >= pwm_state.period_ticks) {
        k_mutex_unlock(&pwm_lock);
        return -EINVAL;
    }
    compare.cmp = pwm_state.trigger_ticks;
    compare.update_trigger = pwm_shadow_register_update_on_modify;
    pwm_config_cmp(HPM_PWM0, 23, &compare);
    trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM0_CMP23);

    pwm_issue_shadow_register_lock_event(HPM_PWM0);

    pwm_timer_reset(HPM_PWM0);
    pwm_start_counter(HPM_PWM0);
    k_busy_wait(3);
    pwm_stop_counter(HPM_PWM0);
    pwm_enable_pwm_sw_force_output(HPM_PWM0, 6);
    pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(6, pwm_output_0));

    pwm_state.period_setting = period_ns;
    pwm_state.initialized = true;
    RestoreHeaterLocked();
    k_mutex_unlock(&pwm_lock);
    return 0;
}

void bsp_pwm_write(const uint16_t *high_ns, size_t count)
{
    if (high_ns == NULL || count == 0 || count > bsp_pwm_max_pulses) {
        return;
    }
    k_mutex_lock(&pwm_lock, K_FOREVER);
    if (!pwm_state.initialized) {
        k_mutex_unlock(&pwm_lock);
        return;
    }

    for (size_t i = 0; i < count; ++i) {
        const uint32_t ticks = NsToTicks(high_ns[i]);
        if (ticks == 0 || ticks >= pwm_state.trigger_ticks) {
            k_mutex_unlock(&pwm_lock);
            return;
        }
        dma_buffers.words[i] = PWM_CMP_CMP_SET(ticks);
    }

    for (size_t i = 0; i < bsp_pwm_reset_pulses; ++i) {
        dma_buffers.words[count + i] = PWM_CMP_CMP_SET(0);
    }

    pwm_enable_pwm_sw_force_output(HPM_PWM0, 7);
    pwm_set_force_output(HPM_PWM0, PWM_FORCE_OUTPUT(7, pwm_output_0));
    pwm_stop_counter(HPM_PWM0);
    pwm_set_reload(HPM_PWM0, 0, pwm_state.period_ticks);

    struct dma_block_config block = {0};

    block.source_address = (uint32_t)&dma_buffers.words[1];
    block.dest_address = (uint32_t)&HPM_PWM0->CMP[6];
    block.block_size = (count + bsp_pwm_reset_pulses - 1) * sizeof(dma_buffers.words[0]);
    block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

    struct dma_config config = {0};
    config.dma_slot = PWM_DMA_SOURCE;
    config.channel_direction = MEMORY_TO_PERIPHERAL;
    config.source_data_size = sizeof(dma_buffers.words[0]);
    config.dest_data_size = sizeof(dma_buffers.words[0]);
    config.source_burst_length = 1;
    config.dest_burst_length = 1;
    config.block_count = 1;
    config.head_block = &block;
    config.dma_callback = pwm_dma_callback;

    k_sem_reset(&dma_done);
    int result = dma_config(dma_device, PWM_DMA_CHANNEL, &config);
    if (result == 0) {
        result = dma_start(dma_device, PWM_DMA_CHANNEL);
    }
    if (result == 0) {
        pwm_shadow_register_unlock(HPM_PWM0);
        pwm_cmp_update_cmp_value(HPM_PWM0, 6,
                                 PWM_CMP_CMP_GET(dma_buffers.words[0]), 0);
        pwm_timer_reset(HPM_PWM0);
        pwm_clear_status(HPM_PWM0, PWM_IRQ_CMP(23));
        pwm_enable_dma_request(HPM_PWM0, PWM_IRQ_CMP(23));
        pwm_disable_pwm_sw_force_output(HPM_PWM0, 6);
        pwm_start_counter(HPM_PWM0);
        k_sem_take(&dma_done, K_MSEC(2));
    }

    pwm_disable_dma_request(HPM_PWM0, PWM_IRQ_CMP(23));

    pwm_stop_counter(HPM_PWM0);

    (void)dma_stop(dma_device, PWM_DMA_CHANNEL);

    pwm_config_force_polarity(HPM_PWM0, false);
    pwm_config_force_cmd_timing(HPM_PWM0, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(HPM_PWM0, 6);
    pwm_set_force_output(HPM_PWM0,
                         PWM_FORCE_OUTPUT(6, pwm_output_0));
    pwm_cmp_update_cmp_value(HPM_PWM0, 6, 0, 0);

    k_busy_wait(80);

    RestoreHeaterLocked();

    k_mutex_unlock(&pwm_lock);
}
