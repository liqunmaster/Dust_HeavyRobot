#include "bsp_pwm.hpp"

#define PWM_NODE DT_NODELABEL(pwm0)
#define PWM_DMA_CHANNEL DT_DMAS_CELL_BY_NAME(PWM_NODE, ws2812b, channel)
#define PWM_DMA_SOURCE DT_DMAS_CELL_BY_NAME(PWM_NODE, ws2812b, source)
#define PWM_OUTPUT_CHANNEL 6U
#define PWM_TRIGGER_CMP 23U

static const struct device *const pwm_device = DEVICE_DT_GET(PWM_NODE);
static const struct device *const dma_device = DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(PWM_NODE, ws2812b));

static uint32_t dma_words[BSP_PWM_MAX_PULSES + BSP_PWM_RESET_PULSES]
    __attribute__((section("AHB_SRAM"), aligned(4)));
static K_MUTEX_DEFINE(pwm_lock);
static K_SEM_DEFINE(dma_done, 0, 1);
static uint64_t clock_hz;
static uint32_t period_ticks;
static uint32_t trigger_ticks;
static uint32_t period_setting;
static bool initialized;

static uint32_t ns_to_ticks(uint32_t ns)
{
    return (uint32_t)((clock_hz * ns + 500000000ULL) / 1000000000ULL);
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
    if (period_ns == 0U) {
        return -EINVAL;
    }
    k_mutex_lock(&pwm_lock, K_FOREVER);
    if (initialized) {
        const int result = period_setting == period_ns ? 0 : -EBUSY;
        k_mutex_unlock(&pwm_lock);
        return result;
    }
    if (!device_is_ready(pwm_device) || !device_is_ready(dma_device)) {
        k_mutex_unlock(&pwm_lock);
        return -ENODEV;
    }

    clock_add_to_group(clock_mot0, 0);

    int result = pwm_get_cycles_per_sec(pwm_device, PWM_OUTPUT_CHANNEL, &clock_hz);
    if (result != 0) {
        k_mutex_unlock(&pwm_lock);
        return result;
    }
    period_ticks = ns_to_ticks(period_ns);
    if (period_ticks < 4U || period_ticks > 0xFFFFFFU) {
        k_mutex_unlock(&pwm_lock);
        return -EINVAL;
    }

    pwm_stop_counter(HPM_PWM0);
    pwm_set_reload(HPM_PWM0, 0U, period_ticks);
    pwm_set_start_count(HPM_PWM0, 0U, 0U);

    pwm_config_t output;
    pwm_cmp_config_t compare;
    pwm_get_default_pwm_config(HPM_PWM0, &output);
    pwm_get_default_cmp_config(HPM_PWM0, &compare);
    output.enable_output = true;
    output.invert_output = true;

    compare.cmp = 0U;
    if (pwm_setup_waveform(HPM_PWM0, PWM_OUTPUT_CHANNEL, &output, PWM_OUTPUT_CHANNEL, &compare, 1U) != status_success) {
        k_mutex_unlock(&pwm_lock);
        return -EIO;
    }

    trigger_ticks = ns_to_ticks(850U);
    if (trigger_ticks >= period_ticks) {
        k_mutex_unlock(&pwm_lock);
        return -EINVAL;
    }
    compare.cmp = trigger_ticks;
    compare.update_trigger = pwm_shadow_register_update_on_modify;
    pwm_config_cmp(HPM_PWM0, PWM_TRIGGER_CMP, &compare);
    trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM0_CMP23);

    pwm_issue_shadow_register_lock_event(HPM_PWM0);

    pwm_timer_reset(HPM_PWM0);
    pwm_start_counter(HPM_PWM0);
    k_busy_wait(3U);
    pwm_stop_counter(HPM_PWM0);

    period_setting = period_ns;
    initialized = true;
    k_mutex_unlock(&pwm_lock);
    return 0;
}

void bsp_pwm_write(const uint16_t *high_ns, size_t count)
{
    if (high_ns == NULL || count == 0U || count > BSP_PWM_MAX_PULSES) {
        return;
    }
    k_mutex_lock(&pwm_lock, K_FOREVER);
    if (!initialized) {
        k_mutex_unlock(&pwm_lock);
        return;
    }

    for (size_t i = 0U; i < count; ++i) {
        const uint32_t ticks = ns_to_ticks(high_ns[i]);
        if (ticks == 0U || ticks >= trigger_ticks) {
            k_mutex_unlock(&pwm_lock);
            return;
        }
        dma_words[i] = PWM_CMP_CMP_SET(ticks);
    }

    for (size_t i = 0U; i < BSP_PWM_RESET_PULSES; ++i) {
        dma_words[count + i] = PWM_CMP_CMP_SET(0U);
    }

    struct dma_block_config block = {0};
    block.source_address = (uint32_t)dma_words;
    block.dest_address = (uint32_t)&HPM_PWM0->CMP[PWM_OUTPUT_CHANNEL];
    block.block_size = (count + BSP_PWM_RESET_PULSES) * sizeof(dma_words[0]);
    block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

    struct dma_config config = {0};
    config.dma_slot = PWM_DMA_SOURCE;
    config.channel_direction = MEMORY_TO_PERIPHERAL;
    config.source_data_size = sizeof(dma_words[0]);
    config.dest_data_size = sizeof(dma_words[0]);
    config.source_burst_length = 1U;
    config.dest_burst_length = 1U;
    config.block_count = 1U;
    config.head_block = &block;
    config.dma_callback = pwm_dma_callback;

    k_sem_reset(&dma_done);
    int result = dma_config(dma_device, PWM_DMA_CHANNEL, &config);
    if (result == 0) {
        result = dma_start(dma_device, PWM_DMA_CHANNEL);
    }
    if (result == 0) {
        pwm_disable_pwm_sw_force_output(HPM_PWM0, PWM_OUTPUT_CHANNEL);
        pwm_cmp_update_cmp_value(HPM_PWM0, PWM_OUTPUT_CHANNEL, 0U, 0U);
        pwm_timer_reset(HPM_PWM0);
        pwm_clear_status(HPM_PWM0, PWM_IRQ_CMP(PWM_TRIGGER_CMP));
        pwm_enable_dma_request(HPM_PWM0, PWM_IRQ_CMP(PWM_TRIGGER_CMP));
        pwm_start_counter(HPM_PWM0);
        k_sem_take(&dma_done, K_MSEC(2));
    }

    pwm_disable_dma_request(HPM_PWM0, PWM_IRQ_CMP(PWM_TRIGGER_CMP));

    pwm_stop_counter(HPM_PWM0);

    (void)dma_stop(dma_device, PWM_DMA_CHANNEL);

    pwm_config_force_polarity(HPM_PWM0, false);
    pwm_config_force_cmd_timing(HPM_PWM0, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(HPM_PWM0, PWM_OUTPUT_CHANNEL);
    pwm_set_force_output(HPM_PWM0,
                         PWM_FORCE_OUTPUT(PWM_OUTPUT_CHANNEL, pwm_output_0));
    pwm_cmp_update_cmp_value(HPM_PWM0, PWM_OUTPUT_CHANNEL, 0U, 0U);

    k_busy_wait(80U);

    k_mutex_unlock(&pwm_lock);
}
