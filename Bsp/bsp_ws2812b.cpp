#include "bsp_ws2812b.hpp"
#include "ws2812b_encoding.hpp"
#include "hpm_clock_nowait.hpp"

#include <errno.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "hpm_clock_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_iomux.h"

namespace
{
    // CH2 generates PA10 edges. Synchronized CH3 requests the NEXT bit's CMP1
    // update at 1100 ns, after both possible falling edges in the current bit.
    // PWM0 belongs to the heater, PWM1 belongs to the servos.
    constexpr uint8_t timer_channel = 2;
    constexpr uint8_t request_channel = 3;
    constexpr uint8_t led_pin = 10;
    constexpr uint32_t dma_channel = DT_DMAS_CELL_BY_NAME(DT_NODELABEL(gpioa), ws2812b, channel);
    constexpr uint32_t dma_source = DT_DMAS_CELL_BY_NAME(DT_NODELABEL(gpioa), ws2812b, source);
    const device *const dma_device = DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(DT_NODELABEL(gpioa), ws2812b));
    const device *const gpio_device = DEVICE_DT_GET(DT_NODELABEL(gpioa));
    uint32_t words[ws2812b_encoding::frame_words]
        __attribute__((section("AHB_SRAM"), aligned(4)));
    atomic_t init_state; // 0 = uninitialized, 1 = configuring, 2 = ready
    atomic_t busy;
    atomic_t fault;
    uint32_t timer_clock_hz;
    enum class Phase { idle, starting, sending, resetting };
    Phase phase = Phase::idle;

    void ResetTimer(k_timer *);
    K_TIMER_DEFINE(reset_timer, ResetTimer, nullptr);

    void StopOutput()
    {
        gptmr_channel_enable_dma_request(HPM_GPTMR0, request_channel, false);
        gptmr_stop_counter(HPM_GPTMR0, request_channel);
        gptmr_stop_counter(HPM_GPTMR0, timer_channel);
        gpio_write_pin(HPM_GPIO0, 0, led_pin, 0);
        HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_GPIO_A_10;
        gptmr_disable_cmp_output(HPM_GPTMR0, timer_channel);
    }

    void DmaDone(const device *, void *, uint32_t, int status)
    {
        StopOutput();
        if (status != 0) {
            atomic_set(&fault, 1);
        }
        // Keep ownership until the reset-low interval completes, without waiting.
        phase = Phase::resetting;
        k_timer_start(&reset_timer, K_MSEC(1), K_NO_WAIT);
    }

    void ResetTimer(k_timer *)
    {
        if (phase == Phase::resetting) {
            phase = Phase::idle;
            atomic_clear(&busy);
            return;
        }
        if (phase != Phase::starting) {
            return;
        }
        phase = Phase::sending;
        gptmr_channel_reset_count(HPM_GPTMR0, timer_channel);
        gptmr_channel_reset_count(HPM_GPTMR0, request_channel);
        gptmr_update_cmp(HPM_GPTMR0, timer_channel, 1, words[0] + 1U);
        gptmr_enable_cmp_output(HPM_GPTMR0, timer_channel);
        gptmr_clear_status(HPM_GPTMR0, 0xFF00U);
        gptmr_channel_enable_dma_request(HPM_GPTMR0, request_channel, true);
        const unsigned int key = irq_lock();
        gptmr_start_counter(HPM_GPTMR0, timer_channel);
        gptmr_start_counter(HPM_GPTMR0, request_channel);
        gptmr_trigger_channel_software_sync(HPM_GPTMR0,
            GPTMR_CH_GCR_SWSYNCT_MASK(timer_channel) | GPTMR_CH_GCR_SWSYNCT_MASK(request_channel));
        // CMP0 is 200 ns after sync, leaving time to connect the idle-low pin.
        HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_GPTMR0_COMP_2;
        irq_unlock(key);
    }
}

int bsp_ws2812b_init()
{
    if (atomic_get(&init_state) == 2) {
        return 0;
    }
    if (!atomic_cas(&init_state, 0, 1)) {
        return -EBUSY;
    }
    if (!device_is_ready(dma_device) || !device_is_ready(gpio_device)) {
        atomic_clear(&init_state);
        return -ENODEV;
    }
    int ret = gpio_pin_configure(gpio_device, led_pin, GPIO_OUTPUT_LOW);
    if (ret == 0) {
        // Keep the existing clock source: switching it in the SDK busy-waits.
        ret = hpm_enable_resource_nowait(sysctl_resource_gptmr0);
    }
    if (ret == 0) {
        gptmr_channel_config_t config;
        gptmr_channel_get_default_config(HPM_GPTMR0, &config);
        config.enable_cmp_output = false;
        config.cmp_initial_polarity_high = false;
        config.enable_software_sync = true;
        config.debug_mode = false;
        timer_clock_hz = clock_get_frequency(clock_gptmr0);
        config.reload = ws2812b_encoding::NsToTicks(timer_clock_hz, ws2812b_encoding::bit_period_ns);
        config.cmp[0] = ws2812b_encoding::NsToTicks(timer_clock_hz, ws2812b_encoding::rising_edge_ns);
        config.cmp[1] = ws2812b_encoding::NsToTicks(timer_clock_hz,
            ws2812b_encoding::rising_edge_ns + ws2812b_encoding::zero_high_ns);
        if (timer_clock_hz < 24000000U || config.reload < 4U ||
            gptmr_channel_config(HPM_GPTMR0, timer_channel, &config, false) != status_success) {
            ret = -EIO;
        }
        if (ret == 0) {
            // CMPEN gates comparison events as well as the output. CH3 must
            // compare to request DMA, but its output is not muxed to any pin.
            config.enable_cmp_output = true;
            config.cmp[0] = ws2812b_encoding::NsToTicks(timer_clock_hz, ws2812b_encoding::dma_update_ns);
            config.cmp[1] = UINT32_MAX;
            config.dma_request_event = gptmr_dma_request_on_cmp0;
            if (gptmr_channel_config(HPM_GPTMR0, request_channel, &config, false) != status_success) {
                ret = -EIO;
            }
        }
        gptmr_channel_enable_dma_request(HPM_GPTMR0, request_channel, false);
    }
    atomic_set(&init_state, ret == 0 ? 2 : 0);
    return ret;
}

int bsp_ws2812b_write(uint8_t red, uint8_t green, uint8_t blue)
{
    const int ret = bsp_ws2812b_init();
    if (ret != 0) {
        return ret;
    }
    if (atomic_get(&fault)) {
        return -EIO;
    }
    if (!atomic_cas(&busy, 0, 1)) {
        return -EBUSY;
    }
    // Recheck after claiming the buffer in case an error callback just ran.
    if (atomic_get(&fault)) {
        atomic_clear(&busy);
        return -EIO;
    }
    ws2812b_encoding::Encode(words, timer_clock_hz, red, green, blue);

    dma_block_config block{};
    // First bit is preloaded; CH3 supplies subsequent values one period ahead.
    block.source_address = reinterpret_cast<uintptr_t>(&words[1]);
    block.dest_address = reinterpret_cast<uintptr_t>(&HPM_GPTMR0->CHANNEL[timer_channel].CMP[1]);
    block.block_size = sizeof(words) - sizeof(words[0]);
    block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    struct dma_config config{};
    config.dma_slot = dma_source;
    config.channel_direction = MEMORY_TO_PERIPHERAL;
    config.source_data_size = sizeof(uint32_t);
    config.dest_data_size = sizeof(uint32_t);
    config.source_burst_length = 1;
    config.dest_burst_length = 1;
    config.block_count = 1;
    config.head_block = &block;
    config.dma_callback = DmaDone;
    int result = dma_config(dma_device, dma_channel, &config);
    if (result == 0) {
        result = dma_start(dma_device, dma_channel);
    }
    if (result != 0) {
        StopOutput();
        atomic_set(&fault, 1);
        atomic_clear(&busy);
        return result;
    }
    // Asynchronous pre-reset: also guarantees the FIRST frame starts after a
    // full low interval. The timer callback starts hardware, never busy-waits.
    phase = Phase::starting;
    k_timer_start(&reset_timer, K_MSEC(1), K_NO_WAIT);
    return 0;
}
