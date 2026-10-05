#include "bsp_crc.h"

#define CRC_CHANNEL 0U

static K_MUTEX_DEFINE(crc_mutex);
static bool crc_clock_enabled;

static uint32_t reflect_seed(uint32_t value, uint32_t width)
{
    uint32_t reflected = 0U;
    for (uint32_t bit = 0U; bit < width; ++bit) {
        reflected = (reflected << 1U) | (value & 1U);
        value >>= 1U;
    }
    return reflected;
}

__attribute__((noinline, noclone)) uint32_t bsp_crc_calculate(const uint8_t *data, size_t length, uint32_t initial,
                                                              uint32_t polynomial, uint32_t width, bool reflected)
{
    if (length == 0U) {
        return initial;
    }

    k_mutex_lock(&crc_mutex, K_FOREVER);
    if (!crc_clock_enabled) {
        clock_add_to_group(clock_crc0, 0U);
        crc_clock_enabled = true;
    }

    HPM_CRC->CHN[CRC_CHANNEL].CLR = CRC_CHN_CLR_CLR_SET(1U);
    crc_channel_config_t config = {0};
    config.preset = crc_preset_none;
    config.init = reflected ? reflect_seed(initial, width) : initial;
    config.poly = polynomial;
    config.poly_width = width;
    config.in_byte_order = crc_in_byte_order_lsb;
    config.refin = reflected ? crc_refin_true : crc_refin_false;
    config.refout = reflected ? crc_refout_true : crc_refout_false;
    config.xorout = 0U;
    (void)crc_setup_channel_config(HPM_CRC, CRC_CHANNEL, &config);

    for (size_t index = 0U; index < length; ++index) {
        crc_calc_byte(HPM_CRC, CRC_CHANNEL, data[index]);
    }
    const uint32_t result = crc_get_result(HPM_CRC, CRC_CHANNEL);
    k_mutex_unlock(&crc_mutex);
    return result;
}
