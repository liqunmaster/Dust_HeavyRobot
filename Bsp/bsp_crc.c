#include "bsp_crc.h"

#define CRC_CHANNEL 0

static K_MUTEX_DEFINE(crc_mutex);
static bool crc_clock_enabled;

/**
 * @brief 将指定宽度的二进制数按位反转
 *
 * @param value 待反转的值
 * @param width 有效位宽
 * @return 反转后的值
 */
static uint32_t reflect_seed(uint32_t value, uint32_t width)
{
    uint32_t reflected = 0;
    for (uint32_t bit = 0; bit < width; ++bit) {
        reflected = (reflected << 1) | (value & 1);
        value >>= 1;
    }
    return reflected;
}

/**
 * @brief 使用硬件 CRC 计算指定数据的校验值
 *
 * @param data 待计算数据指针
 * @param length 数据长度
 * @param initial 初始值
 * @param polynomial 多项式
 * @param width 校验位宽
 * @param reflected 是否按位反转
 * @return 计算得到的 CRC 校验值
 */
__attribute__((noinline, noclone)) uint32_t bsp_crc_calculate(const uint8_t *data, size_t length, uint32_t initial,
uint32_t polynomial, uint32_t width, bool reflected)
{
    if (length == 0) {
        return initial;
    }

    k_mutex_lock(&crc_mutex, K_FOREVER);
    if (!crc_clock_enabled) {
        clock_add_to_group(clock_crc0, 0);
        crc_clock_enabled = true;
    }

    HPM_CRC->CHN[CRC_CHANNEL].CLR = CRC_CHN_CLR_CLR_SET(1);
    crc_channel_config_t config = {0};
    config.preset = crc_preset_none;
    config.init = reflected ? reflect_seed(initial, width) : initial;
    config.poly = polynomial;
    config.poly_width = width;
    config.in_byte_order = crc_in_byte_order_lsb;
    config.refin = reflected ? crc_refin_true : crc_refin_false;
    config.refout = reflected ? crc_refout_true : crc_refout_false;
    config.xorout = 0;
    (void)crc_setup_channel_config(HPM_CRC, CRC_CHANNEL, &config);

    for (size_t index = 0; index < length; ++index) {
        crc_calc_byte(HPM_CRC, CRC_CHANNEL, data[index]);
    }
    const uint32_t result = crc_get_result(HPM_CRC, CRC_CHANNEL);
    k_mutex_unlock(&crc_mutex);
    return result;
}
