#include "crc.hpp"

namespace
{
    constexpr uint8_t crc8_init = 0xFFU;
    constexpr uint16_t crc16_init = 0xFFFFU;
}

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021U, 16U, false));
}

unsigned char get_crc8_check_sum(unsigned char *message, unsigned int length, unsigned char crc8)
{
    return static_cast<unsigned char>(bsp_crc_calculate(message, length, crc8, 0x31U, 8U, true));
}

unsigned int verify_crc8_check_sum(unsigned char *message, unsigned int length)
{
    if (message == nullptr || length <= 2U) {
        return 0U;
    }
    return get_crc8_check_sum(message, length - 1U, crc8_init) == message[length - 1U];
}

void append_crc8_check_sum(unsigned char *message, unsigned int length)
{
    if (message == nullptr || length <= 2U) {
        return;
    }
    message[length - 1U] = get_crc8_check_sum(message, length - 1U, crc8_init);
}

uint16_t get_crc16_check_sum(uint8_t *message, uint32_t length, uint16_t crc_value)
{
    if (message == nullptr) {
        return 0xFFFFU;
    }
    return static_cast<uint16_t>(bsp_crc_calculate(message, length, crc_value, 0x1021U, 16U, true));
}

uint16_t crc16_dji(const uint8_t *data, size_t length)
{
    return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021U, 16U, true));
}

uint32_t verify_crc16_check_sum(uint8_t *message, uint32_t length)
{
    if (message == nullptr || length <= 2U) {
        return 0U;
    }
    const uint16_t expected = get_crc16_check_sum(message, length - 2U, crc16_init);
    return (expected & 0xFFU) == message[length - 2U] && (expected >> 8U) == message[length - 1U];
}

void append_crc16_check_sum(uint8_t *message, uint32_t length)
{
    if (message == nullptr || length <= 2U) {
        return;
    }
    const uint16_t crc_value = get_crc16_check_sum(message, length - 2U, crc16_init);
    message[length - 2U] = static_cast<uint8_t>(crc_value);
    message[length - 1U] = static_cast<uint8_t>(crc_value >> 8U);
}
