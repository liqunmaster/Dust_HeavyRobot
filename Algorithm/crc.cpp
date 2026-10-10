#include "crc.hpp"

namespace
{
    constexpr uint8_t crc8_init = 0xFF;
    constexpr uint16_t crc16_init = 0xFFFF;
}

    /**
     * @brief 计算 CRC-16/CCITT-FALSE 校验值
     *
     * @param data 数据指针
     * @param length 数据长度（字节）
     * @return 16 位 CRC 校验值
    */
    uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
    {
    return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021, 16, false));
}

    /**
     * @brief 计算 CRC8 校验值
     *
     * @param message 数据指针
     * @param length 数据长度（字节）
     * @param crc8 初始校验值
     * @return CRC8 校验值
    */
    unsigned char get_crc8_check_sum(unsigned char *message, unsigned int length, unsigned char crc8)
    {
    return static_cast<unsigned char>(bsp_crc_calculate(message, length, crc8, 0x31, 8, true));
}

    /**
     * @brief 校验消息末尾的 CRC8 字节
     *
     * @param message 数据指针（末字节为 CRC）
     * @param length 数据总长度（字节）
     * @return 校验通过返回 1，否则 0
    */
    unsigned int verify_crc8_check_sum(unsigned char *message, unsigned int length)
    {
    if (message == nullptr || length <= 2) {
        return 0;
    }
    return get_crc8_check_sum(message, length - 1, crc8_init) == message[length - 1];
}

    /**
     * @brief 在消息末尾追加 CRC8 校验字节
     *
     * @param message 数据指针
     * @param length 数据总长度（字节）
    */
    void append_crc8_check_sum(unsigned char *message, unsigned int length)
    {
    if (message == nullptr || length <= 2) {
        return;
    }
    message[length - 1] = get_crc8_check_sum(message, length - 1, crc8_init);
}

    /**
     * @brief 计算 CRC16 校验值
     *
     * @param message 数据指针
     * @param length 数据长度（字节）
     * @param crc_value 初始校验值
     * @return 16 位 CRC 校验值
    */
    uint16_t get_crc16_check_sum(uint8_t *message, uint32_t length, uint16_t crc_value)
    {
    if (message == nullptr) {
        return 0xFFFF;
    }
    return static_cast<uint16_t>(bsp_crc_calculate(message, length, crc_value, 0x1021, 16, true));
}

    /**
     * @brief 计算 DJI 协议专用的 CRC16 校验值
     *
     * @param data 数据指针
     * @param length 数据长度（字节）
     * @return 16 位 CRC 校验值
    */
    uint16_t crc16_dji(const uint8_t *data, size_t length)
    {
    return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021, 16, true));
}

    /**
     * @brief 校验消息末尾的 CRC16 两个字节
     *
     * @param message 数据指针（末两字节为 CRC）
     * @param length 数据总长度（字节）
     * @return 校验通过返回 1，否则 0
    */
    uint32_t verify_crc16_check_sum(uint8_t *message, uint32_t length)
    {
    if (message == nullptr || length <= 2) {
        return 0;
    }
    const uint16_t expected = get_crc16_check_sum(message, length - 2, crc16_init);
    return (expected & 0xFF) == message[length - 2] && (expected >> 8) == message[length - 1];
}

    /**
     * @brief 在消息末尾追加 CRC16 校验两字节（小端）
     *
     * @param message 数据指针
     * @param length 数据总长度（字节）
    */
    void append_crc16_check_sum(uint8_t *message, uint32_t length)
    {
    if (message == nullptr || length <= 2) {
        return;
    }
    const uint16_t crc_value = get_crc16_check_sum(message, length - 2, crc16_init);
    message[length - 2] = static_cast<uint8_t>(crc_value);
    message[length - 1] = static_cast<uint8_t>(crc_value >> 8);
}