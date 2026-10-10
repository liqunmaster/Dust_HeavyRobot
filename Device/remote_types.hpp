#pragma once

#include <stddef.h>
#include <stdint.h>

// 遥控器数据接入的串口来源枚举
enum class remote_uart_source : uint8_t
{
    uart1,
    uart4
};

// 遥控器通信协议类型枚举
enum class remote_protocol : uint8_t
{
    none,
    dt7,
    vt02,
    vt03
};

// 遥控器键盘按键状态结构体 按位展开的布尔按键
struct remote_keyboard
{
    bool w, s, a, d;
    bool shift, ctrl;
    bool q, e, r, f, g, z, x, c, v, b;
};

/**
 * @brief 将 16 位按键位图解码为键盘按键结构体
 *
 * @param bits 按键位图 每位对应一个按键
 * @return 解码后的键盘按键状态
 */
inline remote_keyboard remote_decode_keyboard(uint16_t bits)
{
    remote_keyboard keys{};
    keys.w = ((bits >> 0) & 0x01) != 0;
    keys.s = ((bits >> 1) & 0x01) != 0;
    keys.a = ((bits >> 2) & 0x01) != 0;
    keys.d = ((bits >> 3) & 0x01) != 0;
    keys.shift = ((bits >> 4) & 0x01) != 0;
    keys.ctrl = ((bits >> 5) & 0x01) != 0;
    keys.q = ((bits >> 6) & 0x01) != 0;
    keys.e = ((bits >> 7) & 0x01) != 0;
    keys.r = ((bits >> 8) & 0x01) != 0;
    keys.f = ((bits >> 9) & 0x01) != 0;
    keys.g = ((bits >> 10) & 0x01) != 0;
    keys.z = ((bits >> 11) & 0x01) != 0;
    keys.x = ((bits >> 12) & 0x01) != 0;
    keys.c = ((bits >> 13) & 0x01) != 0;
    keys.v = ((bits >> 14) & 0x01) != 0;
    keys.b = ((bits >> 15) & 0x01) != 0;
    return keys;
}

/**
 * @brief 将小端字节序的 2 字节数据组合为无符号 16 位值
 *
 * @param bytes 小端字节数组指针
 * @return 组合后的无符号 16 位值
 */
inline uint16_t remote_unsigned_word(const uint8_t *bytes)
{
    return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

/**
 * @brief 将小端字节序的 2 字节数据组合为有符号 16 位值
 *
 * @param bytes 小端字节数组指针
 * @return 转换后的有符号 16 位值
 */
inline int16_t remote_signed_word(const uint8_t *bytes)
{
    const int32_t value = remote_unsigned_word(bytes);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
}
