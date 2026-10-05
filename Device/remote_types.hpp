#pragma once

#include <stddef.h>
#include <stdint.h>

enum class remote_uart_source : uint8_t
{
    uart1,
    uart4
};

enum class remote_protocol : uint8_t
{
    none,
    dt7,
    vt02,
    vt03
};

struct remote_keyboard
{
    bool w, s, a, d;
    bool shift, ctrl;
    bool q, e, r, f, g, z, x, c, v, b;
};

inline remote_keyboard remote_decode_keyboard(uint16_t bits)
{
    remote_keyboard keys{};
    keys.w = ((bits >> 0U) & 0x01U) != 0U;
    keys.s = ((bits >> 1U) & 0x01U) != 0U;
    keys.a = ((bits >> 2U) & 0x01U) != 0U;
    keys.d = ((bits >> 3U) & 0x01U) != 0U;
    keys.shift = ((bits >> 4U) & 0x01U) != 0U;
    keys.ctrl = ((bits >> 5U) & 0x01U) != 0U;
    keys.q = ((bits >> 6U) & 0x01U) != 0U;
    keys.e = ((bits >> 7U) & 0x01U) != 0U;
    keys.r = ((bits >> 8U) & 0x01U) != 0U;
    keys.f = ((bits >> 9U) & 0x01U) != 0U;
    keys.g = ((bits >> 10U) & 0x01U) != 0U;
    keys.z = ((bits >> 11U) & 0x01U) != 0U;
    keys.x = ((bits >> 12U) & 0x01U) != 0U;
    keys.c = ((bits >> 13U) & 0x01U) != 0U;
    keys.v = ((bits >> 14U) & 0x01U) != 0U;
    keys.b = ((bits >> 15U) & 0x01U) != 0U;
    return keys;
}

inline uint16_t remote_unsigned_word(const uint8_t *bytes)
{
    return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8U);
}

inline int16_t remote_signed_word(const uint8_t *bytes)
{
    const int32_t value = remote_unsigned_word(bytes);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
}
