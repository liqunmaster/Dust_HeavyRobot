#pragma once

#include <stddef.h>
#include <stdint.h>

enum class RemoteUartSource : uint8_t
{
    uart1,
    uart4
};

enum class RemoteProtocol : uint8_t
{
    none,
    dt7,
    vt02,
    vt03
};

struct RemoteKeyboard
{
    bool w, s, a, d;
    bool shift, ctrl;
    bool q, e, r, f, g, z, x, c, v, b;
};

inline RemoteKeyboard remote_decode_keyboard(uint16_t bits)
{
    RemoteKeyboard keys{};
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

inline uint16_t remote_unsigned_word(const uint8_t *bytes)
{
    return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

inline int16_t remote_signed_word(const uint8_t *bytes)
{
    const int32_t value = remote_unsigned_word(bytes);
    return static_cast<int16_t>(value < 0x8000 ? value : value - 0x10000);
}
