#include "remote_dt7.hpp"

int Dt7::DecodeFrame(const uint8_t *frame, size_t length, Dt7Sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    Dt7Sample decoded{};
    decoded.channel[0] = (static_cast<uint16_t>(frame[0]) | (static_cast<uint16_t>(frame[1]) << 8)) & 0x07FF;
    decoded.channel[1] = ((static_cast<uint16_t>(frame[1]) >> 3) | (static_cast<uint16_t>(frame[2]) << 5)) & 0x07FF;
    decoded.channel[2] = ((static_cast<uint16_t>(frame[2]) >> 6) | (static_cast<uint16_t>(frame[3]) << 2) | (static_cast<uint16_t>(frame[4]) << 10)) & 0x07FF;
    decoded.channel[3] = ((static_cast<uint16_t>(frame[4]) >> 1) | (static_cast<uint16_t>(frame[5]) << 7)) & 0x07FF;
    for (size_t i = 0; i < 4; ++i) {
        if (decoded.channel[i] < 364 || decoded.channel[i] > 1684) {
            return -EBADMSG;
        }
    }
    decoded.switch_left = (frame[5] >> 4) & 0x03;
    decoded.switch_right = (frame[5] >> 6) & 0x03;
    if (decoded.switch_left < 1 || decoded.switch_left > 3 || decoded.switch_right < 1 || decoded.switch_right > 3) {
        return -EBADMSG;
    }

    sample = decoded;

    return 0;
}
