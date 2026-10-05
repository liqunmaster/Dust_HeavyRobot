#include "remote_dt7.hpp"

int dt7::decode_frame(const uint8_t *frame, size_t length, dt7_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    dt7_sample decoded{};
    decoded.channel[0] = (static_cast<uint16_t>(frame[0]) | (static_cast<uint16_t>(frame[1]) << 8U)) & 0x07FFU;
    decoded.channel[1] = ((static_cast<uint16_t>(frame[1]) >> 3U) | (static_cast<uint16_t>(frame[2]) << 5U)) & 0x07FFU;
    decoded.channel[2] = ((static_cast<uint16_t>(frame[2]) >> 6U) | (static_cast<uint16_t>(frame[3]) << 2U) | (static_cast<uint16_t>(frame[4]) << 10U)) & 0x07FFU;
    decoded.channel[3] = ((static_cast<uint16_t>(frame[4]) >> 1U) | (static_cast<uint16_t>(frame[5]) << 7U)) & 0x07FFU;
    for (size_t i = 0U; i < 4U; ++i) {
        if (decoded.channel[i] < 364U || decoded.channel[i] > 1684U) {
            return -EBADMSG;
        }
    }
    decoded.switch_left = (frame[5] >> 4U) & 0x03U;
    decoded.switch_right = (frame[5] >> 6U) & 0x03U;
    if (decoded.switch_left < 1U || decoded.switch_left > 3U || decoded.switch_right < 1U || decoded.switch_right > 3U) {
        return -EBADMSG;
    }

    sample = decoded;

    return 0;
}
