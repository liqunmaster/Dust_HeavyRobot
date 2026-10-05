#include "remote_vt03.hpp"

int vt03::decode_frame(const uint8_t *frame, size_t length, vt03_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }
    if (frame[0] != 0xA9U || frame[1] != 0x53U) {
        return -EBADMSG;
    }

    const uint16_t received_crc = static_cast<uint16_t>(frame[19]) | (static_cast<uint16_t>(frame[20]) << 8U);
    if (crc16_dji(frame, 19U) != received_crc) {
        return -EBADMSG;
    }

    vt03_sample decoded{};
    decoded.channel[0] = (static_cast<uint16_t>(frame[2]) | (static_cast<uint16_t>(frame[3]) << 8U)) & 0x07FFU;
    decoded.channel[1] = ((static_cast<uint16_t>(frame[3]) >> 3U) | (static_cast<uint16_t>(frame[4]) << 5U)) & 0x07FFU;
    decoded.channel[2] = ((static_cast<uint16_t>(frame[4]) >> 6U) | (static_cast<uint16_t>(frame[5]) << 2U) | (static_cast<uint16_t>(frame[6]) << 10U)) & 0x07FFU;
    decoded.channel[3] = ((static_cast<uint16_t>(frame[6]) >> 1U) | (static_cast<uint16_t>(frame[7]) << 7U)) & 0x07FFU;
    for (size_t i = 0U; i < 4U; ++i) {
        if (decoded.channel[i] < 364U || decoded.channel[i] > 1684U) {
            return -EBADMSG;
        }
    }
    decoded.mode_switch = (frame[7] >> 4U) & 0x03U;
    decoded.pause = ((frame[7] >> 6U) & 0x01U) != 0U;
    decoded.custom_left = ((frame[7] >> 7U) & 0x01U) != 0U;
    decoded.custom_right = (frame[8] & 0x01U) != 0U;
    decoded.wheel = ((static_cast<uint16_t>(frame[8]) >> 1U) | (static_cast<uint16_t>(frame[9]) << 7U)) & 0x07FFU;
    decoded.trigger = ((frame[9] >> 4U) & 0x01U) != 0U;
    if (decoded.mode_switch > 2U || decoded.wheel < 364U || decoded.wheel > 1684U) {
        return -EBADMSG;
    }

    decoded.mouse_x = remote_signed_word(frame + 10U);
    decoded.mouse_y = remote_signed_word(frame + 12U);
    decoded.mouse_z = remote_signed_word(frame + 14U);
    if ((frame[16] & 0x03U) > 1U || ((frame[16] >> 2U) & 0x03U) > 1U || ((frame[16] >> 4U) & 0x03U) > 1U) {
        return -EBADMSG;
    }
    decoded.mouse_left = (frame[16] & 0x03U) != 0U;
    decoded.mouse_right = ((frame[16] >> 2U) & 0x03U) != 0U;
    decoded.mouse_middle = ((frame[16] >> 4U) & 0x03U) != 0U;
    decoded.keyboard = remote_decode_keyboard(remote_unsigned_word(frame + 17U));

    sample = decoded;
    return 0;
}
