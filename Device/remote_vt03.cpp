#include "remote_vt03.hpp"

    /**
     * @brief 解析 VT03 遥控器数据帧，校验帧头/CRC 并解出各通道数据
     *
     * @param frame 待解析的数据帧
     * @param length 帧长度
     * @param sample 输出的解析结果
     * @return 成功返回 0，长度/帧头/CRC/数据越界返回对应负错误码
    */
    int vt03::decode_frame(const uint8_t *frame, size_t length, vt03_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }
    if (frame[0] != 0xA9 || frame[1] != 0x53) {
        return -EBADMSG;
    }

    const uint16_t received_crc = static_cast<uint16_t>(frame[19]) | (static_cast<uint16_t>(frame[20]) << 8);
    if (crc16_dji(frame, 19) != received_crc) {
        return -EBADMSG;
    }

    vt03_sample decoded{};
    decoded.channel[0] = (static_cast<uint16_t>(frame[2]) | (static_cast<uint16_t>(frame[3]) << 8)) & 0x07FF;
    decoded.channel[1] = ((static_cast<uint16_t>(frame[3]) >> 3) | (static_cast<uint16_t>(frame[4]) << 5)) & 0x07FF;
    decoded.channel[2] = ((static_cast<uint16_t>(frame[4]) >> 6) | (static_cast<uint16_t>(frame[5]) << 2) | (static_cast<uint16_t>(frame[6]) << 10)) & 0x07FF;
    decoded.channel[3] = ((static_cast<uint16_t>(frame[6]) >> 1) | (static_cast<uint16_t>(frame[7]) << 7)) & 0x07FF;
    for (size_t i = 0; i < 4; ++i) {
        if (decoded.channel[i] < 364 || decoded.channel[i] > 1684) {
            return -EBADMSG;
        }
    }
    decoded.mode_switch = (frame[7] >> 4) & 0x03;
    decoded.pause = ((frame[7] >> 6) & 0x01) != 0;
    decoded.custom_left = ((frame[7] >> 7) & 0x01) != 0;
    decoded.custom_right = (frame[8] & 0x01) != 0;
    decoded.wheel = ((static_cast<uint16_t>(frame[8]) >> 1) | (static_cast<uint16_t>(frame[9]) << 7)) & 0x07FF;
    decoded.trigger = ((frame[9] >> 4) & 0x01) != 0;
    if (decoded.mode_switch > 2 || decoded.wheel < 364 || decoded.wheel > 1684) {
        return -EBADMSG;
    }

    decoded.mouse_x = remote_signed_word(frame + 10);
    decoded.mouse_y = remote_signed_word(frame + 12);
    decoded.mouse_z = remote_signed_word(frame + 14);
    if ((frame[16] & 0x03) > 1 || ((frame[16] >> 2) & 0x03) > 1 || ((frame[16] >> 4) & 0x03) > 1) {
        return -EBADMSG;
    }
    decoded.mouse_left = (frame[16] & 0x03) != 0;
    decoded.mouse_right = ((frame[16] >> 2) & 0x03) != 0;
    decoded.mouse_middle = ((frame[16] >> 4) & 0x03) != 0;
    decoded.keyboard = remote_decode_keyboard(remote_unsigned_word(frame + 17));

    sample = decoded;
    return 0;
}