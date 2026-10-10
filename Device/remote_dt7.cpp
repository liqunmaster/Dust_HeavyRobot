#include "remote_dt7.hpp"

/**
 * @brief 解析 DT7 遥控器数据帧并校验通道与开关值
 *
 * @param frame 待解析的数据帧
 * @param length 帧长度
 * @param sample 输出的解析结果
 * @return 成功返回 0 长度不符返回 -EINVAL 数据异常返回 -EBADMSG
 */
int dt7::decode_frame(const uint8_t *frame, size_t length, dt7_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    dt7_sample decoded{};
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
