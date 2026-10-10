#include "remote_vt02.hpp"

/**
 * @brief 依据帧头校验估算 VT02 数据帧总长度
 *
 * @param frame 待解析的数据帧
 * @param length 已接收的字节数
 * @param frame_length 输出的估算帧总长度
 * @return 成功返回 0 参数非法返回 -EINVAL 帧头校验失败返回 -EBADMSG
 */
int vt02::frame_length_from_header(const uint8_t *frame, size_t length, size_t &frame_length)
{
    if (frame == nullptr || length < header_size) {
        return -EINVAL;
    }
    uint8_t header[header_size];
    memcpy(header, frame, header_size);
    if (frame[0] != 0xA5 || verify_crc8_check_sum(header, header_size) == 0) {
        return -EBADMSG;
    }
    frame_length = static_cast<size_t>(remote_unsigned_word(frame + 1)) + 9;
    return 0;
}

/**
 * @brief 解析 VT02 遥控器数据帧 校验 CRC 并解出鼠标/键盘/滚轮数据
 *
 * @param frame 待解析的数据帧
 * @param length 帧长度
 * @param sample 输出的解析结果
 * @return 成功返回 0 长度/CRC/命令 ID 不符返回对应负错误码
 */
int vt02::decode_frame(const uint8_t *frame, size_t length, vt02_sample &sample)
{
    size_t expected_length = 0;
    const int result = frame_length_from_header(frame, length, expected_length);
    if (result != 0) {
        return result;
    }
    if (length != expected_length) {
        return -EINVAL;
    }
    const uint16_t received_crc = remote_unsigned_word(frame + length - 2);

    if (crc16_dji(frame, length - 2) != received_crc) {
        return -EBADMSG;
    }
    if (remote_unsigned_word(frame + 5) != command_id) {
        return -ENOTSUP;
    }
    if (length != frame_size) {
        return -EBADMSG;
    }
    vt02_sample decoded{};
    decoded.mouse_x = remote_signed_word(frame + 7);
    decoded.mouse_y = remote_signed_word(frame + 9);
    decoded.mouse_z = remote_signed_word(frame + 11);
    if (frame[13] > 1 || frame[14] > 1) {
        return -EBADMSG;
    }
    decoded.mouse_left = (frame[13] & 0x01) != 0;
    decoded.mouse_right = (frame[14] & 0x01) != 0;
    decoded.keyboard = remote_decode_keyboard(remote_unsigned_word(frame + 15));
    decoded.pulley_wheel = remote_signed_word(frame + 17);
    sample = decoded;
    return 0;
}
