#include "remote_vt02.hpp"

int Vt02::FrameLengthFromHeader(const uint8_t *frame, size_t length, size_t &frame_length)
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

int Vt02::DecodeFrame(const uint8_t *frame, size_t length, Vt02Sample &sample)
{
    size_t expected_length = 0;
    const int result = FrameLengthFromHeader(frame, length, expected_length);
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
    Vt02Sample decoded{};
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
