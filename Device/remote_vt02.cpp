#include "remote_vt02.hpp"

int vt02::frame_length_from_header(const uint8_t *frame, size_t length, size_t &frame_length)
{
    if (frame == nullptr || length < header_size) {
        return -EINVAL;
    }
    uint8_t header[header_size];
    memcpy(header, frame, header_size);
    if (frame[0] != 0xA5U || verify_crc8_check_sum(header, header_size) == 0U) {
        return -EBADMSG;
    }
    frame_length = static_cast<size_t>(remote_unsigned_word(frame + 1U)) + 9U;
    return 0;
}

int vt02::decode_frame(const uint8_t *frame, size_t length, vt02_sample &sample)
{
    size_t expected_length = 0U;
    const int result = frame_length_from_header(frame, length, expected_length);
    if (result != 0) {
        return result;
    }
    if (length != expected_length) {
        return -EINVAL;
    }
    const uint16_t received_crc = remote_unsigned_word(frame + length - 2U);

    if (crc16_dji(frame, length - 2U) != received_crc) {
        return -EBADMSG;
    }
    if (remote_unsigned_word(frame + 5U) != command_id) {
        return -ENOTSUP;
    }
    if (length != frame_size) {
        return -EBADMSG;
    }
    vt02_sample decoded{};
    decoded.mouse_x = remote_signed_word(frame + 7U);
    decoded.mouse_y = remote_signed_word(frame + 9U);
    decoded.mouse_z = remote_signed_word(frame + 11U);
    if (frame[13] > 1U || frame[14] > 1U) {
        return -EBADMSG;
    }
    decoded.mouse_left = (frame[13] & 0x01U) != 0U;
    decoded.mouse_right = (frame[14] & 0x01U) != 0U;
    decoded.keyboard = remote_decode_keyboard(remote_unsigned_word(frame + 15U));
    decoded.pulley_wheel = remote_signed_word(frame + 17U);
    sample = decoded;
    return 0;
}
