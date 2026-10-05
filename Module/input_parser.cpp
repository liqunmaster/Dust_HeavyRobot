#include "input_parser.hpp"

void input_stream_parser::reset()
{
    length_ = 0U;
    state_ = parse_state::header;
    candidate_ = remote_protocol::none;
    expected_length_ = 0U;
}

void input_stream_parser::consume(size_t count)
{
    if (count >= length_) {
        length_ = 0U;
    } else {
        memmove(buffer_, buffer_ + count, length_ - count);
        length_ -= count;
    }
    state_ = parse_state::header;
    candidate_ = remote_protocol::none;
    expected_length_ = 0U;
}

void input_stream_parser::try_decode(uint32_t timestamp_ms, sample_callback callback, void *context)
{
    while (length_ != 0U) {
        if (source_ == remote_uart_source::uart4) {
            if (length_ < dt7::frame_size) {
                return;
            }
            input_sample sample{};
            sample.protocol = remote_protocol::dt7;
            sample.source = source_;
            sample.timestamp_ms = timestamp_ms;
            if (dt7::decode_frame(buffer_, dt7::frame_size, sample.dt7_data) == 0) {
                consume(dt7::frame_size);
                callback(sample, context);
            } else {
                consume(1U);
            }
            continue;
        }

        switch (state_) {
            case parse_state::header:
                if (buffer_[0] == 0xA9U) {
                    state_ = parse_state::vt03_header;
                } else if (buffer_[0] == 0xA5U) {
                    state_ = parse_state::vt02_header;
                } else {
                    consume(1U);
                }
                break;
            case parse_state::vt03_header:
                if (length_ < 2U) {
                    return;
                }
                if (buffer_[1] != 0x53U) {
                    consume(1U);
                    break;
                }
                expected_length_ = vt03::frame_size;
                candidate_ = remote_protocol::vt03;
                state_ = parse_state::frame;
                break;
            case parse_state::vt02_header:
                if (length_ < vt02::header_size) {
                    return;
                }
                if (vt02::frame_length_from_header(buffer_, length_, expected_length_) != 0 || expected_length_ > sizeof(buffer_)) {
                    consume(1U);
                    break;
                }
                candidate_ = remote_protocol::vt02;
                state_ = parse_state::frame;
                break;
            case parse_state::frame:
                if (length_ < expected_length_) {
                    return;
                }
                input_sample sample{};
                sample.protocol = candidate_;
                sample.source = source_;
                sample.timestamp_ms = timestamp_ms;
                const int result = candidate_ == remote_protocol::vt03 ? vt03::decode_frame(buffer_, expected_length_, sample.vt03_data) : vt02::decode_frame(buffer_, expected_length_, sample.vt02_data);
                if (result == 0) {
                    consume(expected_length_);
                    callback(sample, context);
                } else if (result == -ENOTSUP) {
                    consume(expected_length_);
                } else {

                    consume(1U);
                }
                break;
        }
    }
}

void input_stream_parser::feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, sample_callback callback, void *context)
{
    if (data == nullptr || callback == nullptr) {
        return;
    }
    for (size_t i = 0U; i < length; ++i) {
        if (length_ == sizeof(buffer_)) {
            consume(1U);
        }
        buffer_[length_++] = data[i];
        try_decode(timestamp_ms, callback, context);
    }
}
