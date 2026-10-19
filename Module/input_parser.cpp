#include "input_parser.hpp"

void InputStreamParser::Reset()
{
    length_ = 0;
    state_ = ParseState::header;
    candidate_ = RemoteProtocol::none;
    expected_length_ = 0;
}

void InputStreamParser::Consume(size_t count)
{
    if (count >= length_) {
        length_ = 0;
    } else {
        memmove(buffer_, buffer_ + count, length_ - count);
        length_ -= count;
    }
    state_ = ParseState::header;
    candidate_ = RemoteProtocol::none;
    expected_length_ = 0;
}

void InputStreamParser::TryDecode(uint32_t timestamp_ms, SampleCallback callback, void *context)
{
    while (length_ != 0) {
        if (source_ == RemoteUartSource::uart4) {
            if (length_ < Dt7::frame_size) {
                return;
            }
            InputSample sample{};
            sample.protocol = RemoteProtocol::dt7;
            sample.source = source_;
            sample.timestamp_ms = timestamp_ms;
            if (Dt7::DecodeFrame(buffer_, Dt7::frame_size, sample.dt7_data) == 0) {
                Consume(Dt7::frame_size);
                callback(sample, context);
            } else {
                Consume(1);
            }
            continue;
        }

        switch (state_) {
            case ParseState::header:
                if (buffer_[0] == 0xA9) {
                    state_ = ParseState::vt03_header;
                } else if (buffer_[0] == 0xA5) {
                    state_ = ParseState::vt02_header;
                } else {
                    Consume(1);
                }
                break;
            case ParseState::vt03_header:
                if (length_ < 2) {
                    return;
                }
                if (buffer_[1] != 0x53) {
                    Consume(1);
                    break;
                }
                expected_length_ = Vt03::frame_size;
                candidate_ = RemoteProtocol::vt03;
                state_ = ParseState::frame;
                break;
            case ParseState::vt02_header:
                if (length_ < Vt02::header_size) {
                    return;
                }
                if (Vt02::FrameLengthFromHeader(buffer_, length_, expected_length_) != 0 || expected_length_ > sizeof(buffer_)) {
                    Consume(1);
                    break;
                }
                candidate_ = RemoteProtocol::vt02;
                state_ = ParseState::frame;
                break;
            case ParseState::frame:
                if (length_ < expected_length_) {
                    return;
                }
                InputSample sample{};
                sample.protocol = candidate_;
                sample.source = source_;
                sample.timestamp_ms = timestamp_ms;
                const int result = candidate_ == RemoteProtocol::vt03 ? Vt03::DecodeFrame(buffer_, expected_length_, sample.vt03_data) : Vt02::DecodeFrame(buffer_, expected_length_, sample.vt02_data);
                if (result == 0) {
                    Consume(expected_length_);
                    callback(sample, context);
                } else if (result == -ENOTSUP) {
                    Consume(expected_length_);
                } else {

                    Consume(1);
                }
                break;
        }
    }
}

void InputStreamParser::Feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, SampleCallback callback, void *context)
{
    if (data == nullptr || callback == nullptr) {
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        if (length_ == sizeof(buffer_)) {
            Consume(1);
        }
        buffer_[length_++] = data[i];
        TryDecode(timestamp_ms, callback, context);
    }
}
