#include "input_parser.hpp"

void InputStreamParser::Reset()
{
	_length = 0;
	_state = ParseState::header;
	_candidate = RemoteProtocol::none;
	_expected_length = 0;
}

void InputStreamParser::Consume(size_t count)
{
	if (count >= _length) {
		_length = 0;

	} else {
		memmove(_buffer, _buffer + count, _length - count);
		_length -= count;
	}

	_state = ParseState::header;
	_candidate = RemoteProtocol::none;
	_expected_length = 0;
}

void InputStreamParser::TryDecode(uint32_t timestamp_ms, SampleCallback callback, void *context)
{
	while (_length != 0) {
		if (_source == RemoteUartSource::uart4) {
			if (_length < Dt7::frame_size) {
				return;
			}

			InputSample sample{};
			sample.protocol = RemoteProtocol::dt7;
			sample.source = _source;
			sample.timestamp_ms = timestamp_ms;

			if (Dt7::DecodeFrame(_buffer, Dt7::frame_size, sample.dt7_data) == 0) {
				Consume(Dt7::frame_size);
				callback(sample, context);

			} else {
				Consume(1);
			}

			continue;
		}

		switch (_state) {
		case ParseState::header:
			if (_buffer[0] == 0xA9) {
				_state = ParseState::vt03_header;

			} else if (_buffer[0] == 0xA5) {
				_state = ParseState::vt02_header;

			} else {
				Consume(1);
			}

			break;

		case ParseState::vt03_header:
			if (_length < 2) {
				return;
			}

			if (_buffer[1] != 0x53) {
				Consume(1);
				break;
			}

			_expected_length = Vt03::frame_size;
			_candidate = RemoteProtocol::vt03;
			_state = ParseState::frame;
			break;

		case ParseState::vt02_header:
			if (_length < Vt02::header_size) {
				return;
			}

			if (Vt02::FrameLengthFromHeader(_buffer, _length, _expected_length) != 0 || _expected_length > sizeof(_buffer)) {
				Consume(1);
				break;
			}

			_candidate = RemoteProtocol::vt02;
			_state = ParseState::frame;
			break;

		case ParseState::frame:
			if (_length < _expected_length) {
				return;
			}

			InputSample sample{};
			sample.protocol = _candidate;
			sample.source = _source;
			sample.timestamp_ms = timestamp_ms;
			const int result = _candidate == RemoteProtocol::vt03 ? Vt03::DecodeFrame(_buffer, _expected_length,
					   sample.vt03_data) : Vt02::DecodeFrame(_buffer, _expected_length, sample.vt02_data);

			if (result == 0) {
				Consume(_expected_length);
				callback(sample, context);

			} else if (result == -ENOTSUP) {
				Consume(_expected_length);

			} else {

				Consume(1);
			}

			break;
		}
	}
}

void InputStreamParser::Feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, SampleCallback callback,
			     void *context)
{
	if (data == nullptr || callback == nullptr) {
		return;
	}

	for (size_t i = 0; i < length; ++i) {
		if (_length == sizeof(_buffer)) {
			Consume(1);
		}

		_buffer[_length++] = data[i];
		TryDecode(timestamp_ms, callback, context);
	}
}
