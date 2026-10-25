#pragma once

#include <string.h>
#include "remote_dt7.hpp"
#include "remote_vt02.hpp"
#include "remote_vt03.hpp"
#include "remote_types.hpp"

struct InputSample {
	RemoteProtocol protocol;
	RemoteUartSource source;
	uint32_t timestamp_ms;

	union {
		Dt7Sample dt7_data;
		Vt02Sample vt02_data;
		Vt03Sample vt03_data;
	};
};

class InputStreamParser
{
public:
	using SampleCallback = void (*)(const InputSample &, void *);

	explicit InputStreamParser(RemoteUartSource source) : _source(source) {}

	void Reset();

	void Feed(const uint8_t *data, size_t length, uint32_t timestamp_ms, SampleCallback callback, void *context);

private:

	enum class ParseState : uint8_t {
		header,
		vt03_header,
		vt02_header,
		frame
	};

	void Consume(size_t count);

	void TryDecode(uint32_t timestamp_ms, SampleCallback callback, void *context);

	RemoteUartSource _source;

	ParseState _state = ParseState::header;

	RemoteProtocol _candidate = RemoteProtocol::none;

	size_t _expected_length = 0;

	size_t _length = 0;

	uint8_t _buffer[128] {};
};
