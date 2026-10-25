#pragma once

#include <errno.h>
#include <string.h>
#include "crc.hpp"
#include "remote_types.hpp"

struct Vt02Sample {
	int16_t mouse_x, mouse_y, mouse_z;
	bool mouse_left, mouse_right;
	RemoteKeyboard keyboard;
	int16_t pulley_wheel;
};

class Vt02
{
public:
	static constexpr size_t header_size = 5;

	static constexpr size_t frame_size = 21;

	static constexpr uint16_t command_id = 0x0304;

	static int FrameLengthFromHeader(const uint8_t *frame, size_t length, size_t &frame_length);

	static int DecodeFrame(const uint8_t *frame, size_t length, Vt02Sample &sample);
};
