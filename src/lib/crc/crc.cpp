#include "crc.hpp"
#include "dvc_crc.h"

namespace
{
constexpr uint8_t crc8_init = 0xFF;
constexpr uint16_t crc16_init = 0xFFFF;
}

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
	return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021, 16, false));
}

unsigned char get_crc8_check_sum(unsigned char *message, unsigned int length, unsigned char crc8)
{
	return static_cast<unsigned char>(bsp_crc_calculate(message, length, crc8, 0x31, 8, true));
}

unsigned int verify_crc8_check_sum(unsigned char *message, unsigned int length)
{
	if (message == nullptr || length <= 2) {
		return 0;
	}

	return get_crc8_check_sum(message, length - 1, crc8_init) == message[length - 1];
}

void append_crc8_check_sum(unsigned char *message, unsigned int length)
{
	if (message == nullptr || length <= 2) {
		return;
	}

	message[length - 1] = get_crc8_check_sum(message, length - 1, crc8_init);
}

uint16_t get_crc16_check_sum(uint8_t *message, uint32_t length, uint16_t crc_value)
{
	if (message == nullptr) {
		return 0xFFFF;
	}

	return static_cast<uint16_t>(bsp_crc_calculate(message, length, crc_value, 0x1021, 16, true));
}

uint16_t crc16_dji(const uint8_t *data, size_t length)
{
	return static_cast<uint16_t>(bsp_crc_calculate(data, length, crc16_init, 0x1021, 16, true));
}

uint32_t verify_crc16_check_sum(uint8_t *message, uint32_t length)
{
	if (message == nullptr || length <= 2) {
		return 0;
	}

	const uint16_t expected = get_crc16_check_sum(message, length - 2, crc16_init);
	return (expected & 0xFF) == message[length - 2] && (expected >> 8) == message[length - 1];
}

void append_crc16_check_sum(uint8_t *message, uint32_t length)
{
	if (message == nullptr || length <= 2) {
		return;
	}

	const uint16_t crc_value = get_crc16_check_sum(message, length - 2, crc16_init);
	message[length - 2] = static_cast<uint8_t>(crc_value);
	message[length - 1] = static_cast<uint8_t>(crc_value >> 8);
}
