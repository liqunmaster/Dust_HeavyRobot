#pragma once

#include <stdint.h>

constexpr uint8_t MCU_FRAME_CHASSIS = 0xAA;
constexpr uint8_t MCU_FRAME_COMMAND = 0xAB;
constexpr uint8_t MCU_FRAME_AUTOAIM = 0xAC;
constexpr uint8_t MCU_FRAME_REFEREE = 0xAF;

struct McuChassisCommand {
	uint16_t x;
	uint16_t y;
	uint16_t rotation;
	uint8_t flags;
};

struct McuCommandCommand {
	uint8_t mouse_lr;
	uint16_t keyboard;
	float imu_yaw_rad;
};

struct McuDecodedFrame {
	uint8_t type;
	McuChassisCommand chassis;
	McuCommandCommand command;
};

bool mcu_decode_frame(const uint8_t *data, uint8_t length, McuDecodedFrame &frame);

void mcu_encode_chassis(const McuChassisCommand &command, uint8_t data[8]);

void mcu_encode_command(const McuCommandCommand &command, uint8_t data[8]);

void mcu_encode_autoaim(uint8_t data[8]);

void mcu_encode_referee(uint8_t data[8]);
