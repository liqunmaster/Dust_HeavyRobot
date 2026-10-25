#pragma once

#include <stddef.h>
#include <stdint.h>

struct Icm42688pRegisters {
	static constexpr uint8_t who_am_i = 0x01;
	static constexpr uint8_t com_cfg = 0x05;
	static constexpr uint8_t data_stat = 0x0B;
	static constexpr uint8_t temp_h = 0x22;
	static constexpr uint8_t acc_conf = 0x40;
	static constexpr uint8_t acc_range = 0x41;
	static constexpr uint8_t gyr_conf = 0x42;
	static constexpr uint8_t gyr_range = 0x43;
	static constexpr uint8_t soft_rst = 0x4A;
	static constexpr uint8_t pwr_ctrl = 0x7D;

	static constexpr uint8_t expected_id = 0x6A;
	static constexpr uint8_t spi_read_bit = 0x80;
	static constexpr uint8_t bdu_auto_inc = 0x50;
	static constexpr uint8_t soft_rst_value = 0xA5;
	static constexpr uint8_t pwr_off = 0x00;
	static constexpr uint8_t pwr_all_on = 0x0F;
	static constexpr uint8_t acc_1600hz = 0x8C;
	static constexpr uint8_t gyr_1600hz = 0x8C;
	static constexpr uint8_t gyr_1600hz_noise_opt = 0xCC;
	static constexpr uint8_t acc_16g = 0x03;
	static constexpr uint8_t gyr_2000dps = 0x00;

	static constexpr uint8_t gyr_conf_error = 1U << 5;
	static constexpr uint8_t acc_conf_error = 1U << 4;
	static constexpr uint8_t gyr_ready = 1U << 1;
	static constexpr uint8_t acc_ready = 1U << 0;

	static constexpr size_t sensor_frame_size = 12;
	static constexpr size_t temp_frame_size = 2;
};
