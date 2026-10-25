#include "icm42688p_hxy.hpp"

namespace
{
#define IMU_NODE DT_ALIAS(imu_spi)

int16_t ReadI16Be(const uint8_t *data)
{
	const uint16_t value = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
	return static_cast<int16_t>(value);
}
}

int Icm42688pHxy::ReadRegisters(uint8_t address, uint8_t *data, size_t length)
{
	if (data == nullptr || length == 0 || length + 1 > spi_buffer_size || address > 0x7F) {
		return -EINVAL;
	}

	uint8_t tx[spi_buffer_size] {};
	uint8_t rx[spi_buffer_size] {};
	tx[0] = static_cast<uint8_t>(address | Icm42688pRegisters::spi_read_bit);
	const int ret = bsp_spi_transceive(&_spi, tx, rx, length + 1);

	if (ret != 0) {
		return ret;
	}

	for (size_t index = 0; index < length; ++index) {
		data[index] = rx[index + 1];
	}

	return 0;
}

int Icm42688pHxy::WriteRegister(uint8_t address, uint8_t value)
{
	if (address > 0x7F) {
		return -EINVAL;
	}

	const uint8_t tx[2] = {address, value};
	uint8_t rx[sizeof(tx)] {};
	return bsp_spi_transceive(&_spi, tx, rx, sizeof(tx));
}

int Icm42688pHxy::ConfigureRegister(uint8_t address, uint8_t value, uint32_t settle_ms)
{
	int ret = WriteRegister(address, value);

	if (ret != 0) {
		return ret;
	}

	if (settle_ms != 0) {
		k_sleep(K_MSEC(settle_ms));
	}

	uint8_t readback = 0;
	ret = ReadRegisters(address, &readback, 1);
	return ret != 0 ? ret : (readback == value ? 0 : -EIO);
}

int Icm42688pHxy::Init()
{
	if (_initialized) {
		return 0;
	}

	int ret = bsp_spi_init(&_spi, DEVICE_DT_GET(DT_BUS(IMU_NODE)), static_cast<uint16_t>(DT_REG_ADDR(IMU_NODE)),
			       DT_PROP(IMU_NODE, spi_max_frequency), SPI_MODE_CPOL | SPI_MODE_CPHA);

	if (ret != 0) {
		return ret;
	}

	k_busy_wait(3000);

	if (WriteRegister(Icm42688pRegisters::soft_rst, Icm42688pRegisters::soft_rst_value) != 0) {
		return -EIO;
	}

	k_sleep(K_MSEC(50));

	uint8_t who_am_i = 0;

	if (ReadRegisters(Icm42688pRegisters::who_am_i, &who_am_i, 1) != 0 || who_am_i != Icm42688pRegisters::expected_id) {
		return -ENODEV;
	}

	if (ConfigureRegister(Icm42688pRegisters::com_cfg, Icm42688pRegisters::bdu_auto_inc, 1) != 0 ||
	    ConfigureRegister(Icm42688pRegisters::acc_conf, Icm42688pRegisters::acc_1600hz, 1) != 0 ||
	    ConfigureRegister(Icm42688pRegisters::acc_range, Icm42688pRegisters::acc_16g, 1) != 0 ||
	    ConfigureRegister(Icm42688pRegisters::gyr_conf, Icm42688pRegisters::gyr_1600hz, 1) != 0 ||
	    ConfigureRegister(Icm42688pRegisters::gyr_range, Icm42688pRegisters::gyr_2000dps, 1) != 0) {
		return -EIO;
	}

	if (WriteRegister(Icm42688pRegisters::pwr_ctrl, Icm42688pRegisters::pwr_off) != 0) {
		return -EIO;
	}

	k_sleep(K_MSEC(2));

	if (ConfigureRegister(Icm42688pRegisters::gyr_conf, Icm42688pRegisters::gyr_1600hz_noise_opt, 1) != 0
	    || WriteRegister(Icm42688pRegisters::pwr_ctrl, Icm42688pRegisters::pwr_all_on) != 0) {
		return -EIO;
	}

	k_sleep(K_MSEC(50));

	uint8_t status = 0;

	if (ReadRegisters(Icm42688pRegisters::data_stat, &status, 1) != 0
	    || (status & (Icm42688pRegisters::gyr_conf_error | Icm42688pRegisters::acc_conf_error)) != 0) {
		return -EIO;
	}

	_initialized = true;
	return 0;
}

int Icm42688pHxy::ReadSample(Icm42688pHxySample &sample)
{
	if (!_initialized) {
		return -ENODEV;
	}

	constexpr size_t temp_offset = Icm42688pRegisters::temp_h - Icm42688pRegisters::data_stat;
	constexpr size_t burst_length = temp_offset + Icm42688pRegisters::temp_frame_size;
	uint8_t frame[burst_length] {};
	int ret = ReadRegisters(Icm42688pRegisters::data_stat, frame, burst_length);

	if (ret != 0) {
		return ret;
	}

	const uint8_t status = frame[0];

	if ((status & (Icm42688pRegisters::gyr_conf_error | Icm42688pRegisters::acc_conf_error)) != 0) {
		return -EIO;
	}

	constexpr uint8_t ready_mask = Icm42688pRegisters::acc_ready | Icm42688pRegisters::gyr_ready;

	if ((status & ready_mask) != ready_mask) {
		return -EAGAIN;
	}

	uint8_t decoded_frame[frame_size] {};
	memcpy(decoded_frame, &frame[1], Icm42688pRegisters::sensor_frame_size);
	memcpy(&decoded_frame[Icm42688pRegisters::sensor_frame_size], &frame[temp_offset], Icm42688pRegisters::temp_frame_size);
	return DecodeFrame(decoded_frame, frame_size, sample);
}

int Icm42688pHxy::DecodeFrame(const uint8_t *frame, size_t length, Icm42688pHxySample &sample)
{
	if (frame == nullptr || length != frame_size) {
		return -EINVAL;
	}

	for (size_t axis = 0; axis < 3; ++axis) {
		sample.accel_raw[axis] = ReadI16Be(&frame[axis * 2]);
		sample.gyro_raw[axis] = ReadI16Be(&frame[6 + axis * 2]);
	}

	sample.temperature_raw = ReadI16Be(&frame[12]);
	return 0;
}
