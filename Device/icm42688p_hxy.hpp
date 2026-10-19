#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "bsp_spi.hpp"
#include "icm42688p_hxy_reg.hpp"

struct Icm42688pHxySample
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
};

class Icm42688pHxy final
{
    public:
    static constexpr size_t frame_size = Icm42688pRegisters::sensor_frame_size + Icm42688pRegisters::temp_frame_size;

    int Init();
    int ReadSample(Icm42688pHxySample &sample);

    static int DecodeFrame(const uint8_t *frame, size_t length, Icm42688pHxySample &sample);

    private:
    int ReadRegisters(uint8_t address, uint8_t *data, size_t length);
    int WriteRegister(uint8_t address, uint8_t value);
    int ConfigureRegister(uint8_t address, uint8_t value, uint32_t settle_ms);

    Spi spi_{};

    bool initialized_{false};
};
