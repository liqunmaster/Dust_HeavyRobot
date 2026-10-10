#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "bsp_gpio.h"
#include "bsp_spi.hpp"
#include "icm42688p_hxy_reg.h"

// ICM42688P 惯性传感器采样数据，包含加速度计/陀螺仪与温度原始值
struct icm42688phxy_sample
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
};

// ICM42688P 六轴惯性传感器驱动类，基于 SPI 通信与外部中断
class icm42688phxy final
{
    public:
    static constexpr size_t frame_size = ICM42688PHXY_SENSOR_FRAME_SIZE + ICM42688PHXY_TEMP_FRAME_SIZE;

    int init();
    int wait_data_ready(k_timeout_t timeout);
    int read_sample(icm42688phxy_sample &sample);

    static int decode_frame(const uint8_t *frame, size_t length, icm42688phxy_sample &sample);

    private:
    int read_registers(uint8_t address, uint8_t *data, size_t length);
    int write_register(uint8_t address, uint8_t value);
    int configure_register(uint8_t address, uint8_t value, uint32_t settle_ms);

    spi spi_{};

    bsp_gpio_irq int1_irq_{};

    bool initialized_{false};
};
