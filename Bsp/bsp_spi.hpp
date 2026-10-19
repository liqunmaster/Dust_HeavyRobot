#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "ring_buffer.hpp"

inline constexpr size_t spi_buffer_size = 64;

struct Spi
{
    const struct device *device;
    struct spi_config config;
    struct k_mutex lock;
    RingBuffer tx_buffer;
    RingBuffer rx_buffer;
    uint8_t tx_storage[spi_buffer_size] __aligned(4);
    uint8_t rx_storage[spi_buffer_size] __aligned(4);
    bool initialized;
};

int bsp_spi_init(Spi *spi, const struct device *device, uint16_t peripheral, uint32_t frequency, uint16_t operation);

int bsp_spi_write(Spi *spi, const uint8_t *data, size_t length);

int bsp_spi_read(Spi *spi, uint8_t *data, size_t length);

int bsp_spi_transceive(Spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length);

void bsp_spi_clear_buffers(Spi *spi);

struct SpiDmaBuffers
{
    uint8_t tx[spi_buffer_size];
    uint8_t rx[spi_buffer_size];
};
