#include "bsp_spi.hpp"

namespace
{
    SpiDmaBuffers dma_buffers __attribute__((section("AHB_SRAM"), aligned(4)));
    K_MUTEX_DEFINE(dma_lock);

    int BspSpiCheckReady(const Spi *spi, const uint8_t *data, size_t length)
    {
        if (spi == nullptr || !spi->initialized || spi->device == nullptr || data == nullptr || length == 0) {
            return -EINVAL;
        }
        return 0;
    }

    int BspSpiTransferLocked(Spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
    {
        if (length > spi_buffer_size) {
            return -EMSGSIZE;
        }

        k_mutex_lock(&dma_lock, K_FOREVER);

        memcpy(dma_buffers.tx, tx_data, length);

        struct spi_buf tx_buf{dma_buffers.tx, length};
        struct spi_buf rx_buf{dma_buffers.rx, length};
        struct spi_buf_set tx_set{&tx_buf, 1};
        struct spi_buf_set rx_set{&rx_buf, 1};
        const int ret = spi_transceive(spi->device, &spi->config, &tx_set, rx_data != nullptr ? &rx_set : nullptr);

        if (ret != 0) {
            k_mutex_unlock(&dma_lock);
            return ret;
        }

        if (rx_data != nullptr) {
            memcpy(rx_data, dma_buffers.rx, length);
        }

        k_mutex_unlock(&dma_lock);
        return 0;
    }
}

int bsp_spi_init(Spi *spi, const struct device *device, uint16_t peripheral, uint32_t frequency, uint16_t operation)
{
    if (spi == nullptr || device == nullptr || frequency == 0 || !device_is_ready(device)) {
        return -EINVAL;
    }

    if (spi->initialized) {
        return spi->device == device ? 0 : -EBUSY;
    }

    spi->device = device;
    memset(&spi->config, 0, sizeof(spi->config));
    spi->config.frequency = frequency;
    spi->config.operation = operation | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi->config.peripheral = peripheral;
    k_mutex_init(&spi->lock);
    ring_buffer_init(&spi->tx_buffer, spi->tx_storage, sizeof(spi->tx_storage));
    ring_buffer_init(&spi->rx_buffer, spi->rx_storage, sizeof(spi->rx_storage));
    spi->initialized = true;
    return 0;
}

int bsp_spi_write(Spi *spi, const uint8_t *data, size_t length)
{
    const int valid = BspSpiCheckReady(spi, data, length);

    if (valid != 0) {
        return valid;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = BspSpiTransferLocked(spi, data, nullptr, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

int bsp_spi_read(Spi *spi, uint8_t *data, size_t length)
{
    const int valid = BspSpiCheckReady(spi, data, length);

    if (valid != 0) {
        return valid;
    }

    uint8_t dummy[spi_buffer_size]{};
    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = BspSpiTransferLocked(spi, dummy, data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

int bsp_spi_transceive(Spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    const int valid = BspSpiCheckReady(spi, tx_data, length);

    if (valid != 0 || rx_data == nullptr) {
        return valid != 0 ? valid : -EINVAL;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = BspSpiTransferLocked(spi, tx_data, rx_data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

void bsp_spi_clear_buffers(Spi *spi)
{
    if (spi == nullptr || !spi->initialized) {
        return;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    ring_buffer_clear(&spi->tx_buffer);
    ring_buffer_clear(&spi->rx_buffer);
    k_mutex_unlock(&spi->lock);
}
