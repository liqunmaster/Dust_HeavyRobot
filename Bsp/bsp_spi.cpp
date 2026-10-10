#include "bsp_spi.hpp"

namespace
{

    static uint8_t dma_tx[SPI_BUFFER_SIZE] __attribute__((section("AHB_SRAM"), aligned(4)));
    static uint8_t dma_rx[SPI_BUFFER_SIZE] __attribute__((section("AHB_SRAM"), aligned(4)));
    K_MUTEX_DEFINE(dma_lock);

    /**
     * @brief 校验 SPI 实例与收发参数是否就绪合法
     *
     * @param spi SPI 实例
     * @param data 数据缓冲区
     * @param length 数据长度
     * @return 合法返回 0 否则返回负错误码
     */
    int bsp_spi_check_ready(const spi *spi, const uint8_t *data, size_t length)
    {
        if (spi == nullptr || !spi->initialized || spi->device == nullptr || data == nullptr || length == 0) {
            return -EINVAL;
        }
        return 0;
    }

    /**
     * @brief 持锁执行一次 SPI 传输
     *
     * @param spi SPI 实例
     * @param tx_data 待发送数据
     * @param rx_data 接收数据缓冲区 可为空
     * @param length 传输长度
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_spi_transfer_locked(spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
    {
        if (length > SPI_BUFFER_SIZE) {
            return -EMSGSIZE;
        }

        k_mutex_lock(&dma_lock, K_FOREVER);

        memcpy(dma_tx, tx_data, length);

        struct spi_buf tx_buf{dma_tx, length};
        struct spi_buf rx_buf{dma_rx, length};
        struct spi_buf_set tx_set{&tx_buf, 1};
        struct spi_buf_set rx_set{&rx_buf, 1};
        const int ret = spi_transceive(spi->device, &spi->config, &tx_set, rx_data != nullptr ? &rx_set : nullptr);

        if (ret != 0) {
            k_mutex_unlock(&dma_lock);
            return ret;
        }

        if (rx_data != nullptr) {
            memcpy(rx_data, dma_rx, length);
        }

        k_mutex_unlock(&dma_lock);
        return 0;
    }

}

/**
 * @brief 初始化 SPI 实例 配置频率、操作模式与外设
 *
 * @param spi SPI 实例
 * @param device Zephyr SPI 设备句柄
 * @param peripheral 外设编号
 * @param frequency 时钟频率
 * @param operation 操作模式位
 * @return 成功返回 0 失败返回负错误码
 */
int bsp_spi_init(spi *spi, const struct device *device, uint16_t peripheral, uint32_t frequency, uint16_t operation)
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

/**
 * @brief 向 SPI 设备发送数据
 *
 * @param spi SPI 实例
 * @param data 待发送数据
 * @param length 数据长度
 * @return 成功返回 0 失败返回负错误码
 */
int bsp_spi_write(spi *spi, const uint8_t *data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, data, length);

    if (valid != 0) {
        return valid;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, data, nullptr, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

/**
 * @brief 从 SPI 设备读取数据
 *
 * @param spi SPI 实例
 * @param data 接收数据缓冲区
 * @param length 数据长度
 * @return 成功返回 0 失败返回负错误码
 */
int bsp_spi_read(spi *spi, uint8_t *data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, data, length);

    if (valid != 0) {
        return valid;
    }

    uint8_t dummy[SPI_BUFFER_SIZE]{};
    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, dummy, data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

/**
 * @brief 同时发送并接收 SPI 数据
 *
 * @param spi SPI 实例
 * @param tx_data 待发送数据
 * @param rx_data 接收数据缓冲区
 * @param length 数据长度
 * @return 成功返回 0 失败返回负错误码
 */
int bsp_spi_transceive(spi *spi, const uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    const int valid = bsp_spi_check_ready(spi, tx_data, length);

    if (valid != 0 || rx_data == nullptr) {
        return valid != 0 ? valid : -EINVAL;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    const int ret = bsp_spi_transfer_locked(spi, tx_data, rx_data, length);
    k_mutex_unlock(&spi->lock);
    return ret;
}

/**
 * @brief 清空 SPI 实例的发送与接收缓冲
 *
 * @param spi SPI 实例
 */
void bsp_spi_clear_buffers(spi *spi)
{
    if (spi == nullptr || !spi->initialized) {
        return;
    }

    k_mutex_lock(&spi->lock, K_FOREVER);
    ring_buffer_clear(&spi->tx_buffer);
    ring_buffer_clear(&spi->rx_buffer);
    k_mutex_unlock(&spi->lock);
}
