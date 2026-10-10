#include "icm42688p_hxy.hpp"

namespace
{
#define IMU_NODE DT_ALIAS(imu_spi)

    K_SEM_DEFINE(data_ready_sem, 0, 1);

    const gpio_dt_spec int1 = GPIO_DT_SPEC_GET(IMU_NODE, int1_gpios);
    const gpio_dt_spec int2 = GPIO_DT_SPEC_GET(IMU_NODE, int2_gpios);

    /**
     * @brief 读取 16 位大端数据并转为有符号值
     *
     * @param data 大端字节数据指针
     * @return 转换后的有符号 16 位值
    */
    int16_t read_i16_be(const uint8_t *data)
    {
        const uint16_t value = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
        return static_cast<int16_t>(value);
    }

    /**
     * @brief 数据就绪中断回调，释放就绪信号量
     *
    */
    void data_ready_callback(void *)
    {
        k_sem_give(&data_ready_sem);
    }
}

    /**
     * @brief 通过 SPI 从寄存器连续读取数据
     *
     * @param address 起始寄存器地址
     * @param data 读取数据输出缓冲区
     * @param length 读取字节数
     * @return 成功返回 0，失败返回负错误码
    */
    int icm42688phxy::read_registers(uint8_t address, uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0 || length + 1 > SPI_BUFFER_SIZE || address > 0x7F) {
        return -EINVAL;
    }

    uint8_t tx[SPI_BUFFER_SIZE]{};
    uint8_t rx[SPI_BUFFER_SIZE]{};
    tx[0] = static_cast<uint8_t>(address | ICM42688PHXY_SPI_READ_BIT);
    const int ret = bsp_spi_transceive(&spi_, tx, rx, length + 1);
    if (ret != 0) {
        return ret;
    }
    for (size_t index = 0; index < length; ++index) {
        data[index] = rx[index + 1];
    }
    return 0;
}

    /**
     * @brief 通过 SPI 写入单个寄存器
     *
     * @param address 寄存器地址
     * @param value 待写入的寄存器值
     * @return 成功返回 0，失败返回负错误码
    */
    int icm42688phxy::write_register(uint8_t address, uint8_t value)
{
    if (address > 0x7F) {
        return -EINVAL;
    }
    const uint8_t tx[2] = {address, value};
    uint8_t rx[sizeof(tx)]{};
    return bsp_spi_transceive(&spi_, tx, rx, sizeof(tx));
}

    /**
     * @brief 配置寄存器并回读校验，可带稳定等待时间
     *
     * @param address 寄存器地址
     * @param value 待写入的寄存器值
     * @param settle_ms 写完后的稳定等待毫秒数
     * @return 成功返回 0，读写或校验失败返回负错误码
    */
    int icm42688phxy::configure_register(uint8_t address, uint8_t value, uint32_t settle_ms)
{
    int ret = write_register(address, value);
    if (ret != 0) {
        return ret;
    }
    if (settle_ms != 0) {
        k_sleep(K_MSEC(settle_ms));
    }

    uint8_t readback = 0;
    ret = read_registers(address, &readback, 1);
    return ret != 0 ? ret : (readback == value ? 0 : -EIO);
}

    /**
     * @brief 初始化 ICM42688P，完成 SPI 配置、软复位、各子模块参数与数据就绪中断配置
     *
     * @return 成功返回 0，参数或初始化失败返回负错误码
    */
    int icm42688phxy::init()
{
    if (initialized_) {
        return 0;
    }
    if (!device_is_ready(int1.port) || !device_is_ready(int2.port)) {
        return -ENODEV;
    }

    int ret = bsp_spi_init(&spi_, DEVICE_DT_GET(DT_BUS(IMU_NODE)), static_cast<uint16_t>(DT_REG_ADDR(IMU_NODE)), DT_PROP(IMU_NODE, spi_max_frequency), SPI_MODE_CPOL | SPI_MODE_CPHA);
    if (ret != 0) {
        return ret;
    }

    k_busy_wait(3000);
    if (write_register(ICM42688PHXY_REG_SOFT_RST, ICM42688PHXY_SOFT_RST_VALUE) != 0) {
        return -EIO;
    }
    k_sleep(K_MSEC(50));

    uint8_t who_am_i = 0;
    if (read_registers(ICM42688PHXY_REG_WHO_AM_I, &who_am_i, 1) != 0 || who_am_i != ICM42688PHXY_WHO_AM_I_VALUE) {
        return -ENODEV;
    }

    if (configure_register(ICM42688PHXY_REG_COM_CFG, ICM42688PHXY_COM_CFG_BDU_AUTO_INC, 1) != 0 ||
        configure_register(ICM42688PHXY_REG_ACC_CONF, ICM42688PHXY_ACC_CONF_200HZ, 1) != 0 ||
        configure_register(ICM42688PHXY_REG_ACC_RANGE, ICM42688PHXY_ACC_RANGE_16G, 1) != 0 ||
        configure_register(ICM42688PHXY_REG_GYR_CONF, ICM42688PHXY_GYR_CONF_200HZ, 1) != 0 ||
        configure_register(ICM42688PHXY_REG_GYR_RANGE, ICM42688PHXY_GYR_RANGE_2000DPS, 1) != 0) {
        return -EIO;
    }

    if (write_register(ICM42688PHXY_REG_PWR_CTRL, ICM42688PHXY_PWR_OFF) != 0) {
        return -EIO;
    }
    k_sleep(K_MSEC(2));
    if (configure_register(ICM42688PHXY_REG_GYR_CONF, ICM42688PHXY_GYR_CONF_NOISE_OPT, 1) != 0 || write_register(ICM42688PHXY_REG_PWR_CTRL, ICM42688PHXY_PWR_ALL_ON) != 0) {
        return -EIO;
    }
    k_sleep(K_MSEC(50));

    uint8_t status = 0;
    if (read_registers(ICM42688PHXY_REG_DATA_STAT, &status, 1) != 0 || (status & (ICM42688PHXY_DATA_STAT_GYR_CONF_ERR | ICM42688PHXY_DATA_STAT_ACC_CONF_ERR)) != 0) {
        return -EIO;
    }

    ret = bsp_gpio_exti_init(&int1_irq_, &int1, GPIO_INT_EDGE_TO_ACTIVE, data_ready_callback, nullptr);
    if (ret != 0) {
        return ret;
    }
    ret = configure_register(ICM42688PHXY_REG_INT_CFG1, ICM42688PHXY_INT1_DRDY_ACCEL, 1);
    if (ret != 0) {
        return ret;
    }

    initialized_ = true;
    return 0;
}

    /**
     * @brief 等待传感器数据就绪
     *
     * @param timeout 等待超时
     * @return 成功返回 0，超时返回 -EAGAIN，未初始化返回 -ENODEV
    */
    int icm42688phxy::wait_data_ready(k_timeout_t timeout)
{
    if (!initialized_) {
        return -ENODEV;
    }
    return k_sem_take(&data_ready_sem, timeout);
}

    /**
     * @brief 读取一帧加速度/陀螺仪/温度原始数据
     *
     * @param sample 输出的采样数据
     * @return 成功返回 0，数据未就绪返回 -EAGAIN，失败返回负错误码
    */
    int icm42688phxy::read_sample(icm42688phxy_sample &sample)
{
    if (!initialized_) {
        return -ENODEV;
    }

    constexpr size_t temp_offset = ICM42688PHXY_REG_TEMP_H - ICM42688PHXY_REG_DATA_STAT;
    constexpr size_t burst_length = temp_offset + ICM42688PHXY_TEMP_FRAME_SIZE;
    uint8_t frame[burst_length]{};
    int ret = read_registers(ICM42688PHXY_REG_DATA_STAT, frame, burst_length);
    if (ret != 0) {
        return ret;
    }
    const uint8_t status = frame[0];
    if ((status & (ICM42688PHXY_DATA_STAT_GYR_CONF_ERR | ICM42688PHXY_DATA_STAT_ACC_CONF_ERR)) != 0) {
        return -EIO;
    }
    constexpr uint8_t ready_mask = ICM42688PHXY_DATA_STAT_ACC_READY | ICM42688PHXY_DATA_STAT_GYR_READY;
    if ((status & ready_mask) != ready_mask) {
        return -EAGAIN;
    }

    uint8_t decoded_frame[frame_size]{};
    memcpy(decoded_frame, &frame[1], ICM42688PHXY_SENSOR_FRAME_SIZE);
    memcpy(&decoded_frame[ICM42688PHXY_SENSOR_FRAME_SIZE], &frame[temp_offset], ICM42688PHXY_TEMP_FRAME_SIZE);
    return decode_frame(decoded_frame, frame_size, sample);
}

    /**
     * @brief 解析原始数据帧，解出加速度/陀螺仪/温度值
     *
     * @param frame 待解析的原始字节帧
     * @param length 帧长度
     * @param sample 输出的解析结果
     * @return 成功返回 0，长度不符返回 -EINVAL
    */
    int icm42688phxy::decode_frame(const uint8_t *frame, size_t length, icm42688phxy_sample &sample)
{
    if (frame == nullptr || length != frame_size) {
        return -EINVAL;
    }

    icm42688phxy_sample decoded{};
    for (size_t axis = 0; axis < 3; ++axis) {
        decoded.accel_raw[axis] = read_i16_be(&frame[axis * 2]);
        decoded.gyro_raw[axis] = read_i16_be(&frame[6 + axis * 2]);
    }
    decoded.temperature_raw = read_i16_be(&frame[ICM42688PHXY_SENSOR_FRAME_SIZE]);
    sample = decoded;
    return 0;
}