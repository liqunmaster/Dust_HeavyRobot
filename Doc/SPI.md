# HPM5361 SPI 使用说明

## 当前调用链

`Device/imu/icm42688p_hxy` 只负责 ICM-42688P-HXY 的寄存器配置和数据解包，`Bsp/bsp_spi` 负责提供带互斥保护的 SPI 传输接口。设备层不直接访问 HPM 寄存器，也不在任务中轮询 SPI 状态。

`Bsp/bsp_spi/spi.conf` 当前启用了：

```conf
CONFIG_SPI=y
CONFIG_DMA=y
CONFIG_SPI_HPM_SPI_DMA=y
```

Zephyr 的 `spi_transceive()` 是同步接口：调用者在一次传输完成后返回，DMA 完成和 SPI 结束由底层驱动处理中断。应用层没有读取 `SPIACTIVE` 的循环。`Bsp/bsp_spi` 的全局 DMA 缓冲区放在 `AHB_SRAM` 并按 4 字节对齐，多个 SPI 设备通过互斥量串行使用它。

## ICM-42688P-HXY 传输

传感器的状态、加速度、陀螺仪和温度寄存器现在使用一次连续窗口读取：从 `DATA_STAT(0x0B)` 读到 `TEMP_L(0x23)`，设备层只取其中的有效字段。这样每个数据就绪中断只需要一次 SPI DMA 事务，避免状态、运动数据、温度分别启动事务。

ICM 的 `INT1` 通过 `Bsp/bsp_gpio` 配置为边沿中断，回调只释放信号量；IMU 线程被信号量唤醒后再读取数据。`INT2` 保留为设备就绪检查和后续扩展接口。

## 修改底层驱动时的注意事项

当前工程只修改了应用侧 SPI 封装，没有覆盖外部 SDK 文件。若以后修改 `D:/MCU/hpm/zephyr_sdk_glue/drivers/spi/spi_hpmicro.c`，应记录 SDK 版本和补丁，避免 SDK 更新后丢失修改。不要在应用层重新加入 `SPIACTIVE` 轮询；若要改为真正异步 API，应同时提供完成信号量、超时、错误复位和片选释放路径，并对 HPM 的 DMA 通道完成事件做板级测试。

## 负载判断

CPU 负载应在关闭 IMU 调试浮点打印、USB 空转线程和 UART 发送轮询后重新测量。当前 `CONFIG_IMU_DEBUG_OUTPUT` 默认关闭，USB 线程由 USB 中断信号量唤醒，UART 发送等待 TX 完成信号量；历史文档中的百分比只作为旧固件记录，不能直接代表当前版本。
