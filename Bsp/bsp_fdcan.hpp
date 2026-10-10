#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C"
{
#endif

constexpr size_t fdcan_max_data_length = 64;

    // 超时描述结构体，用于指定 FDCAN 发送/恢复操作的等待时间
    typedef struct
    {
        int32_t milliseconds;
    } fdcan_timeout;

    // FDCAN 通道枚举，标识具体的 CAN 外设和数量
    typedef enum
    {
        FDCAN_DEVICE_CAN0 = 0,
        FDCAN_DEVICE_CAN1 = 1,
        FDCAN_DEVICE_CAN2 = 2,
        FDCAN_DEVICE_CAN3 = 3,
        FDCAN_DEVICE_COUNT = 4,
    } fdcan_device;

    // FDCAN 帧协议枚举，区分经典 CAN 与 CAN-FD
    typedef enum
    {
        FDCAN_PROTOCOL_CLASSIC = 0,
        FDCAN_PROTOCOL_FD = 1,
    } fdcan_protocol;

    // FDCAN 帧 ID 类型枚举，区分标准帧与扩展帧
    typedef enum
    {
        FDCAN_ID_STANDARD = 0,
        FDCAN_ID_EXTENDED = 1,
    } fdcan_id_type;

    // 比特率切换开关枚举，仅在 CAN-FD 生效
    typedef enum
    {
        FDCAN_BRS_DISABLED = 0,
        FDCAN_BRS_ENABLED = 1,
    } fdcan_brs;

    // FDCAN 工作模式枚举
    typedef enum
    {
        FDCAN_MODE_NORMAL = 0,
        FDCAN_MODE_LOOPBACK = 1,
        FDCAN_MODE_LISTEN_ONLY = 2,
    } fdcan_mode;

    // 重发机制枚举，是否允许控制器自动重发报文
    typedef enum
    {
        FDCAN_RETRANSMISSION_ENABLED = 0,
        FDCAN_RETRANSMISSION_DISABLED = 1,
    } fdcan_retransmission;

    // FDCAN 总线状态枚举
    typedef enum
    {
        FDCAN_BUS_ERROR_ACTIVE = 0,
        FDCAN_BUS_ERROR_WARNING = 1,
        FDCAN_BUS_ERROR_PASSIVE = 2,
        FDCAN_BUS_OFF = 3,
        FDCAN_BUS_STOPPED = 4,
    } fdcan_bus_state;

    // FDCAN 恢复流程状态枚举，用于查询总线错误后的恢复进度
    typedef enum
    {
        FDCAN_RECOVERY_IDLE = 0,
        FDCAN_RECOVERY_STOPPING = 1,
        FDCAN_RECOVERY_CLEARING_TX = 2,
        FDCAN_RECOVERY_RECONFIGURING = 3,
        FDCAN_RECOVERY_RESTARTING = 4,
        FDCAN_RECOVERY_SUCCEEDED = 5,
        FDCAN_RECOVERY_FAILED = 6,
    } fdcan_recovery_state;

    // FDCAN 发送帧结构体，描述待发送报文的完整信息
    typedef struct
    {
        uint32_t id;
        fdcan_id_type id_type;
        fdcan_protocol protocol;
        fdcan_brs bitrate_switch;
        uint8_t length;
        uint8_t data[fdcan_max_data_length];
    } fdcan_frame;

    // FDCAN 配置结构体，描述工作模式与重发策略
    typedef struct
    {
        fdcan_mode mode;
        fdcan_retransmission retransmission;
    } fdcan_config;

    // FDCAN 错误计数结构体，记录发送/接收错误次数
    typedef struct
    {
        uint8_t transmit;
        uint8_t receive;
    } fdcan_error_count;

    // FDCAN 统计信息结构体，记录收发与恢复等多类计数
    typedef struct
    {
        uint32_t tx_queued;
        uint32_t tx_completed;
        uint32_t tx_dropped;
        uint32_t tx_errors;
        uint32_t rx_received;
        uint32_t rx_dropped;
        uint32_t recovery_succeeded;
        uint32_t recovery_failed;
    } fdcan_statistics;

    int bsp_fdcan_init(fdcan_device device, const fdcan_config *config);

    int bsp_fdcan_deinit(fdcan_device device);

    int bsp_fdcan_transmit(fdcan_device device, const fdcan_frame *frame, fdcan_timeout timeout);

    // FDCAN 接收报文视图结构体，data 指针仅在本回调返回前有效
    typedef struct
    {
        uint32_t id;
        fdcan_id_type id_type;
        fdcan_protocol protocol;
        uint8_t length;
        const uint8_t *data;
    } fdcan_rx_view;

    // The data pointer is valid only until the callback returns.
    typedef void (*fdcan_rx_callback_t)(fdcan_device device, const fdcan_rx_view *frame, void *user_data);

    int bsp_fdcan_set_rx_callback(fdcan_device device, fdcan_rx_callback_t callback, void *user_data);

    // 发送完成回调类型，error 为 0 表示发送成功
    typedef void (*fdcan_tx_callback_t)(fdcan_device device, int error, void *user_data);

    int bsp_fdcan_set_tx_callback(fdcan_device device, fdcan_tx_callback_t callback, void *user_data);

    int bsp_fdcan_recover(fdcan_device device, fdcan_timeout timeout);

    int bsp_fdcan_get_state(fdcan_device device, fdcan_bus_state *state, fdcan_error_count *error_count);

    fdcan_recovery_state bsp_fdcan_get_recovery_state(fdcan_device device);

    void bsp_fdcan_get_statistics(fdcan_device device, fdcan_statistics *statistics);

    void bsp_fdcan_clear_statistics(fdcan_device device);

    bool bsp_fdcan_is_ready(fdcan_device device);

    uint32_t bsp_fdcan_get_data_bitrate(fdcan_device device);

    size_t bsp_fdcan_tx_pending(fdcan_device device);

#ifdef __cplusplus
}
#endif

constexpr fdcan_timeout fdcan_no_wait{ 0 };

constexpr fdcan_timeout fdcan_wait_forever{ -1 };

    /**
     * @brief 将毫秒数转换为 fdcan_timeout 超时结构体
     *
     * @param ms 超时毫秒数，-1 表示无限等待
     * @return 对应的 fdcan_timeout 结构体
    */
    constexpr fdcan_timeout fdcan_msec(int32_t ms)
    {
        return fdcan_timeout{ ms };
    }