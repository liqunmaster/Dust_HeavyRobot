#include "bsp_fdcan.hpp"

namespace
{
    struct fdcan_atomic_statistics
    {
        atomic_t tx_queued;
        atomic_t tx_completed;
        atomic_t tx_dropped;
        atomic_t tx_errors;
        atomic_t rx_received;
        atomic_t rx_dropped;
        atomic_t recovery_succeeded;
        atomic_t recovery_failed;
    };

    // FDCAN 运行上下文结构体 保存某一通道的设备句柄、配置、统计与回调等信息
    struct fdcan_context
    {
        fdcan_device id;
        const struct device *device;
        fdcan_config config;
        uint32_t bitrate;
        uint16_t sample_point;
        uint32_t data_bitrate;
        uint16_t data_sample_point;
        bool fd_enabled;
        fdcan_atomic_statistics statistics;
        atomic_t tx_active;
        atomic_t recovering;
        atomic_t recovery_state;
        fdcan_rx_callback_t rx_callback;
        void *rx_user_data;
        fdcan_tx_callback_t tx_callback;
        void *tx_user_data;
        int standard_filter_id;
        int extended_filter_id;
        bool initialized;
    };

    static fdcan_context contexts[FDCAN_DEVICE_COUNT];

    /**
     * @brief 校验 CAN 通道编号是否合法
     *
     * @param device CAN 通道编号
     * @return 合法返回 true 否则返回 false
     */
    static bool valid_device(fdcan_device device)
    {
        return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
    }

    /**
     * @brief 根据通道编号获取对应的 Zephyr 设备句柄
     *
     * @param device CAN 通道编号
     * @return 对应设备的句柄 找不到时返回 nullptr
     */
    static const struct device *device_from_id(fdcan_device device)
    {
        switch (device) {
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
            case FDCAN_DEVICE_CAN0:
            return DEVICE_DT_GET(DT_NODELABEL(can0));
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
            case FDCAN_DEVICE_CAN1:
            return DEVICE_DT_GET(DT_NODELABEL(can1));
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
            case FDCAN_DEVICE_CAN2:
            return DEVICE_DT_GET(DT_NODELABEL(can2));
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
            case FDCAN_DEVICE_CAN3:
            return DEVICE_DT_GET(DT_NODELABEL(can3));
        #endif
            default:
            return nullptr;
        }
    }

    /**
     * @brief 将内部配置转换为 Zephyr 的 can_mode_t 模式位
     *
     * @param context FDCAN 运行上下文 读取其工作模式与重发配置
     * @return Zephyr CAN 模式位掩码
     */
    static can_mode_t to_can_mode(const fdcan_context &context)
    {
        can_mode_t mode = 0;

        if (context.config.mode == FDCAN_MODE_LOOPBACK) {
            mode |= CAN_MODE_LOOPBACK;
        } else if (context.config.mode == FDCAN_MODE_LISTEN_ONLY) {
            mode |= CAN_MODE_LISTENONLY;
        }
        if (context.config.retransmission == FDCAN_RETRANSMISSION_DISABLED) {
            mode |= CAN_MODE_ONE_SHOT;
        }
    #if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
        mode |= CAN_MODE_MANUAL_RECOVERY;
    #endif
    #if defined(CONFIG_CAN_FD_MODE)
        if (context.fd_enabled) {
            mode |= CAN_MODE_FD;
        }
    #endif
        return mode;
    }

    /**
     * @brief 校验 FDCAN 初始化配置参数是否合法
     *
     * @param config 待校验的配置指针
     * @return 合法返回 0 非法返回负错误码
     */
    static int validate_config(const fdcan_config *config)
    {
        if (config == nullptr || config->mode > FDCAN_MODE_LISTEN_ONLY || config->retransmission > FDCAN_RETRANSMISSION_DISABLED) {
            return -EINVAL;
        }
        return 0;
    }

    /**
     * @brief 校验待发送报文的内容是否合法
     *
     * @param context FDCAN 运行上下文 用于判断是否启用 CAN-FD
     * @param frame 待校验的报文字段
     * @return 合法返回 0 非法返回负错误码
     */
    static int validate_frame(const fdcan_context &context, const fdcan_frame *frame)
    {
        if (frame == nullptr || frame->id_type > FDCAN_ID_EXTENDED || frame->protocol > FDCAN_PROTOCOL_FD || frame->bitrate_switch > FDCAN_BRS_ENABLED) {
            return -EINVAL;
        }
        if ((frame->id_type == FDCAN_ID_STANDARD && frame->id > CAN_STD_ID_MASK) || (frame->id_type == FDCAN_ID_EXTENDED && frame->id > CAN_EXT_ID_MASK)) {
            return -EINVAL;
        }
        if (frame->protocol == FDCAN_PROTOCOL_CLASSIC) {
            if (frame->length > 8 || frame->bitrate_switch != FDCAN_BRS_DISABLED) {
                return -EMSGSIZE;
            }
        } else {
            const bool valid_fd_length = frame->length <= 8 || frame->length == 12 || frame->length == 16 || frame->length == 20 || frame->length == 24 || frame->length == 32 || frame->length == 48 || frame->length == 64;
            if (!context.fd_enabled || !valid_fd_length) {
                return -EMSGSIZE;
            }
        }
        return 0;
    }

    /**
     * @brief 将内部报文字段转换为 Zephyr 的 can_frame 结构
     *
     * @param source 内部 FDCAN 帧
     * @param destination 输出到 Zephyr can_frame 结构
     */
    static void to_can_frame(const fdcan_frame &source, struct can_frame *destination)
    {
        memset(destination, 0, sizeof(*destination));
        destination->id = source.id;
        destination->dlc = can_bytes_to_dlc(source.length);
        if (source.id_type == FDCAN_ID_EXTENDED) {
            destination->flags |= CAN_FRAME_IDE;
        }
        if (source.protocol == FDCAN_PROTOCOL_FD) {
            destination->flags |= CAN_FRAME_FDF;
        }
        if (source.bitrate_switch == FDCAN_BRS_ENABLED) {
            destination->flags |= CAN_FRAME_BRS;
        }
        memcpy(destination->data, source.data, source.length);
    }

    /**
     * @brief Zephyr 发送完成回调 统计结果并调用用户发送回调
     *
     * @param device 回调对应的 CAN 设备
     * @param error 发送错误码 0 表示成功
     * @param user_data 用户上下文指针
     */
    static void tx_complete_callback(const struct device *device, int error, void *user_data)
    {
        fdcan_context *context = static_cast<fdcan_context *>(user_data);
        if (context == nullptr || context->device != device) {
            return;
        }
        if (atomic_set(&context->tx_active, 0) == 0) {
            return;
        }

        if (error == 0) {
            atomic_inc(&context->statistics.tx_completed);
        } else {
            atomic_inc(&context->statistics.tx_errors);
        }
        if (context->tx_callback != nullptr) {
            context->tx_callback(context->id, error, context->tx_user_data);
        }
    }

    /**
     * @brief Zephyr 接收回调 构造接收视图并调用用户接收回调
     *
     * @param device 回调对应的 CAN 设备
     * @param frame 接收到的 Zephyr can_frame
     * @param user_data 用户上下文指针
     */
    static void rx_callback(const struct device *device, struct can_frame *frame, void *user_data)
    {
        fdcan_context *context = static_cast<fdcan_context *>(user_data);
        if (context == nullptr || context->device != device || frame == nullptr || !context->initialized || atomic_get(&context->recovering)) {
            return;
        }

        const fdcan_rx_view received{
            frame->id,
            (frame->flags & CAN_FRAME_IDE) != 0 ? FDCAN_ID_EXTENDED : FDCAN_ID_STANDARD,
            (frame->flags & CAN_FRAME_FDF) != 0 ? FDCAN_PROTOCOL_FD : FDCAN_PROTOCOL_CLASSIC,
            static_cast<uint8_t>(can_dlc_to_bytes(frame->dlc)),
            frame->data,
        };

        atomic_inc(&context->statistics.rx_received);
        if (context->rx_callback != nullptr) {
            context->rx_callback(context->id, &received, context->rx_user_data);
        }
    }

    /**
     * @brief 添加默认的标准帧接收过滤器
     *
     * @param context FDCAN 运行上下文
     * @return 成功返回 0 失败返回负错误码
     */
    static int add_default_filters(fdcan_context *context)
    {
        const struct can_filter standard_filter = {
            .id = 0,
            .mask = 0,
            .flags = 0,
        };
        context->standard_filter_id = can_add_rx_filter(context->device, rx_callback, context, &standard_filter);
        if (context->standard_filter_id < 0) {
            return context->standard_filter_id;
        }

        context->extended_filter_id = -1;
        return 0;
    }

    /**
     * @brief 移除默认的接收过滤器
     *
     * @param context FDCAN 运行上下文
     */
    static void remove_default_filters(fdcan_context *context)
    {
        if (context->standard_filter_id >= 0) {
            can_remove_rx_filter(context->device, context->standard_filter_id);
            context->standard_filter_id = -1;
        }
        if (context->extended_filter_id >= 0) {
            can_remove_rx_filter(context->device, context->extended_filter_id);
            context->extended_filter_id = -1;
        }
    }

    /**
     * @brief 从设备树读取该通道的比特率等配置
     *
     * @param device CAN 通道编号
     * @param context FDCAN 运行上下文 输出配置结果
     * @return 成功返回 0 失败返回负错误码
     */
    static int read_dt_config(fdcan_device device, fdcan_context *context)
    {
        uint32_t bitrate = 0;
        uint16_t sample_point = 0;
        uint32_t data_bitrate = 0;
        uint16_t data_sample_point = 0;
        bool fd_enabled = false;

        switch (device) {
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
            case FDCAN_DEVICE_CAN0:
            bitrate = DT_PROP_OR(DT_NODELABEL(can0), bitrate, 0);
            sample_point = DT_PROP_OR(DT_NODELABEL(can0), sample_point, 0);
        #if defined(CONFIG_CAN_FD_MODE)
            fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can0), bitrate_data);
            data_bitrate = DT_PROP_OR(DT_NODELABEL(can0), bitrate_data, 0);
            data_sample_point = DT_PROP_OR(DT_NODELABEL(can0), sample_point_data, 0);
        #endif
            break;
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
            case FDCAN_DEVICE_CAN1:
            bitrate = DT_PROP_OR(DT_NODELABEL(can1), bitrate, 0);
            sample_point = DT_PROP_OR(DT_NODELABEL(can1), sample_point, 0);
        #if defined(CONFIG_CAN_FD_MODE)
            fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can1), bitrate_data);
            data_bitrate = DT_PROP_OR(DT_NODELABEL(can1), bitrate_data, 0);
            data_sample_point = DT_PROP_OR(DT_NODELABEL(can1), sample_point_data, 0);
        #endif
            break;
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
            case FDCAN_DEVICE_CAN2:
            bitrate = DT_PROP_OR(DT_NODELABEL(can2), bitrate, 0);
            sample_point = DT_PROP_OR(DT_NODELABEL(can2), sample_point, 0);
        #if defined(CONFIG_CAN_FD_MODE)
            fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can2), bitrate_data);
            data_bitrate = DT_PROP_OR(DT_NODELABEL(can2), bitrate_data, 0);
            data_sample_point = DT_PROP_OR(DT_NODELABEL(can2), sample_point_data, 0);
        #endif
            break;
        #endif
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
            case FDCAN_DEVICE_CAN3:
            bitrate = DT_PROP_OR(DT_NODELABEL(can3), bitrate, 0);
            sample_point = DT_PROP_OR(DT_NODELABEL(can3), sample_point, 0);
        #if defined(CONFIG_CAN_FD_MODE)
            fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can3), bitrate_data);
            data_bitrate = DT_PROP_OR(DT_NODELABEL(can3), bitrate_data, 0);
            data_sample_point = DT_PROP_OR(DT_NODELABEL(can3), sample_point_data, 0);
        #endif
            break;
        #endif
            default:
            return -EINVAL;
        }

        if (bitrate == 0) {
            return -EINVAL;
        }
        #if defined(CONFIG_CAN_FD_MODE)
            if (fd_enabled && data_bitrate == 0) {
                return -EINVAL;
            }
        #endif

        context->bitrate = bitrate;
        context->sample_point = sample_point;
        context->data_bitrate = data_bitrate;
        context->data_sample_point = data_sample_point;
        context->fd_enabled = fd_enabled;
        return 0;
    }

    /**
     * @brief 将上下文配置应用到 Zephyr CAN 驱动
     *
     * @param context FDCAN 运行上下文
     * @return 成功返回 0 失败返回负错误码
     */
    static int apply_configuration(fdcan_context *context)
    {
        int ret = can_set_mode(context->device, to_can_mode(*context));
        if (ret != 0) {
            return ret;
        }

        if (context->sample_point == 0) {
            ret = can_set_bitrate(context->device, context->bitrate);
        } else {
            struct can_timing timing{};
            ret = can_calc_timing(context->device, &timing, context->bitrate, context->sample_point);
            if (ret < 0) {
                return ret;
            }
            ret = can_set_timing(context->device, &timing);
        }
        if (ret != 0) {
            return ret;
        }

        #if defined(CONFIG_CAN_FD_MODE)
            if (context->fd_enabled) {
                if (context->data_sample_point == 0) {
                    ret = can_set_bitrate_data(context->device, context->data_bitrate);
                } else {
                    struct can_timing data_timing{};
                    ret = can_calc_timing_data(context->device, &data_timing, context->data_bitrate, context->data_sample_point);
                    if (ret < 0) {
                        return ret;
                    }
                    ret = can_set_timing_data(context->device, &data_timing);
                }
                if (ret != 0) {
                    return ret;
                }
            }
        #endif
        return 0;
    }

    /**
     * @brief 清空 FDCAN 原子统计计数
     *
     * @param statistics 待清零的统计结构体指针
     */
    static void clear_atomic_statistics(fdcan_atomic_statistics *statistics)
    {
        atomic_clear(&statistics->tx_queued);
        atomic_clear(&statistics->tx_completed);
        atomic_clear(&statistics->tx_dropped);
        atomic_clear(&statistics->tx_errors);
        atomic_clear(&statistics->rx_received);
        atomic_clear(&statistics->rx_dropped);
        atomic_clear(&statistics->recovery_succeeded);
        atomic_clear(&statistics->recovery_failed);
    }

}

extern "C"
{

    /**
     * @brief 初始化指定 FDCAN 通道
     *
     * @param device CAN 通道编号
     * @param config 通道配置 可为空指针表示使用默认配置
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_init(fdcan_device device, const fdcan_config *config)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        int ret = validate_config(config);
        if (ret != 0) {
            return ret;
        }

        fdcan_context &context = contexts[device];
        if (context.initialized) {
            return -EALREADY;
        }

        const struct device *fdcan_device = device_from_id(device);
        if (fdcan_device == nullptr || !device_is_ready(fdcan_device)) {
            return -ENODEV;
        }

        ret = read_dt_config(device, &context);
        if (ret != 0) {
            return ret;
        }

        context.id = device;
        context.device = fdcan_device;
        context.config = *config;
        context.standard_filter_id = -1;
        context.extended_filter_id = -1;
        context.rx_callback  = nullptr;
        context.rx_user_data = nullptr;
        context.tx_callback  = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_IDLE);
        clear_atomic_statistics(&context.statistics);

        ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            context.device   = nullptr;
            return ret;
        }
        ret = apply_configuration(&context);
        if (ret != 0) {
            context.device = nullptr;
            return ret;
        }
        ret = add_default_filters(&context);
        if (ret != 0) {
            context.device = nullptr;
            return ret;
        }
        ret = can_start(context.device);
        if (ret != 0 && ret != -EALREADY) {
            remove_default_filters(&context);
            context.device = nullptr;
            return ret;
        }

        context.initialized = true;
        return 0;
    }

    /**
     * @brief 反初始化指定 FDCAN 通道
     *
     * @param device CAN 通道编号
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_deinit(fdcan_device device)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        fdcan_context &context = contexts[device];
        if (!context.initialized) {
            return -EALREADY;
        }

        atomic_set(&context.recovering, 1);
        remove_default_filters(&context);
        int ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            atomic_clear(&context.recovering);
            return ret;
        }
        context.rx_callback  = nullptr;
        context.rx_user_data = nullptr;
        context.tx_callback  = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        context.initialized  = false;
        context.id = FDCAN_DEVICE_COUNT;
        context.device = nullptr;
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_IDLE);
        return 0;
    }

    /**
     * @brief 向指定 FDCAN 通道发送一帧报文
     *
     * @param device CAN 通道编号
     * @param frame 待发送的报文字段
     * @param timeout 超时设置 负数表示无限等待
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_transmit(fdcan_device device, const fdcan_frame *frame, fdcan_timeout timeout)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        fdcan_context &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        if (atomic_get(&context.recovering)) {
            atomic_inc(&context.statistics.tx_dropped);
            return -EBUSY;
        }

        const int ret = validate_frame(context, frame);
        if (ret != 0) {
            return ret;
        }

        if (!atomic_cas(&context.tx_active, 0, 1)) {
            atomic_inc(&context.statistics.tx_dropped);
            return -EAGAIN;
        }

        struct can_frame can_frame{};
        to_can_frame(*frame, &can_frame);
        const k_timeout_t zephyr_timeout = timeout.milliseconds < 0 ? K_FOREVER : K_MSEC(timeout.milliseconds);
        const int send_result = can_send(context.device, &can_frame, zephyr_timeout, tx_complete_callback, &context);
        if (send_result != 0) {
            atomic_inc(&context.statistics.tx_errors);
            atomic_clear(&context.tx_active);
            return send_result;
        }

        atomic_inc(&context.statistics.tx_queued);
        return 0;
    }

    /**
     * @brief 设置接收回调函数
     *
     * @param device CAN 通道编号
     * @param callback 接收回调函数指针
     * @param user_data 透传给回调的用户数据
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_set_rx_callback(fdcan_device device, fdcan_rx_callback_t callback, void *user_data)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        fdcan_context &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        context.rx_callback = callback;
        context.rx_user_data = user_data;
        return 0;
    }

    /**
     * @brief 设置发送完成回调函数
     *
     * @param device CAN 通道编号
     * @param callback 发送回调函数指针
     * @param user_data 透传给回调的用户数据
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_set_tx_callback(fdcan_device device, fdcan_tx_callback_t callback, void *user_data)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        fdcan_context &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        context.tx_callback = callback;
        context.tx_user_data = user_data;
        return 0;
    }

    /**
     * @brief 触发总线错误后的恢复流程
     *
     * @param device CAN 通道编号
     * @param timeout 手动恢复等待超时 负数表示无限等待
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_recover(fdcan_device device, fdcan_timeout timeout)
    {
        if (!valid_device(device)) {
            return -EINVAL;
        }
        fdcan_context &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        if (!atomic_cas(&context.recovering, 0, 1)) {
            return -EALREADY;
        }

        #if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
            const k_timeout_t zephyr_timeout = timeout.milliseconds < 0 ? K_FOREVER : K_MSEC(timeout.milliseconds);
            int ret = can_recover(context.device, zephyr_timeout);
            if (ret == 0) {
                atomic_inc(&context.statistics.recovery_succeeded);
                atomic_set(&context.recovery_state, FDCAN_RECOVERY_SUCCEEDED);
                atomic_clear(&context.recovering);
                return 0;
            }
        #else
            ARG_UNUSED(timeout);
            int ret;
        #endif

        atomic_set(&context.recovery_state, FDCAN_RECOVERY_STOPPING);
        ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            goto failed;
        }

        atomic_set(&context.recovery_state, FDCAN_RECOVERY_CLEARING_TX);
        atomic_clear(&context.tx_active);

        atomic_set(&context.recovery_state, FDCAN_RECOVERY_RESTARTING);
        ret = can_start(context.device);
        if (ret != 0 && ret != -EALREADY) {
            goto failed;
        }

        atomic_inc(&context.statistics.recovery_succeeded);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_SUCCEEDED);
        atomic_clear(&context.recovering);
        return 0;

        failed:
        atomic_inc(&context.statistics.recovery_failed);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_FAILED);
        atomic_clear(&context.recovering);
        return ret;
    }

    /**
     * @brief 获取指定 FDCAN 通道的总线状态与错误计数
     *
     * @param device CAN 通道编号
     * @param state 输出总线状态
     * @param error_count 输出错误计数 可为空指针
     * @return 成功返回 0 失败返回负错误码
     */
    int bsp_fdcan_get_state(fdcan_device device, fdcan_bus_state *state, fdcan_error_count *error_count)
    {
        if (!valid_device(device) || state == nullptr) {
            return -EINVAL;
        }
        const fdcan_context &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }

        enum can_state native_state
        {
        };
        struct can_bus_err_cnt native_error_count{};
        const int ret = can_get_state(context.device, &native_state, error_count == nullptr ? nullptr : &native_error_count);
        if (ret != 0) {
            return ret;
        }

        switch (native_state) {
            case CAN_STATE_ERROR_ACTIVE:
            *state = FDCAN_BUS_ERROR_ACTIVE;
            break;
            case CAN_STATE_ERROR_WARNING:
            *state = FDCAN_BUS_ERROR_WARNING;
            break;
            case CAN_STATE_ERROR_PASSIVE:
            *state = FDCAN_BUS_ERROR_PASSIVE;
            break;
            case CAN_STATE_BUS_OFF:
            *state = FDCAN_BUS_OFF;
            break;
            case CAN_STATE_STOPPED:
            default:
            *state = FDCAN_BUS_STOPPED;
            break;
        }

        if (error_count != nullptr) {
            error_count->transmit = native_error_count.tx_err_cnt;
            error_count->receive = native_error_count.rx_err_cnt;
        }
        return 0;
    }

    /**
     * @brief 获取指定 FDCAN 通道当前的恢复状态
     *
     * @param device CAN 通道编号
     * @return 恢复状态枚举值
     */
    fdcan_recovery_state bsp_fdcan_get_recovery_state(fdcan_device device)
    {
        if (!valid_device(device)) {
            return FDCAN_RECOVERY_FAILED;
        }
        return static_cast<fdcan_recovery_state>(
        atomic_get(&contexts[device].recovery_state));
    }

    /**
     * @brief 获取指定 FDCAN 通道的统计数据
     *
     * @param device CAN 通道编号
     * @param statistics 输出统计信息
     */
    void bsp_fdcan_get_statistics(fdcan_device device, fdcan_statistics *statistics)
    {
        fdcan_atomic_statistics &source = contexts[device].statistics;
        statistics->tx_queued = atomic_get(&source.tx_queued);
        statistics->tx_completed = atomic_get(&source.tx_completed);
        statistics->tx_dropped = atomic_get(&source.tx_dropped);
        statistics->tx_errors = atomic_get(&source.tx_errors);
        statistics->rx_received = atomic_get(&source.rx_received);
        statistics->rx_dropped = atomic_get(&source.rx_dropped);
        statistics->recovery_succeeded = atomic_get(&source.recovery_succeeded);
        statistics->recovery_failed = atomic_get(&source.recovery_failed);
    }

    /**
     * @brief 清零指定 FDCAN 通道的统计数据
     *
     * @param device CAN 通道编号
     */
    void bsp_fdcan_clear_statistics(fdcan_device device)
    {
        clear_atomic_statistics(&contexts[device].statistics);
    }

    /**
     * @brief 查询指定 FDCAN 通道是否已就绪
     *
     * @param device CAN 通道编号
     * @return 就绪返回 true 否则返回 false
     */
    bool bsp_fdcan_is_ready(fdcan_device device)
    {
        if (!valid_device(device)) {
            return false;
        }
        const fdcan_context &context = contexts[device];
        return context.initialized && context.device != nullptr && device_is_ready(context.device);
    }

    /**
     * @brief 获取指定 FDCAN 通道的数据段比特率
     *
     * @param device CAN 通道编号
     * @return 数据段比特率 未启用 CAN-FD 或未就绪返回 0
     */
    uint32_t bsp_fdcan_get_data_bitrate(fdcan_device device)
    {
        if (!bsp_fdcan_is_ready(device)) {
            return 0;
        }
        const fdcan_context &context = contexts[device];
        return context.fd_enabled ? context.data_bitrate : 0;
    }

    /**
     * @brief 查询指定 FDCAN 通道是否有待完成的发送
     *
     * @param device CAN 通道编号
     * @return 有待发送报文返回 1 否则返回 0
     */
    size_t bsp_fdcan_tx_pending(fdcan_device device)
    {
        if (!valid_device(device)) {
            return 0;
        }
        const fdcan_context &context = contexts[device];
        return atomic_get(&context.tx_active) ? 1 : 0;
    }
}
