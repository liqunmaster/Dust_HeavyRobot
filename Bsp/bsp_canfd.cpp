#include "bsp_canfd.hpp"

namespace
{
    using bsp_canfd_detail::CanfdAtomicStatistics;
    using bsp_canfd_detail::CanfdContext;

    static CanfdContext contexts[CANFD_DEVICE_COUNT];

    static bool ValidDevice(CanfdDevice device)
    {
        return device >= CANFD_DEVICE_CAN0 && device < CANFD_DEVICE_COUNT;
    }

    static const struct device *DeviceFromId(CanfdDevice device)
    {
        switch (device) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
            case CANFD_DEVICE_CAN0:
                return DEVICE_DT_GET(DT_NODELABEL(can0));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
            case CANFD_DEVICE_CAN1:
                return DEVICE_DT_GET(DT_NODELABEL(can1));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
            case CANFD_DEVICE_CAN2:
                return DEVICE_DT_GET(DT_NODELABEL(can2));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
            case CANFD_DEVICE_CAN3:
                return DEVICE_DT_GET(DT_NODELABEL(can3));
#endif
            default:
                return nullptr;
        }
    }

    static can_mode_t ToCanMode(const CanfdContext &context)
    {
        can_mode_t mode = 0;

        if (context.config.mode == CANFD_MODE_LOOPBACK) {
            mode |= CAN_MODE_LOOPBACK;
        } else if (context.config.mode == CANFD_MODE_LISTEN_ONLY) {
            mode |= CAN_MODE_LISTENONLY;
        }
        if (context.config.retransmission == CANFD_RETRANSMISSION_DISABLED) {
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

    static int ValidateConfig(const CanfdConfig *config)
    {
        if (config == nullptr || config->mode > CANFD_MODE_LISTEN_ONLY || config->retransmission > CANFD_RETRANSMISSION_DISABLED) {
            return -EINVAL;
        }
        return 0;
    }

    static int ValidateFrame(const CanfdContext &context, const CanfdFrame *frame)
    {
        if (frame == nullptr || frame->id_type > CANFD_ID_EXTENDED || frame->protocol > CANFD_PROTOCOL_FD || frame->bitrate_switch > CANFD_BRS_ENABLED) {
            return -EINVAL;
        }
        if ((frame->id_type == CANFD_ID_STANDARD && frame->id > CAN_STD_ID_MASK) || (frame->id_type == CANFD_ID_EXTENDED && frame->id > CAN_EXT_ID_MASK)) {
            return -EINVAL;
        }
        if (frame->protocol == CANFD_PROTOCOL_CLASSIC) {
            if (frame->length > 8 || frame->bitrate_switch != CANFD_BRS_DISABLED) {
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

    static void ToCanFrame(const CanfdFrame &source, struct can_frame *destination)
    {
        memset(destination, 0, sizeof(*destination));
        destination->id = source.id;
        destination->dlc = can_bytes_to_dlc(source.length);
        if (source.id_type == CANFD_ID_EXTENDED) {
            destination->flags |= CAN_FRAME_IDE;
        }
        if (source.protocol == CANFD_PROTOCOL_FD) {
            destination->flags |= CAN_FRAME_FDF;
        }
        if (source.bitrate_switch == CANFD_BRS_ENABLED) {
            destination->flags |= CAN_FRAME_BRS;
        }
        memcpy(destination->data, source.data, source.length);
    }

    static void tx_complete_callback(const struct device *device, int error, void *user_data)
    {
        CanfdContext *context = static_cast<CanfdContext *>(user_data);
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

    static void rx_callback(const struct device *device, struct can_frame *frame, void *user_data)
    {
        CanfdContext *context = static_cast<CanfdContext *>(user_data);
        if (context == nullptr || context->device != device || frame == nullptr || !context->initialized || atomic_get(&context->recovering)) {
            return;
        }

        const uint32_t start_cycles = k_cycle_get_32();
        const CanfdRxView received{
            frame->id,
            (frame->flags & CAN_FRAME_IDE) != 0 ? CANFD_ID_EXTENDED : CANFD_ID_STANDARD,
            (frame->flags & CAN_FRAME_FDF) != 0 ? CANFD_PROTOCOL_FD : CANFD_PROTOCOL_CLASSIC,
            static_cast<uint8_t>(can_dlc_to_bytes(frame->dlc)),
            frame->data,
        };

        atomic_inc(&context->statistics.rx_received);
        if (context->rx_callback != nullptr) {
            context->rx_callback(context->id, &received, context->rx_user_data);
        }
        for (size_t index = 0; index < CanfdContext::extra_rx_callback_count; ++index) {
            if (context->extra_rx_callbacks[index] != nullptr) {
                context->extra_rx_callbacks[index](context->id, &received, context->extra_rx_user_data[index]);
            }
        }
        const uint32_t elapsed = k_cycle_get_32() - start_cycles;
        context->rx_timing.cycles += elapsed;
        if (elapsed > context->rx_timing.max_cycles) {
            context->rx_timing.max_cycles = elapsed;
        }
    }

    static int AddDefaultFilters(CanfdContext *context)
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

    static void RemoveDefaultFilters(CanfdContext *context)
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

    static int ReadDtConfig(CanfdDevice device, CanfdContext *context)
    {
        uint32_t bitrate = 0;
        uint16_t sample_point = 0;
        uint32_t data_bitrate = 0;
        uint16_t data_sample_point = 0;
        bool fd_enabled = false;

        switch (device) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
            case CANFD_DEVICE_CAN0:
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
            case CANFD_DEVICE_CAN1:
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
            case CANFD_DEVICE_CAN2:
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
            case CANFD_DEVICE_CAN3:
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

    static int ApplyConfiguration(CanfdContext *context)
    {
        int ret = can_set_mode(context->device, ToCanMode(*context));
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

    static void ClearAtomicStatistics(CanfdAtomicStatistics *statistics)
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

    int bsp_canfd_init(CanfdDevice device, const CanfdConfig *config)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        int ret = ValidateConfig(config);
        if (ret != 0) {
            return ret;
        }

        CanfdContext &context = contexts[device];
        if (context.initialized) {
            return -EALREADY;
        }

        const struct device *canfd_device = DeviceFromId(device);
        if (canfd_device == nullptr || !device_is_ready(canfd_device)) {
            return -ENODEV;
        }

        ret = ReadDtConfig(device, &context);
        if (ret != 0) {
            return ret;
        }

        context.id = device;
        context.device = canfd_device;
        context.config = *config;
        context.standard_filter_id = -1;
        context.extended_filter_id = -1;
        context.rx_callback = nullptr;
        context.rx_user_data = nullptr;
        memset(context.extra_rx_callbacks, 0, sizeof(context.extra_rx_callbacks));
        memset(context.extra_rx_user_data, 0, sizeof(context.extra_rx_user_data));
        context.tx_callback = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, CANFD_RECOVERY_IDLE);
        ClearAtomicStatistics(&context.statistics);

        ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            context.device = nullptr;
            return ret;
        }
        ret = ApplyConfiguration(&context);
        if (ret != 0) {
            context.device = nullptr;
            return ret;
        }
        ret = AddDefaultFilters(&context);
        if (ret != 0) {
            context.device = nullptr;
            return ret;
        }
        ret = can_start(context.device);
        if (ret != 0 && ret != -EALREADY) {
            RemoveDefaultFilters(&context);
            context.device = nullptr;
            return ret;
        }

        context.initialized = true;
        return 0;
    }

    int bsp_canfd_deinit(CanfdDevice device)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
        if (!context.initialized) {
            return -EALREADY;
        }

        atomic_set(&context.recovering, 1);
        RemoveDefaultFilters(&context);
        int ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            atomic_clear(&context.recovering);
            return ret;
        }
        context.rx_callback = nullptr;
        context.rx_user_data = nullptr;
        memset(context.extra_rx_callbacks, 0, sizeof(context.extra_rx_callbacks));
        memset(context.extra_rx_user_data, 0, sizeof(context.extra_rx_user_data));
        context.tx_callback = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        context.initialized = false;
        context.id = CANFD_DEVICE_COUNT;
        context.device = nullptr;
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, CANFD_RECOVERY_IDLE);
        return 0;
    }

    int bsp_canfd_transmit(CanfdDevice device, const CanfdFrame *frame, CanfdTimeout timeout)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        if (atomic_get(&context.recovering)) {
            atomic_inc(&context.statistics.tx_dropped);
            return -EBUSY;
        }

        const int ret = ValidateFrame(context, frame);
        if (ret != 0) {
            return ret;
        }

        if (!atomic_cas(&context.tx_active, 0, 1)) {
            atomic_inc(&context.statistics.tx_dropped);
            return -EAGAIN;
        }

        struct can_frame can_frame{};
        ToCanFrame(*frame, &can_frame);
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

    int bsp_canfd_set_rx_callback(CanfdDevice device, CanfdRxCallback callback, void *user_data)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        context.rx_callback = callback;
        context.rx_user_data = user_data;
        return 0;
    }

    int bsp_canfd_add_rx_callback(CanfdDevice device, CanfdRxCallback callback, void *user_data)
    {
        if (!ValidDevice(device) || callback == nullptr) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        for (size_t index = 0; index < CanfdContext::extra_rx_callback_count; ++index) {
            if (context.extra_rx_callbacks[index] == callback && context.extra_rx_user_data[index] == user_data) {
                return 0;
            }
            if (context.extra_rx_callbacks[index] == nullptr) {
                context.extra_rx_callbacks[index] = callback;
                context.extra_rx_user_data[index] = user_data;
                return 0;
            }
        }
        return -ENOSPC;
    }

    int bsp_canfd_set_tx_callback(CanfdDevice device, CanfdTxCallback callback, void *user_data)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
        if (!context.initialized || context.device == nullptr) {
            return -ENODEV;
        }
        context.tx_callback = callback;
        context.tx_user_data = user_data;
        return 0;
    }

    int bsp_canfd_recover(CanfdDevice device, CanfdTimeout timeout)
    {
        if (!ValidDevice(device)) {
            return -EINVAL;
        }
        CanfdContext &context = contexts[device];
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
            atomic_set(&context.recovery_state, CANFD_RECOVERY_SUCCEEDED);
            atomic_clear(&context.recovering);
            return 0;
        }
#else
        ARG_UNUSED(timeout);
        int ret;
#endif

        atomic_set(&context.recovery_state, CANFD_RECOVERY_STOPPING);
        ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            goto failed;
        }

        atomic_set(&context.recovery_state, CANFD_RECOVERY_CLEARING_TX);
        atomic_clear(&context.tx_active);

        atomic_set(&context.recovery_state, CANFD_RECOVERY_RESTARTING);
        ret = can_start(context.device);
        if (ret != 0 && ret != -EALREADY) {
            goto failed;
        }

        atomic_inc(&context.statistics.recovery_succeeded);
        atomic_set(&context.recovery_state, CANFD_RECOVERY_SUCCEEDED);
        atomic_clear(&context.recovering);
        return 0;

    failed:
        atomic_inc(&context.statistics.recovery_failed);
        atomic_set(&context.recovery_state, CANFD_RECOVERY_FAILED);
        atomic_clear(&context.recovering);
        return ret;
    }

    int bsp_canfd_get_state(CanfdDevice device, CanfdBusState *state, CanfdErrorCount *error_count)
    {
        if (!ValidDevice(device) || state == nullptr) {
            return -EINVAL;
        }
        const CanfdContext &context = contexts[device];
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
                *state = CANFD_BUS_ERROR_ACTIVE;
                break;
            case CAN_STATE_ERROR_WARNING:
                *state = CANFD_BUS_ERROR_WARNING;
                break;
            case CAN_STATE_ERROR_PASSIVE:
                *state = CANFD_BUS_ERROR_PASSIVE;
                break;
            case CAN_STATE_BUS_OFF:
                *state = CANFD_BUS_OFF;
                break;
            case CAN_STATE_STOPPED:
            default:
                *state = CANFD_BUS_STOPPED;
                break;
        }

        if (error_count != nullptr) {
            error_count->transmit = native_error_count.tx_err_cnt;
            error_count->receive = native_error_count.rx_err_cnt;
        }
        return 0;
    }

    CanfdRecoveryState bsp_canfd_get_recovery_state(CanfdDevice device)
    {
        if (!ValidDevice(device)) {
            return CANFD_RECOVERY_FAILED;
        }
        return static_cast<CanfdRecoveryState>(
            atomic_get(&contexts[device].recovery_state));
    }

    void bsp_canfd_get_statistics(CanfdDevice device, CanfdStatistics *statistics)
    {
        CanfdAtomicStatistics &source = contexts[device].statistics;
        statistics->tx_queued = atomic_get(&source.tx_queued);
        statistics->tx_completed = atomic_get(&source.tx_completed);
        statistics->tx_dropped = atomic_get(&source.tx_dropped);
        statistics->tx_errors = atomic_get(&source.tx_errors);
        statistics->rx_received = atomic_get(&source.rx_received);
        statistics->rx_dropped = atomic_get(&source.rx_dropped);
        statistics->recovery_succeeded = atomic_get(&source.recovery_succeeded);
        statistics->recovery_failed = atomic_get(&source.recovery_failed);
    }

    int bsp_canfd_get_rx_timing(CanfdDevice device, CanfdRxTiming *timing)
    {
        if (!ValidDevice(device) || timing == nullptr) {
            return -EINVAL;
        }
        // RX callbacks run in interrupt context on this single-core target.
        const unsigned int key = irq_lock();
        *timing = contexts[device].rx_timing;
        irq_unlock(key);
        return 0;
    }

    void bsp_canfd_clear_statistics(CanfdDevice device)
    {
        ClearAtomicStatistics(&contexts[device].statistics);
    }

    bool bsp_canfd_is_ready(CanfdDevice device)
    {
        if (!ValidDevice(device)) {
            return false;
        }
        const CanfdContext &context = contexts[device];
        return context.initialized && context.device != nullptr && device_is_ready(context.device);
    }

    uint32_t bsp_canfd_get_data_bitrate(CanfdDevice device)
    {
        if (!bsp_canfd_is_ready(device)) {
            return 0;
        }
        const CanfdContext &context = contexts[device];
        return context.fd_enabled ? context.data_bitrate : 0;
    }

    size_t bsp_canfd_tx_pending(CanfdDevice device)
    {
        if (!ValidDevice(device)) {
            return 0;
        }
        const CanfdContext &context = contexts[device];
        return atomic_get(&context.tx_active) ? 1 : 0;
    }
}
