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

    static bool valid_device(fdcan_device device)
    {
        return device >= FDCAN_DEVICE_CAN0 && device < FDCAN_DEVICE_COUNT;
    }

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

    static MCAN_Type *mcan_from_id(fdcan_device device)
    {
        switch (device) {
            case FDCAN_DEVICE_CAN0:
                return reinterpret_cast<MCAN_Type *>(DT_REG_ADDR(DT_NODELABEL(can0)));
            case FDCAN_DEVICE_CAN1:
                return reinterpret_cast<MCAN_Type *>(DT_REG_ADDR(DT_NODELABEL(can1)));
            case FDCAN_DEVICE_CAN2:
                return reinterpret_cast<MCAN_Type *>(DT_REG_ADDR(DT_NODELABEL(can2)));
            case FDCAN_DEVICE_CAN3:
                return reinterpret_cast<MCAN_Type *>(DT_REG_ADDR(DT_NODELABEL(can3)));
            default:
                return nullptr;
        }
    }

    static can_mode_t to_can_mode(const fdcan_context &context)
    {
        can_mode_t mode = 0U;

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

    static int validate_config(const fdcan_config *config)
    {
        if (config == nullptr || config->mode > FDCAN_MODE_LISTEN_ONLY ||
            config->retransmission > FDCAN_RETRANSMISSION_DISABLED) {
            return -EINVAL;
        }
        return 0;
    }

    static int validate_frame(const fdcan_context &context, const fdcan_frame *frame)
    {
        if (frame == nullptr || frame->id_type > FDCAN_ID_EXTENDED || frame->protocol > FDCAN_PROTOCOL_FD || frame->bitrate_switch > FDCAN_BRS_ENABLED) {
            return -EINVAL;
        }
        if ((frame->id_type == FDCAN_ID_STANDARD && frame->id > CAN_STD_ID_MASK) || (frame->id_type == FDCAN_ID_EXTENDED && frame->id > CAN_EXT_ID_MASK)) {
            return -EINVAL;
        }
        if (frame->protocol == FDCAN_PROTOCOL_CLASSIC) {
            if (frame->length > 8U || frame->bitrate_switch != FDCAN_BRS_DISABLED) {
                return -EMSGSIZE;
            }
        } else {
            const bool valid_fd_length = frame->length <= 8U || frame->length == 12U || frame->length == 16U || frame->length == 20U || frame->length == 24U || frame->length == 32U || frame->length == 48U || frame->length == 64U;
            if (!context.fd_enabled || !valid_fd_length) {
                return -EMSGSIZE;
            }
        }
        return 0;
    }

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

    static void tx_complete_callback(const struct device *device, int error, void *user_data)
    {
        fdcan_context *context = static_cast<fdcan_context *>(user_data);
        if (context == nullptr || context->device != device) {
            return;
        }

        if (error == 0) {
            atomic_inc(&context->statistics.tx_completed);
        } else {
            atomic_inc(&context->statistics.tx_errors);
        }
        atomic_clear(&context->tx_active);
        if (context->tx_callback != nullptr) {
            context->tx_callback(context->id, error, context->tx_user_data);
        }
    }

    static void rx_callback(const struct device *device, struct can_frame *frame, void *user_data)
    {
        fdcan_context *context = static_cast<fdcan_context *>(user_data);
        if (context == nullptr || context->device != device || frame == nullptr || !context->initialized || atomic_get(&context->recovering)) {
            return;
        }

        fdcan_frame received{};
        received.id = frame->id;
        received.id_type = (frame->flags & CAN_FRAME_IDE) != 0U ? FDCAN_ID_EXTENDED : FDCAN_ID_STANDARD;
        received.protocol = (frame->flags & CAN_FRAME_FDF) != 0U ? FDCAN_PROTOCOL_FD : FDCAN_PROTOCOL_CLASSIC;
        received.bitrate_switch = (frame->flags & CAN_FRAME_BRS) != 0U ? FDCAN_BRS_ENABLED : FDCAN_BRS_DISABLED;
        received.length = can_dlc_to_bytes(frame->dlc);
        memcpy(received.data, frame->data, received.length);

        atomic_inc(&context->statistics.rx_received);
        if (context->rx_callback != nullptr) {
            context->rx_callback(context->id, &received, context->rx_user_data);
        }
    }

    static int add_default_filters(fdcan_context *context)
    {
        const struct can_filter standard_filter = {
            .id = 0U,
            .mask = 0U,
            .flags = 0U,
        };
        context->standard_filter_id = can_add_rx_filter(context->device, rx_callback, context, &standard_filter);
        if (context->standard_filter_id < 0) {
            return context->standard_filter_id;
        }

        context->extended_filter_id = -1;
        return 0;
    }

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

    static int read_dt_config(fdcan_device device, fdcan_context *context)
    {
        uint32_t bitrate = 0U;
        uint16_t sample_point = 0U;
        uint32_t data_bitrate = 0U;
        uint16_t data_sample_point = 0U;
        bool fd_enabled = false;

        switch (device) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
            case FDCAN_DEVICE_CAN0:
                bitrate = DT_PROP_OR(DT_NODELABEL(can0), bitrate, 0U);
                sample_point = DT_PROP_OR(DT_NODELABEL(can0), sample_point, 0U);
#if defined(CONFIG_CAN_FD_MODE)
                fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can0), bitrate_data);
                data_bitrate = DT_PROP_OR(DT_NODELABEL(can0), bitrate_data, 0U);
                data_sample_point = DT_PROP_OR(DT_NODELABEL(can0), sample_point_data, 0U);
#endif
                break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
            case FDCAN_DEVICE_CAN1:
                bitrate = DT_PROP_OR(DT_NODELABEL(can1), bitrate, 0U);
                sample_point = DT_PROP_OR(DT_NODELABEL(can1), sample_point, 0U);
#if defined(CONFIG_CAN_FD_MODE)
                fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can1), bitrate_data);
                data_bitrate = DT_PROP_OR(DT_NODELABEL(can1), bitrate_data, 0U);
                data_sample_point = DT_PROP_OR(DT_NODELABEL(can1), sample_point_data, 0U);
#endif
                break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
            case FDCAN_DEVICE_CAN2:
                bitrate = DT_PROP_OR(DT_NODELABEL(can2), bitrate, 0U);
                sample_point = DT_PROP_OR(DT_NODELABEL(can2), sample_point, 0U);
#if defined(CONFIG_CAN_FD_MODE)
                fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can2), bitrate_data);
                data_bitrate = DT_PROP_OR(DT_NODELABEL(can2), bitrate_data, 0U);
                data_sample_point = DT_PROP_OR(DT_NODELABEL(can2), sample_point_data, 0U);
#endif
                break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
            case FDCAN_DEVICE_CAN3:
                bitrate = DT_PROP_OR(DT_NODELABEL(can3), bitrate, 0U);
                sample_point = DT_PROP_OR(DT_NODELABEL(can3), sample_point, 0U);
#if defined(CONFIG_CAN_FD_MODE)
                fd_enabled = DT_NODE_HAS_PROP(DT_NODELABEL(can3), bitrate_data);
                data_bitrate = DT_PROP_OR(DT_NODELABEL(can3), bitrate_data, 0U);
                data_sample_point = DT_PROP_OR(DT_NODELABEL(can3), sample_point_data, 0U);
#endif
                break;
#endif
            default:
                return -EINVAL;
        }

        if (bitrate == 0U) {
            return -EINVAL;
        }
#if defined(CONFIG_CAN_FD_MODE)
        if (fd_enabled && data_bitrate == 0U) {
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

    static int apply_configuration(fdcan_context *context)
    {
        int ret = can_set_mode(context->device, to_can_mode(*context));
        if (ret != 0) {
            return ret;
        }

        if (context->sample_point == 0U) {
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
            if (context->data_sample_point == 0U) {
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
        context.rx_callback = nullptr;
        context.rx_user_data = nullptr;
        context.tx_callback = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_IDLE);
        clear_atomic_statistics(&context.statistics);

        ret = can_stop(context.device);
        if (ret != 0 && ret != -EALREADY) {
            context.device = nullptr;
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
        context.rx_callback = nullptr;
        context.rx_user_data = nullptr;
        context.tx_callback = nullptr;
        context.tx_user_data = nullptr;
        atomic_clear(&context.tx_active);
        context.initialized = false;
        context.id = FDCAN_DEVICE_COUNT;
        context.device = nullptr;
        atomic_clear(&context.recovering);
        atomic_set(&context.recovery_state, FDCAN_RECOVERY_IDLE);
        return 0;
    }

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

    int bsp_fdcan_set_rx_interrupt(fdcan_device device, bool enabled)
    {
        if (!valid_device(device) || !contexts[device].initialized) {
            return -ENODEV;
        }
        MCAN_Type *can = mcan_from_id(device);
        if (can == nullptr) {
            return -ENODEV;
        }

        const unsigned int key = irq_lock();
        if (enabled) {
            mcan_enable_interrupts(can, MCAN_INT_RXFIFO0_NEW_MSG);
        } else {
            mcan_disable_interrupts(can, MCAN_INT_RXFIFO0_NEW_MSG);
        }
        irq_unlock(key);
        return 0;
    }

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

        atomic_set(&context.recovery_state, FDCAN_RECOVERY_RECONFIGURING);
        ret = apply_configuration(&context);
        if (ret != 0) {
            goto failed;
        }

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

    fdcan_recovery_state bsp_fdcan_get_recovery_state(fdcan_device device)
    {
        if (!valid_device(device)) {
            return FDCAN_RECOVERY_FAILED;
        }
        return static_cast<fdcan_recovery_state>(
            atomic_get(&contexts[device].recovery_state));
    }

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

    void bsp_fdcan_clear_statistics(fdcan_device device)
    {
        clear_atomic_statistics(&contexts[device].statistics);
    }

    bool bsp_fdcan_is_ready(fdcan_device device)
    {
        if (!valid_device(device)) {
            return false;
        }
        const fdcan_context &context = contexts[device];
        return context.initialized && context.device != nullptr && device_is_ready(context.device);
    }

    uint32_t bsp_fdcan_get_data_bitrate(fdcan_device device)
    {
        if (!bsp_fdcan_is_ready(device)) {
            return 0U;
        }
        const fdcan_context &context = contexts[device];
        return context.fd_enabled ? context.data_bitrate : 0U;
    }

    size_t bsp_fdcan_tx_pending(fdcan_device device)
    {
        if (!valid_device(device)) {
            return 0U;
        }
        const fdcan_context &context = contexts[device];
        return atomic_get(&context.tx_active) ? 1U : 0U;
    }
}
