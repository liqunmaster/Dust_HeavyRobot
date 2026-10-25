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

    constexpr size_t canfd_max_data_length = 64;

    typedef struct
    {
        int32_t milliseconds;
    } CanfdTimeout;

    typedef enum
    {
        CANFD_DEVICE_CAN0 = 0,
        CANFD_DEVICE_CAN1 = 1,
        CANFD_DEVICE_CAN2 = 2,
        CANFD_DEVICE_CAN3 = 3,
        CANFD_DEVICE_COUNT = 4,
    } CanfdDevice;

    typedef enum
    {
        CANFD_PROTOCOL_CLASSIC = 0,
        CANFD_PROTOCOL_FD = 1,
    } CanfdProtocol;

    typedef enum
    {
        CANFD_ID_STANDARD = 0,
        CANFD_ID_EXTENDED = 1,
    } CanfdIdType;

    typedef enum
    {
        CANFD_BRS_DISABLED = 0,
        CANFD_BRS_ENABLED = 1,
    } CanfdBrs;

    typedef enum
    {
        CANFD_MODE_NORMAL = 0,
        CANFD_MODE_LOOPBACK = 1,
        CANFD_MODE_LISTEN_ONLY = 2,
    } CanfdMode;

    typedef enum
    {
        CANFD_RETRANSMISSION_ENABLED = 0,
        CANFD_RETRANSMISSION_DISABLED = 1,
    } CanfdRetransmission;

    typedef enum
    {
        CANFD_BUS_ERROR_ACTIVE = 0,
        CANFD_BUS_ERROR_WARNING = 1,
        CANFD_BUS_ERROR_PASSIVE = 2,
        CANFD_BUS_OFF = 3,
        CANFD_BUS_STOPPED = 4,
    } CanfdBusState;

    typedef enum
    {
        CANFD_RECOVERY_IDLE = 0,
        CANFD_RECOVERY_STOPPING = 1,
        CANFD_RECOVERY_CLEARING_TX = 2,
        CANFD_RECOVERY_RECONFIGURING = 3,
        CANFD_RECOVERY_RESTARTING = 4,
        CANFD_RECOVERY_SUCCEEDED = 5,
        CANFD_RECOVERY_FAILED = 6,
    } CanfdRecoveryState;

    typedef struct
    {
        uint32_t id;
        CanfdIdType id_type;
        CanfdProtocol protocol;
        CanfdBrs bitrate_switch;
        uint8_t length;
        uint8_t data[canfd_max_data_length];
    } CanfdFrame;

    typedef struct
    {
        CanfdMode mode;
        CanfdRetransmission retransmission;
    } CanfdConfig;

    typedef struct
    {
        uint8_t transmit;
        uint8_t receive;
    } CanfdErrorCount;

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
    } CanfdStatistics;

    // Cumulative timer cycles spent in BSP RX callbacks, including subscribers.
    typedef struct
    {
        uint32_t cycles;
        uint32_t max_cycles;
    } CanfdRxTiming;

    int bsp_canfd_get_rx_timing(CanfdDevice device, CanfdRxTiming *timing);

    int bsp_canfd_init(CanfdDevice device, const CanfdConfig *config);

    int bsp_canfd_deinit(CanfdDevice device);

    int bsp_canfd_transmit(CanfdDevice device, const CanfdFrame *frame, CanfdTimeout timeout);

    typedef struct
    {
        uint32_t id;
        CanfdIdType id_type;
        CanfdProtocol protocol;
        uint8_t length;
        const uint8_t *data;
    } CanfdRxView;

    typedef void (*CanfdRxCallback)(CanfdDevice device, const CanfdRxView *frame, void *user_data);

    int bsp_canfd_set_rx_callback(CanfdDevice device, CanfdRxCallback callback, void *user_data);

    int bsp_canfd_add_rx_callback(CanfdDevice device, CanfdRxCallback callback, void *user_data);

    typedef void (*CanfdTxCallback)(CanfdDevice device, int error, void *user_data);

    int bsp_canfd_set_tx_callback(CanfdDevice device, CanfdTxCallback callback, void *user_data);

    int bsp_canfd_recover(CanfdDevice device, CanfdTimeout timeout);

    int bsp_canfd_get_state(CanfdDevice device, CanfdBusState *state, CanfdErrorCount *error_count);

    CanfdRecoveryState bsp_canfd_get_recovery_state(CanfdDevice device);

    void bsp_canfd_get_statistics(CanfdDevice device, CanfdStatistics *statistics);

    void bsp_canfd_clear_statistics(CanfdDevice device);

    bool bsp_canfd_is_ready(CanfdDevice device);

    uint32_t bsp_canfd_get_data_bitrate(CanfdDevice device);

    size_t bsp_canfd_tx_pending(CanfdDevice device);

#ifdef __cplusplus
}
#endif

constexpr CanfdTimeout canfd_no_wait{0};

constexpr CanfdTimeout canfd_wait_forever{-1};

constexpr CanfdTimeout canfd_msec(int32_t ms)
{
    return CanfdTimeout{ms};
}

namespace bsp_canfd_detail
{
    struct CanfdAtomicStatistics
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

    struct CanfdContext
    {
        CanfdDevice id;
        const struct device *device;
        CanfdConfig config;
        uint32_t bitrate;
        uint16_t sample_point;
        uint32_t data_bitrate;
        uint16_t data_sample_point;
        bool fd_enabled;
        CanfdAtomicStatistics statistics;
        CanfdRxTiming rx_timing;
        atomic_t tx_active;
        atomic_t recovering;
        atomic_t recovery_state;
        CanfdRxCallback rx_callback;
        void *rx_user_data;
        static constexpr size_t extra_rx_callback_count = 4;
        CanfdRxCallback extra_rx_callbacks[extra_rx_callback_count];
        void *extra_rx_user_data[extra_rx_callback_count];
        CanfdTxCallback tx_callback;
        void *tx_user_data;
        int standard_filter_id;
        int extended_filter_id;
        bool initialized;
    };
}
