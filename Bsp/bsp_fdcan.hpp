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

#include <hpm_mcan_drv.h>

#ifdef __cplusplus
#undef __R
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define FDCAN_MAX_DATA_LENGTH 64U

    typedef struct
    {
        int32_t milliseconds;
    } fdcan_timeout;

#define FDCAN_NO_WAIT ((fdcan_timeout){0})
#define FDCAN_WAIT_FOREVER ((fdcan_timeout){-1})
#define FDCAN_MSEC(ms) ((fdcan_timeout){(int32_t)(ms)})

    typedef enum
    {
        FDCAN_DEVICE_CAN0 = 0,
        FDCAN_DEVICE_CAN1 = 1,
        FDCAN_DEVICE_CAN2 = 2,
        FDCAN_DEVICE_CAN3 = 3,
        FDCAN_DEVICE_COUNT = 4,
    } fdcan_device;

    typedef enum
    {
        FDCAN_PROTOCOL_CLASSIC = 0,
        FDCAN_PROTOCOL_FD = 1,
    } fdcan_protocol;

    typedef enum
    {
        FDCAN_ID_STANDARD = 0,
        FDCAN_ID_EXTENDED = 1,
    } fdcan_id_type;

    typedef enum
    {
        FDCAN_BRS_DISABLED = 0,
        FDCAN_BRS_ENABLED = 1,
    } fdcan_brs;

    typedef enum
    {
        FDCAN_MODE_NORMAL = 0,
        FDCAN_MODE_LOOPBACK = 1,
        FDCAN_MODE_LISTEN_ONLY = 2,
    } fdcan_mode;

    typedef enum
    {
        FDCAN_RETRANSMISSION_ENABLED = 0,
        FDCAN_RETRANSMISSION_DISABLED = 1,
    } fdcan_retransmission;

    typedef enum
    {
        FDCAN_BUS_ERROR_ACTIVE = 0,
        FDCAN_BUS_ERROR_WARNING = 1,
        FDCAN_BUS_ERROR_PASSIVE = 2,
        FDCAN_BUS_OFF = 3,
        FDCAN_BUS_STOPPED = 4,
    } fdcan_bus_state;

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

    typedef struct
    {
        uint32_t id;
        fdcan_id_type id_type;
        fdcan_protocol protocol;
        fdcan_brs bitrate_switch;
        uint8_t length;
        uint8_t data[FDCAN_MAX_DATA_LENGTH];
    } fdcan_frame;

    typedef struct
    {
        fdcan_mode mode;
        fdcan_retransmission retransmission;
    } fdcan_config;

    typedef struct
    {
        uint8_t transmit;
        uint8_t receive;
    } fdcan_error_count;

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

    typedef void (*fdcan_rx_callback_t)(fdcan_device device, const fdcan_frame *frame, void *user_data);

    int bsp_fdcan_set_rx_callback(fdcan_device device, fdcan_rx_callback_t callback, void *user_data);

    int bsp_fdcan_set_rx_interrupt(fdcan_device device, bool enabled);

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
