#pragma once

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "bsp_canfd.hpp"
#include "cubemars.hpp"
#include "dji_c610.hpp"
#include "dji_c620.hpp"
#include "dm_motor.hpp"
#include "motor_channel.hpp"
#include "canfd_bus.hpp"

class C610;
class C620;

constexpr size_t CANFD_PORT_MAX_MOTORS = 32;

struct CanfdRawSnapshot
{
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp_ms;
};

struct CanfdMotorHealthData
{
    MotorChannelKind kind;
    CanfdDevice bus;
    uint32_t feedback_id;
    uint32_t valid_feedback_count;
    uint32_t control_frame_id;
    uint8_t status;
    uint32_t request_count;
    uint32_t response_count;
    uint32_t pending_since_ms;
    bool enable_acknowledged;
    bool automatic_recovery;
};

class MotorPort
{
    public:
    int Init();

    int SubscribeRaw(CanfdDevice bus, uint32_t id);

    int LatestRaw(CanfdDevice bus, uint32_t id, CanfdRawSnapshot &snapshot);

    int Bind(C610 &motor);

    int Bind(C620 &motor);

    int Bind(DmMotor &motor, bool automatic_recovery);

    int Bind(Cubemars &motor);

    int MotorHealth(size_t index, CanfdMotorHealthData &data);
    size_t MotorCount();

    void SetMotorOffline(size_t index, bool offline);

    int BuildOfflineFrame(size_t index, CanfdFrame &frame);

    int BuildRecoveryFrame(size_t index, bool clear_error, CanfdFrame &frame);

    int BuildControlFrame(size_t index, CanfdFrame &frame);

    int Submit(CanfdDevice bus, const CanfdFrame &frame);

    int Submit(const C610 &motor);

    int Submit(const C620 &motor);

    int Submit(const DmMotor &motor);

    int Submit(const Cubemars &motor);

    int SendOnce(CanfdDevice bus, const CanfdFrame &frame);

    int RequestMode(DmMotor &motor, DmControlMode mode);

    int Enable(const Cubemars &motor);

    int Disable(const Cubemars &motor);

    int SaveZero(const Cubemars &motor);

    uint32_t ReceivedCount(const C610 &motor);

    uint32_t ReceivedCount(const C620 &motor);

    uint32_t ReceivedCount(const DmMotor &motor);

    uint32_t ReceivedCount(const Cubemars &motor);

    uint32_t RxDroppedCount();

    uint32_t TxErrorCount();

    uint32_t ControlDroppedCount();

    private:
    enum class MotorKind : uint8_t
    {
        c610,
        c620,
        dm,
        cubemars
    };

    struct RawSubscription
    {
        CanfdDevice bus;
        uint32_t id;
        PlatformCanfdRxHandle handle;
    };

    struct MotorEntry
    {
        bool used;
        MotorKind kind;
        void *motor;
        CanfdDevice bus;
        uint32_t feedback_id;
        CanfdProtocol protocol;
        uint8_t motor_id;
        atomic_t received_count;
        uint32_t request_count;
        uint32_t response_count;
        uint32_t pending_since_ms;
        bool offline;
        bool enable_pending;
        bool enable_acknowledged;
        bool automatic_recovery;
    };

    bool CubeEnableFrame(const CanfdFrame &frame);

    void MarkCubeRequestSent(CanfdDevice bus, const CanfdFrame &frame);

    bool ValidBus(CanfdDevice bus);

    bool ValidFrame(const CanfdFrame &frame);

    bool CubeControlBlocked(const MotorControlTopicData &control);

    static void rx_callback(CanfdDevice bus, const CanfdRxView *frame, void *arg3);

    void OnRxCallback(CanfdDevice bus, const CanfdRxView *frame, void *arg3);

    static int TopicControlSink(const MotorControlTopicData &control);

    int OnTopicControlSink(const MotorControlTopicData &control);

    int BindMotor(MotorKind kind, void *motor, CanfdDevice bus, uint32_t feedback_id,
                  CanfdProtocol protocol, uint8_t motor_id, bool automatic_recovery = true);

    uint32_t ReceivedCount(const void *motor);
    struct k_spinlock motor_lock_{};
    MotorEntry motors_[CANFD_PORT_MAX_MOTORS]{};
    RawSubscription raw_subscriptions_[32]{};
    size_t raw_subscription_count_ = 0;
    size_t motor_count_ = 0;
    atomic_t rx_dropped_{};
    atomic_t tx_errors_{};
    atomic_t control_send_dropped_{};
    atomic_t initialized_{};
    bool bus_registered_[CANFD_DEVICE_COUNT]{};
};

int canfd_port_subscribe_raw(CanfdDevice bus, uint32_t id);

int canfd_port_latest_raw(CanfdDevice bus, uint32_t id, CanfdRawSnapshot &snapshot);

int canfd_port_init();

int canfd_port_bind(C610 &motor);

int canfd_port_bind(C620 &motor);

int canfd_port_bind(DmMotor &motor, bool automatic_recovery = true);

int canfd_port_bind(Cubemars &motor);

int canfd_port_motor_health(size_t index, CanfdMotorHealthData &data);

size_t canfd_port_motor_count();

void canfd_port_set_motor_offline(size_t index, bool offline);

int canfd_port_build_offline_frame(size_t index, CanfdFrame &frame);

int canfd_port_build_recovery_frame(size_t index, bool clear_error, CanfdFrame &frame);

int canfd_port_build_control_frame(size_t index, CanfdFrame &frame);

int canfd_port_submit(CanfdDevice device, const CanfdFrame &frame);

int canfd_port_submit(const C610 &motor);

int canfd_port_submit(const C620 &motor);

int canfd_port_submit(const DmMotor &motor);

int canfd_port_submit(const Cubemars &motor);

int canfd_port_send_once(CanfdDevice device, const CanfdFrame &frame);

int canfd_port_request_mode(DmMotor &motor, DmControlMode mode);

int canfd_port_enable(const Cubemars &motor);

int canfd_port_disable(const Cubemars &motor);

int canfd_port_save_zero(const Cubemars &motor);

uint32_t canfd_port_received_count(const C610 &motor);

uint32_t canfd_port_received_count(const C620 &motor);

uint32_t canfd_port_received_count(const DmMotor &motor);

uint32_t canfd_port_received_count(const Cubemars &motor);

uint32_t canfd_port_rx_dropped_count();

uint32_t canfd_port_tx_error_count();

uint32_t canfd_port_control_dropped_count();
