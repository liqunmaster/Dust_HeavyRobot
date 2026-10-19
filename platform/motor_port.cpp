#include "motor_port.hpp"

#include <string.h>

namespace
{
    MotorPort motor_port;

    K_MUTEX_DEFINE(init_lock);

}

bool MotorPort::CubeEnableFrame(const CanfdFrame &frame)
{
    if (frame.id_type != CANFD_ID_STANDARD || frame.id != 0x01 ||
        frame.protocol != CANFD_PROTOCOL_CLASSIC || frame.length != 8 || frame.data[7] != 0xFC) {
        return false;
    }
    for (uint8_t index = 0; index < 7; ++index) {
        if (frame.data[index] != 0xFF) {
            return false;
        }
    }
    return true;
}

void MotorPort::MarkCubeRequestSent(CanfdDevice bus, const CanfdFrame &frame)
{
    const uint32_t now_ms = k_uptime_get_32();
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    for (MotorEntry &entry : motors_) {
        if (!entry.used || entry.kind != MotorKind::cubemars || entry.bus != bus) {
            continue;
        }
        if (entry.request_count == entry.response_count) {
            entry.pending_since_ms = now_ms;
        }
        ++entry.request_count;
        if (CubeEnableFrame(frame)) {
            entry.enable_pending = true;
        }
        break;
    }
    k_spin_unlock(&motor_lock_, key);
}

bool MotorPort::ValidBus(CanfdDevice bus)
{
    return bus >= CANFD_DEVICE_CAN0 && bus < CANFD_DEVICE_COUNT;
}

bool MotorPort::ValidFrame(const CanfdFrame &frame)
{
    if (frame.id_type == CANFD_ID_STANDARD && frame.id > 0x7FF) {
        return false;
    }
    if (frame.id_type == CANFD_ID_EXTENDED && frame.id > 0x1FFFFFFF) {
        return false;
    }
    if (frame.id_type != CANFD_ID_STANDARD && frame.id_type != CANFD_ID_EXTENDED) {
        return false;
    }
    if (frame.protocol == CANFD_PROTOCOL_CLASSIC) {
        return frame.length <= 8;
    }
    return frame.protocol == CANFD_PROTOCOL_FD && frame.length <= canfd_max_data_length;
}

bool MotorPort::CubeControlBlocked(const MotorControlTopicData &control)
{
    if (control.frame.id_type != CANFD_ID_STANDARD || control.frame.id != 0x01 || control.frame.protocol != CANFD_PROTOCOL_CLASSIC) {
        return false;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    for (const MotorEntry &entry : motors_) {
        if (entry.used && entry.kind == MotorKind::cubemars && entry.bus == control.bus) {
            const bool blocked = entry.offline || !entry.enable_acknowledged;
            k_spin_unlock(&motor_lock_, key);
            return blocked;
        }
    }
    k_spin_unlock(&motor_lock_, key);
    return false;
}

void MotorPort::rx_callback(CanfdDevice bus, const CanfdRxView *frame, void *context)
{
    motor_port.OnRxCallback(bus, frame, context);
}

void MotorPort::OnRxCallback(CanfdDevice bus, const CanfdRxView *frame, void *)
{
    if (!ValidBus(bus) || frame == nullptr || frame->id_type != CANFD_ID_STANDARD || frame->length < 6 || frame->length > 8) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    for (size_t index = 0; index < motor_count_; ++index) {
        MotorEntry &entry = motors_[index];
        if (!entry.used || entry.bus != bus || entry.feedback_id != frame->id || entry.protocol != frame->protocol ||
            (entry.kind == MotorKind::dm && (frame->data[0] & 0x0F) != entry.motor_id)) {
            continue;
        }
        atomic_inc(&entry.received_count);
        if (entry.kind == MotorKind::cubemars) {
            if (entry.response_count < entry.request_count) {
                ++entry.response_count;
            }
            entry.pending_since_ms = k_uptime_get_32();
            if (entry.enable_pending) {
                entry.enable_acknowledged = true;
                entry.enable_pending = false;
            }
        }
    }
    k_spin_unlock(&motor_lock_, key);
}

int MotorPort::TopicControlSink(const MotorControlTopicData &control)
{
    return motor_port.OnTopicControlSink(control);
}

int MotorPort::OnTopicControlSink(const MotorControlTopicData &control)
{
    return canfd_port_submit(control.bus, control.frame);
}

int MotorPort::BindMotor(MotorKind kind, void *motor, CanfdDevice bus, uint32_t feedback_id,
                         CanfdProtocol protocol, uint8_t motor_id, bool automatic_recovery)
{
    if (motor == nullptr || !ValidBus(bus) || feedback_id > 0x7FF) {
        return -EINVAL;
    }
    const int ret = canfd_port_init();
    if (ret != 0) {
        return ret;
    }
    if (!bus_registered_[bus]) {
        return -ENODEV;
    }
    if (protocol == CANFD_PROTOCOL_FD && bsp_canfd_get_data_bitrate(bus) == 0) {
        return -ENOTSUP;
    }

    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    MotorEntry *free_entry = nullptr;
    for (MotorEntry &entry : motors_) {
        if (!entry.used) {
            if (free_entry == nullptr) {
                free_entry = &entry;
            }
            continue;
        }
        if (entry.motor == motor) {
            k_spin_unlock(&motor_lock_, key);
            return -EALREADY;
        }
        if (entry.bus == bus && entry.feedback_id == feedback_id && entry.protocol == protocol && (entry.kind != MotorKind::dm || kind != MotorKind::dm || entry.motor_id == motor_id)) {
            k_spin_unlock(&motor_lock_, key);
            return -EADDRINUSE;
        }
    }
    if (free_entry == nullptr) {
        k_spin_unlock(&motor_lock_, key);
        return -ENOSPC;
    }
    free_entry->used = true;
    free_entry->kind = kind;
    free_entry->motor = motor;
    free_entry->bus = bus;
    free_entry->feedback_id = feedback_id;
    free_entry->protocol = protocol;
    free_entry->motor_id = motor_id;
    atomic_set(&free_entry->received_count, 0);
    free_entry->request_count = 0;
    free_entry->response_count = 0;
    free_entry->pending_since_ms = 0;
    free_entry->offline = kind == MotorKind::cubemars;
    free_entry->enable_pending = false;
    free_entry->enable_acknowledged = false;
    free_entry->automatic_recovery = automatic_recovery;
    ++motor_count_;
    k_spin_unlock(&motor_lock_, key);
    return 0;
}

uint32_t MotorPort::ReceivedCount(const void *motor)
{
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    for (const MotorEntry &entry : motors_) {
        if (entry.used && entry.motor == motor) {
            const uint32_t count = static_cast<uint32_t>(atomic_get(&entry.received_count));
            k_spin_unlock(&motor_lock_, key);
            return count;
        }
    }
    k_spin_unlock(&motor_lock_, key);
    return 0;
}

int MotorPort::Init()
{
    if (atomic_get(&initialized_) != 0) {
        return 0;
    }
    k_mutex_lock(&init_lock, K_FOREVER);
    if (atomic_get(&initialized_) != 0) {
        k_mutex_unlock(&init_lock);
        return 0;
    }

    motor_topic_init();

    for (uint8_t index = 0; index < CANFD_DEVICE_COUNT; ++index) {
        const CanfdDevice bus = static_cast<CanfdDevice>(index);
        int bus_result = 0;
        if (!bsp_canfd_is_ready(bus)) {
            const CanfdConfig config{CANFD_MODE_NORMAL, CANFD_RETRANSMISSION_DISABLED};
            bus_result = bsp_canfd_init(bus, &config);
            if (bus_result == -EALREADY) {
                bus_result = 0;
            }
        }
        if (bus_result == 0) {
            bus_result = bsp_canfd_set_rx_callback(bus, rx_callback, nullptr);
        }
        if (bus_result != 0) {
            if (bus == CANFD_DEVICE_CAN0) {
                k_mutex_unlock(&init_lock);
                return bus_result;
            }
            continue;
        }
        bus_registered_[index] = true;
    }

    motor_topic_set_control_sink(TopicControlSink);

    atomic_set(&initialized_, 1);
    k_mutex_unlock(&init_lock);
    return 0;
}

int MotorPort::SubscribeRaw(CanfdDevice bus, uint32_t id)
{
    if (!ValidBus(bus) || id >= 0x800) {
        return -EINVAL;
    }
    k_mutex_lock(&init_lock, K_FOREVER);
    for (size_t index = 0; index < raw_subscription_count_; ++index) {
        if (raw_subscriptions_[index].bus == bus && raw_subscriptions_[index].id == id) {
            k_mutex_unlock(&init_lock);
            return -EALREADY;
        }
    }
    if (raw_subscription_count_ == 32) {
        k_mutex_unlock(&init_lock);
        return -ENOSPC;
    }
    PlatformCanfdRxHandle handle{};
    const int ret = platform_canfd_subscribe(bus, id, CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC, handle);
    if (ret == 0) {
        const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
        raw_subscriptions_[raw_subscription_count_++] = {bus, id, handle};
        k_spin_unlock(&motor_lock_, key);
    }
    k_mutex_unlock(&init_lock);
    return ret;
}

int MotorPort::LatestRaw(CanfdDevice bus, uint32_t id, CanfdRawSnapshot &snapshot)
{
    PlatformCanfdRxHandle handle{};
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    for (size_t index = 0; index < raw_subscription_count_; ++index) {
        if (raw_subscriptions_[index].bus == bus && raw_subscriptions_[index].id == id) {
            handle = raw_subscriptions_[index].handle;
            break;
        }
    }
    k_spin_unlock(&motor_lock_, key);
    if (handle.slot == 0xFF) {
        return -ENOENT;
    }
    PlatformCanfdSnapshot raw{};
    const int ret = platform_canfd_latest(handle, raw);
    if (ret != 0) {
        return ret;
    }
    if (raw.length > sizeof(snapshot.data)) {
        return -EMSGSIZE;
    }
    memcpy(snapshot.data, raw.data, raw.length);
    snapshot.length = raw.length;
    snapshot.timestamp_ms = raw.timestamp_ms;
    return 0;
}

int MotorPort::Bind(C610 &motor)
{
    return BindMotor(MotorKind::c610, &motor, motor.GetDevice(), motor.GetFeedbackId(), CANFD_PROTOCOL_CLASSIC, motor.GetMotorId());
}

int MotorPort::Bind(C620 &motor)
{
    return BindMotor(MotorKind::c620, &motor, motor.GetDevice(), motor.GetFeedbackId(), CANFD_PROTOCOL_CLASSIC, motor.GetMotorId());
}

int MotorPort::Bind(DmMotor &motor, bool automatic_recovery)
{
    return BindMotor(MotorKind::dm, &motor, motor.GetDevice(), motor.GetFeedbackId(), motor.GetFeedbackProtocol(), motor.GetMotorId(), automatic_recovery);
}

int MotorPort::Bind(Cubemars &motor)
{
    return BindMotor(MotorKind::cubemars, &motor, motor.GetDevice(), motor.GetFeedbackId(), CANFD_PROTOCOL_CLASSIC, 0);
}

int MotorPort::MotorHealth(size_t index, CanfdMotorHealthData &data)
{
    if (index >= CANFD_PORT_MAX_MOTORS) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    MotorEntry entry = motors_[index];
    k_spin_unlock(&motor_lock_, key);
    if (!entry.used) {
        return -ENOENT;
    }

    const k_spinlock_key_t updated_key = k_spin_lock(&motor_lock_);
    entry = motors_[index];
    k_spin_unlock(&motor_lock_, updated_key);

    data.bus = entry.bus;
    data.feedback_id = entry.feedback_id;
    switch (entry.kind) {
        case MotorKind::c610: {
            const auto &motor = *static_cast<C610 *>(entry.motor);
            data.kind = MotorChannelKind::c610;
            data.valid_feedback_count = motor.GetFeedbackCount();
            data.control_frame_id = motor.GetCommandFrameId();
            data.status = 0;
            break;
        }
        case MotorKind::c620: {
            const auto &motor = *static_cast<C620 *>(entry.motor);
            data.kind = MotorChannelKind::c620;
            data.valid_feedback_count = motor.GetFeedbackCount();
            data.control_frame_id = motor.GetCommandFrameId();
            data.status = 0;
            break;
        }
        case MotorKind::dm: {
            const auto &motor = *static_cast<DmMotor *>(entry.motor);
            data.kind = MotorChannelKind::dm;
            data.valid_feedback_count = motor.GetFeedbackCount();
            data.control_frame_id = motor.GetControlFrameId();
            data.status = motor.GetData().status;
            break;
        }
        case MotorKind::cubemars: {
            const auto &motor = *static_cast<Cubemars *>(entry.motor);
            data.kind = MotorChannelKind::cubemars;
            data.valid_feedback_count = motor.GetFeedbackCount();
            data.control_frame_id = motor.GetControlFrameId();
            data.status = 0;
            break;
        }
    }
    data.request_count = entry.request_count;
    data.response_count = entry.response_count;
    data.pending_since_ms = entry.pending_since_ms;
    data.enable_acknowledged = entry.enable_acknowledged;
    data.automatic_recovery = entry.automatic_recovery;
    return 0;
}

size_t MotorPort::MotorCount()
{
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    const size_t count = motor_count_;
    k_spin_unlock(&motor_lock_, key);
    return count;
}

void MotorPort::SetMotorOffline(size_t index, bool offline)
{
    if (index >= CANFD_PORT_MAX_MOTORS) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    if (!motors_[index].used) {
        k_spin_unlock(&motor_lock_, key);
        return;
    }
    const MotorKind kind = motors_[index].kind;
    const bool was_offline = motors_[index].offline;
    motors_[index].offline = offline;
    if (kind == MotorKind::cubemars && offline && !was_offline) {
        motors_[index].enable_pending = false;
        motors_[index].enable_acknowledged = false;
    }
    k_spin_unlock(&motor_lock_, key);

}

int MotorPort::BuildOfflineFrame(size_t index, CanfdFrame &frame)
{
    return canfd_port_build_recovery_frame(index, false, frame);
}

int MotorPort::BuildRecoveryFrame(size_t index, bool clear_error, CanfdFrame &frame)
{
    if (index >= CANFD_PORT_MAX_MOTORS) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    const MotorEntry entry = motors_[index];
    k_spin_unlock(&motor_lock_, key);
    if (!entry.used) {
        return -ENOENT;
    }
    switch (entry.kind) {
        case MotorKind::c610:
        case MotorKind::c620:
            return -ENOTSUP;
        case MotorKind::dm:
            return clear_error ? static_cast<DmMotor *>(entry.motor)->BuildClearErrorFrame(frame) : static_cast<DmMotor *>(entry.motor)->BuildEnableFrame(frame);
        case MotorKind::cubemars:
            return static_cast<Cubemars *>(entry.motor)->BuildEnableFrame(frame);
    }
    return -EINVAL;
}

int MotorPort::BuildControlFrame(size_t index, CanfdFrame &frame)
{
    if (index >= CANFD_PORT_MAX_MOTORS) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock_);
    const MotorEntry entry = motors_[index];
    k_spin_unlock(&motor_lock_, key);
    if (!entry.used) {
        return -ENOENT;
    }
    switch (entry.kind) {
        case MotorKind::c610:
            return static_cast<C610 *>(entry.motor)->BuildControlFrame(frame);
        case MotorKind::c620:
            return static_cast<C620 *>(entry.motor)->BuildControlFrame(frame);
        case MotorKind::dm:
            return static_cast<DmMotor *>(entry.motor)->BuildControlFrame(frame);
        case MotorKind::cubemars:
            return static_cast<Cubemars *>(entry.motor)->BuildControlFrame(frame);
    }
    return -EINVAL;
}

int MotorPort::Submit(CanfdDevice bus, const CanfdFrame &frame)
{
    if (!ValidBus(bus) || !ValidFrame(frame)) {
        atomic_inc(&control_send_dropped_);
        return -EINVAL;
    }
    const int result = platform_canfd_submit(bus, frame);
    if (result != 0) {
        atomic_inc(&control_send_dropped_);
        return result;
    }
    return 0;
}

int MotorPort::Submit(const C610 &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildControlFrame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped_);
        return ret;
    }
    return canfd_port_submit(motor.GetDevice(), frame);
}

int MotorPort::Submit(const C620 &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildControlFrame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped_);
        return ret;
    }
    return canfd_port_submit(motor.GetDevice(), frame);
}

int MotorPort::Submit(const DmMotor &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildControlFrame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped_);
        return ret;
    }
    return canfd_port_submit(motor.GetDevice(), frame);
}

int MotorPort::Submit(const Cubemars &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildControlFrame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped_);
        return ret;
    }
    return canfd_port_submit(motor.GetDevice(), frame);
}

int MotorPort::SendOnce(CanfdDevice bus, const CanfdFrame &frame)
{
    const int ret = platform_canfd_send_once(bus, frame);
    if (ret == 0 && CubeEnableFrame(frame)) {
        MarkCubeRequestSent(bus, frame);
    }
    return ret;
}

int MotorPort::RequestMode(DmMotor &motor, DmControlMode mode)
{
    CanfdFrame frame{};
    int ret = motor.BuildModeFrame(mode, frame);
    if (ret != 0) {
        return ret;
    }

    const CanfdDevice bus = motor.GetDevice();
    if (!ValidBus(bus)) {
        return -EINVAL;
    }
    ret = motor.MarkModeRequest(mode);
    if (ret != 0) {
        return ret;
    }
    return platform_canfd_send_once(bus, frame);
}

int MotorPort::Enable(const Cubemars &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildEnableFrame(frame);
    return ret == 0 ? canfd_port_send_once(motor.GetDevice(), frame) : ret;
}

int MotorPort::Disable(const Cubemars &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildDisableFrame(frame);
    return ret == 0 ? canfd_port_send_once(motor.GetDevice(), frame) : ret;
}

int MotorPort::SaveZero(const Cubemars &motor)
{
    CanfdFrame frame{};
    const int ret = motor.BuildSaveZeroFrame(frame);
    return ret == 0 ? canfd_port_send_once(motor.GetDevice(), frame) : ret;
}

uint32_t MotorPort::ReceivedCount(const C610 &motor)
{
    return ReceivedCount(&motor);
}

uint32_t MotorPort::ReceivedCount(const C620 &motor)
{
    return ReceivedCount(&motor);
}

uint32_t MotorPort::ReceivedCount(const DmMotor &motor)
{
    return ReceivedCount(&motor);
}

uint32_t MotorPort::ReceivedCount(const Cubemars &motor)
{
    return ReceivedCount(&motor);
}

uint32_t MotorPort::RxDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&rx_dropped_));
}

uint32_t MotorPort::TxErrorCount()
{
    return static_cast<uint32_t>(atomic_get(&tx_errors_));
}

uint32_t MotorPort::ControlDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&control_send_dropped_));
}

int canfd_port_init()
{
    return motor_port.Init();
}

int canfd_port_subscribe_raw(CanfdDevice bus, uint32_t id)
{
    return motor_port.SubscribeRaw(bus, id);
}

int canfd_port_latest_raw(CanfdDevice bus, uint32_t id, CanfdRawSnapshot &snapshot)
{
    return motor_port.LatestRaw(bus, id, snapshot);
}

int canfd_port_bind(C610 &motor)
{
    return motor_port.Bind(motor);
}

int canfd_port_bind(C620 &motor)
{
    return motor_port.Bind(motor);
}

int canfd_port_bind(DmMotor &motor, bool automatic_recovery)
{
    return motor_port.Bind(motor, automatic_recovery);
}

int canfd_port_bind(Cubemars &motor)
{
    return motor_port.Bind(motor);
}

int canfd_port_motor_health(size_t index, CanfdMotorHealthData &data)
{
    return motor_port.MotorHealth(index, data);
}

size_t canfd_port_motor_count()
{
    return motor_port.MotorCount();
}

void canfd_port_set_motor_offline(size_t index, bool offline)
{
    motor_port.SetMotorOffline(index, offline);
}

int canfd_port_build_offline_frame(size_t index, CanfdFrame &frame)
{
    return motor_port.BuildOfflineFrame(index, frame);
}

int canfd_port_build_recovery_frame(size_t index, bool clear_error, CanfdFrame &frame)
{
    return motor_port.BuildRecoveryFrame(index, clear_error, frame);
}

int canfd_port_build_control_frame(size_t index, CanfdFrame &frame)
{
    return motor_port.BuildControlFrame(index, frame);
}

int canfd_port_submit(CanfdDevice bus, const CanfdFrame &frame)
{
    return motor_port.Submit(bus, frame);
}

int canfd_port_submit(const C610 &motor)
{
    return motor_port.Submit(motor);
}

int canfd_port_submit(const C620 &motor)
{
    return motor_port.Submit(motor);
}

int canfd_port_submit(const DmMotor &motor)
{
    return motor_port.Submit(motor);
}

int canfd_port_submit(const Cubemars &motor)
{
    return motor_port.Submit(motor);
}

int canfd_port_send_once(CanfdDevice bus, const CanfdFrame &frame)
{
    return motor_port.SendOnce(bus, frame);
}

int canfd_port_request_mode(DmMotor &motor, DmControlMode mode)
{
    return motor_port.RequestMode(motor, mode);
}

int canfd_port_enable(const Cubemars &motor)
{
    return motor_port.Enable(motor);
}

int canfd_port_disable(const Cubemars &motor)
{
    return motor_port.Disable(motor);
}

int canfd_port_save_zero(const Cubemars &motor)
{
    return motor_port.SaveZero(motor);
}

uint32_t canfd_port_received_count(const C610 &motor)
{
    return motor_port.ReceivedCount(motor);
}

uint32_t canfd_port_received_count(const C620 &motor)
{
    return motor_port.ReceivedCount(motor);
}

uint32_t canfd_port_received_count(const DmMotor &motor)
{
    return motor_port.ReceivedCount(motor);
}

uint32_t canfd_port_received_count(const Cubemars &motor)
{
    return motor_port.ReceivedCount(motor);
}

uint32_t canfd_port_rx_dropped_count()
{
    return motor_port.RxDroppedCount();
}

uint32_t canfd_port_tx_error_count()
{
    return motor_port.TxErrorCount();
}

uint32_t canfd_port_control_dropped_count()
{
    return motor_port.ControlDroppedCount();
}

static_assert((CANFD_PORT_MAX_MOTORS + 32) <= UINT8_MAX);
