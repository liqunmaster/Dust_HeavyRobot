#include "canfd_bus.hpp"

#include <string.h>

namespace
{
    CanfdBus canfd_bus;

    K_THREAD_STACK_ARRAY_DEFINE(tx_stacks, CANFD_DEVICE_COUNT, 1536);
}

bool CanfdBus::ValidBus(CanfdDevice bus)
{
    return bus >= CANFD_DEVICE_CAN0 && bus < CANFD_DEVICE_COUNT;
}

bool CanfdBus::ValidFrame(const CanfdFrame &frame)
{
    if (frame.id_type == CANFD_ID_STANDARD && frame.id > CAN_STD_ID_MASK) {
        return false;
    }
    if (frame.id_type == CANFD_ID_EXTENDED && frame.id > CAN_EXT_ID_MASK) {
        return false;
    }
    if (frame.id_type != CANFD_ID_STANDARD && frame.id_type != CANFD_ID_EXTENDED) {
        return false;
    }
    if (frame.protocol == CANFD_PROTOCOL_CLASSIC) {
        return frame.length <= 8 && frame.bitrate_switch == CANFD_BRS_DISABLED;
    }
    return frame.protocol == CANFD_PROTOCOL_FD && frame.length <= canfd_max_data_length;
}

void CanfdBus::rx_callback(CanfdDevice bus, const CanfdRxView *frame, void *arg3)
{
    canfd_bus.OnRxCallback(bus, frame, arg3);
}

void CanfdBus::OnRxCallback(CanfdDevice bus, const CanfdRxView *frame, void *)
{
    if (!ValidBus(bus) || frame == nullptr || frame->length > canfd_max_data_length) {
        return;
    }

    const uint32_t timestamp = k_uptime_get_32();
    bool matched = false;
    const k_spinlock_key_t key = k_spin_lock(&subscription_lock_);
    for (Subscription &entry : subscriptions_) {
        if (!entry.used || entry.bus != bus || entry.id != frame->id ||
            entry.id_type != frame->id_type || entry.protocol != frame->protocol ||
            (entry.payload_type >= 0 && (frame->length == 0 || frame->data[0] != entry.payload_type))) {
            continue;
        }
        matched = true;
        memcpy(entry.snapshot.data, frame->data, frame->length);
        if (frame->length < sizeof(entry.snapshot.data)) {
            memset(entry.snapshot.data + frame->length, 0,
                   sizeof(entry.snapshot.data) - frame->length);
        }
        entry.snapshot.length = frame->length;
        entry.snapshot.timestamp_ms = timestamp;
        ++entry.snapshot.sequence;
    }
    k_spin_unlock(&subscription_lock_, key);
    if (!matched) {
        atomic_inc(&rx_dropped_);
    }
}

bool CanfdBus::TakeNextFrame(CanfdDevice bus, CanfdFrame &frame)
{
    TxMailbox &mailbox = mailboxes_[bus];
    const k_spinlock_key_t key = k_spin_lock(&tx_lock_);
    if (mailbox.admin_count != 0) {
        frame = mailbox.admin[mailbox.admin_head];
        mailbox.admin_head = (mailbox.admin_head + 1) % 8;
        --mailbox.admin_count;
        k_spin_unlock(&tx_lock_, key);
        return true;
    }
    for (size_t offset = 0; offset < 32; ++offset) {
        const size_t index = (mailbox.next_periodic + offset) % 32;
        auto &entry = periodic_[index];
        if (entry.pending && entry.bus == bus) {
            frame = entry.frame;
            entry.pending = false;
            mailbox.next_periodic = (index + 1) % 32;
            k_spin_unlock(&tx_lock_, key);
            return true;
        }
    }
    k_spin_unlock(&tx_lock_, key);
    return false;
}

void CanfdBus::TxWorker(void *p1, void *arg2, void *arg3)
{
    canfd_bus.OnTxWorker(p1, arg2, arg3);
}

void CanfdBus::OnTxWorker(void *p1, void *, void *)
{
    const CanfdDevice bus = static_cast<CanfdDevice>(reinterpret_cast<uintptr_t>(p1));
    TxMailbox &mailbox = mailboxes_[bus];
    bool retry_pending = false;
    while (1) {
        k_sem_take(&mailbox.wake,
                   retry_pending ? K_MSEC(1) : K_FOREVER);
        retry_pending = false;
        CanfdFrame frame{};
        while (TakeNextFrame(bus, frame)) {
            int result = -EAGAIN;
            for (int32_t attempt = 0; attempt < 20; ++attempt) {
                result = bsp_canfd_transmit(bus, &frame, canfd_no_wait);
                if (result == 0) {
                    break;
                }
                if (result != -EAGAIN && result != -EBUSY) {
                    break;
                }
                k_sleep(K_MSEC(1));
            }
            if (result != 0) {
                atomic_inc(&tx_dropped_);

                retry_pending = true;
                break;
            }
        }
    }
}

int CanfdBus::EnsureBus(CanfdDevice bus)
{
    if (!ValidBus(bus)) {
        return -EINVAL;
    }
    if (!bsp_canfd_is_ready(bus)) {
        const CanfdConfig config{CANFD_MODE_NORMAL, CANFD_RETRANSMISSION_DISABLED};
        const int result = bsp_canfd_init(bus, &config);
        if (result != 0 && result != -EALREADY) {
            return result;
        }
    }
    if (!mailboxes_[bus].ready) {
        k_sem_init(&mailboxes_[bus].wake, 0, 1);
        k_thread_create(&mailboxes_[bus].thread, tx_stacks[bus],
                        K_THREAD_STACK_SIZEOF(tx_stacks[bus]), TxWorker,
                        reinterpret_cast<void *>(static_cast<uintptr_t>(bus)),
                        nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
        mailboxes_[bus].ready = true;
    }
    return bsp_canfd_add_rx_callback(bus, rx_callback, nullptr);
}

int CanfdBus::Init()
{
    if (atomic_get(&initialized_) != 0) {
        return 0;
    }
    k_mutex_lock(&init_lock_, K_FOREVER);
    if (atomic_get(&initialized_) != 0) {
        k_mutex_unlock(&init_lock_);
        return 0;
    }

    int first_error = 0;
    size_t ready_count = 0;
    for (uint8_t index = 0; index < CANFD_DEVICE_COUNT; ++index) {
        const int result = EnsureBus(static_cast<CanfdDevice>(index));
        if (result == 0 || result == -EALREADY) {
            ++ready_count;
        } else if (first_error == 0) {
            first_error = result;
        }
    }
    if (ready_count != 0) {
        atomic_set(&initialized_, 1);
        first_error = 0;
    }
    k_mutex_unlock(&init_lock_);
    return first_error;
}

int CanfdBus::Subscribe(CanfdDevice bus, uint32_t id, CanfdIdType id_type,
                        CanfdProtocol protocol, PlatformCanfdRxHandle &handle, int16_t payload_type)
{
    if (!ValidBus(bus) || (id_type == CANFD_ID_STANDARD && id > CAN_STD_ID_MASK) ||
        (id_type == CANFD_ID_EXTENDED && id > CAN_EXT_ID_MASK)) {
        return -EINVAL;
    }
    const int init_result = platform_canfd_init();
    if (init_result != 0 || !mailboxes_[bus].ready) {
        return init_result != 0 ? init_result : -ENODEV;
    }

    const k_spinlock_key_t key = k_spin_lock(&subscription_lock_);
    for (size_t index = 0; index < 64; ++index) {
        Subscription &entry = subscriptions_[index];
        if (entry.used && entry.bus == bus && entry.id == id && entry.id_type == id_type && entry.protocol == protocol && entry.payload_type == payload_type) {
            handle.slot = static_cast<uint8_t>(index);
            k_spin_unlock(&subscription_lock_, key);
            return 0;
        }
    }
    for (size_t index = 0; index < 64; ++index) {
        Subscription &entry = subscriptions_[index];
        if (!entry.used) {
            entry = {};
            entry.used = true;
            entry.bus = bus;
            entry.id = id;
            entry.id_type = id_type;
            entry.protocol = protocol;
            entry.payload_type = payload_type;
            handle.slot = static_cast<uint8_t>(index);
            k_spin_unlock(&subscription_lock_, key);
            return 0;
        }
    }
    k_spin_unlock(&subscription_lock_, key);
    return -ENOSPC;
}

int CanfdBus::Latest(const PlatformCanfdRxHandle &handle, PlatformCanfdSnapshot &snapshot)
{
    if (handle.slot >= 64) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&subscription_lock_);
    const Subscription &entry = subscriptions_[handle.slot];
    if (!entry.used) {
        k_spin_unlock(&subscription_lock_, key);
        return -ENOENT;
    }
    snapshot = entry.snapshot;
    k_spin_unlock(&subscription_lock_, key);
    return snapshot.sequence == 0 ? -ENODATA : 0;
}

int CanfdBus::Submit(CanfdDevice bus, const CanfdFrame &frame)
{
    if (!ValidBus(bus) || !ValidFrame(frame)) {
        return -EINVAL;
    }
    const int init_result = platform_canfd_init();
    if (init_result != 0 || !mailboxes_[bus].ready) {
        return init_result != 0 ? init_result : -ENODEV;
    }
    const k_spinlock_key_t key = k_spin_lock(&tx_lock_);
    TxMailbox::PeriodicEntry *slot = nullptr;
    for (auto &entry : periodic_) {
        if (entry.used && entry.bus == bus && entry.frame.id == frame.id && entry.frame.id_type == frame.id_type &&
            entry.frame.protocol == frame.protocol) {
            slot = &entry;
            break;
        }
        if (!entry.used && slot == nullptr) {
            slot = &entry;
        }
    }
    if (slot == nullptr) {
        k_spin_unlock(&tx_lock_, key);
        atomic_inc(&tx_dropped_);
        return -ENOSPC;
    }
    slot->used = true;
    slot->pending = true;
    slot->bus = bus;
    slot->frame = frame;
    k_spin_unlock(&tx_lock_, key);
    k_sem_give(&mailboxes_[bus].wake);
    return 0;
}

int CanfdBus::SendOnce(CanfdDevice bus, const CanfdFrame &frame)
{
    if (!ValidBus(bus) || !ValidFrame(frame)) {
        return -EINVAL;
    }
    const int init_result = platform_canfd_init();
    if (init_result != 0 || !mailboxes_[bus].ready) {
        return init_result != 0 ? init_result : -ENODEV;
    }
    const k_spinlock_key_t key = k_spin_lock(&tx_lock_);
    TxMailbox &mailbox = mailboxes_[bus];
    if (mailbox.admin_count == 8) {
        k_spin_unlock(&tx_lock_, key);
        atomic_inc(&tx_dropped_);
        return -ENOBUFS;
    }
    mailbox.admin[mailbox.admin_tail] = frame;
    mailbox.admin_tail = (mailbox.admin_tail + 1) % 8;
    ++mailbox.admin_count;
    k_spin_unlock(&tx_lock_, key);
    k_sem_give(&mailbox.wake);
    return 0;
}

uint32_t CanfdBus::RxDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&rx_dropped_));
}

uint32_t CanfdBus::TxDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&tx_dropped_));
}

int platform_canfd_init()
{
    return canfd_bus.Init();
}

int platform_canfd_subscribe(CanfdDevice bus, uint32_t id, CanfdIdType id_type,
                             CanfdProtocol protocol, PlatformCanfdRxHandle &handle, int16_t payload_type)
{
    return canfd_bus.Subscribe(bus, id, id_type, protocol, handle, payload_type);
}

int platform_canfd_latest(const PlatformCanfdRxHandle &handle, PlatformCanfdSnapshot &snapshot)
{
    return canfd_bus.Latest(handle, snapshot);
}

int platform_canfd_submit(CanfdDevice bus, const CanfdFrame &frame)
{
    return canfd_bus.Submit(bus, frame);
}

int platform_canfd_send_once(CanfdDevice bus, const CanfdFrame &frame)
{
    return canfd_bus.SendOnce(bus, frame);
}

uint32_t platform_canfd_rx_dropped_count()
{
    return canfd_bus.RxDroppedCount();
}

uint32_t platform_canfd_tx_dropped_count()
{
    return canfd_bus.TxDroppedCount();
}
