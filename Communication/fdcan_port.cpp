#include "fdcan_port.hpp"

#include <string.h>

namespace
{
    constexpr size_t max_motors = FDCAN_PORT_MAX_MOTORS;
    constexpr size_t max_raw_subscriptions = 32U;
    constexpr size_t max_rx_slots = max_motors + max_raw_subscriptions;
    static_assert(max_rx_slots <= UINT8_MAX);
    constexpr size_t max_periodic_frames = FDCAN_CONTROL_SLOT_COUNT;
    constexpr size_t one_shot_depth = 8U;
    constexpr size_t standard_id_count = 0x800U;
    constexpr uint32_t tx_timeout_ms = 5U;

    enum class motor_kind : uint8_t
    {
        c610,
        c620,
        dm,
        cubemars
    };

    struct rx_item
    {
        fdcan_device bus;
        fdcan_frame frame;
        uint32_t timestamp_ms;
    };

    struct raw_feedback
    {
        uint8_t data[8];
        uint8_t length;
        uint32_t timestamp_ms;
        uint32_t generation;
    };

    struct raw_subscription
    {
        fdcan_device bus;
        uint32_t id;
    };

    struct motor_entry
    {
        bool used;
        motor_kind kind;
        void *motor;
        fdcan_device bus;
        uint32_t feedback_id;
        fdcan_protocol protocol;
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

    struct periodic_entry
    {
        bool used;
        bool pending;
        bool eligible;
        fdcan_device bus;
        fdcan_frame frame;
        uint32_t generation;
    };

    struct one_shot_queue
    {
        fdcan_frame frames[one_shot_depth];
        uint8_t head;
        uint8_t tail;
        uint8_t count;
    };

    struct inflight_frame
    {
        bool used;
        bool admin;
        fdcan_frame frame;
        size_t slot;
        uint32_t generation;
        uint32_t started_ms;
    };

    K_SEM_DEFINE(port_sem, 0, 1);
    K_MUTEX_DEFINE(init_lock);
    K_MUTEX_DEFINE(feedback_decode_lock);
    K_THREAD_STACK_DEFINE(port_stack, 2048);

    struct k_thread port_thread;
    struct k_timer port_timer;
    struct k_spinlock motor_lock;
    struct k_spinlock tx_lock;
    struct k_spinlock rx_lock;
    motor_entry motors[max_motors]{};
    uint8_t rx_routes[FDCAN_DEVICE_COUNT][standard_id_count]{};
    uint8_t next_rx_route[max_rx_slots]{};
    raw_feedback raw_feedbacks[max_rx_slots]{};
    uint32_t processed_generation[max_rx_slots]{};
    raw_subscription raw_subscriptions[max_raw_subscriptions]{};
    size_t raw_subscription_count = 0U;
    size_t motor_count = 0U;
    periodic_entry periodic[max_periodic_frames]{};
    one_shot_queue one_shots[FDCAN_DEVICE_COUNT]{};
    inflight_frame inflight[FDCAN_DEVICE_COUNT]{};
    size_t next_periodic[FDCAN_DEVICE_COUNT]{};
    uint32_t submit_failure_since_ms[FDCAN_DEVICE_COUNT]{};
    bool submit_failure_active[FDCAN_DEVICE_COUNT]{};
    atomic_t tx_result[FDCAN_DEVICE_COUNT]{};
    atomic_t tick_due{};
    atomic_t tx_due{};
    atomic_t ready_bus_mask{};
    atomic_t rx_dropped{};
    atomic_t tx_errors{};
    atomic_t control_send_dropped{};
    atomic_t initialized{};
    bool bus_registered[FDCAN_DEVICE_COUNT]{};

    bool cube_enable_frame(const fdcan_frame &frame)
    {
        if (frame.id_type != FDCAN_ID_STANDARD || frame.id != 0x01U ||
            frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8U || frame.data[7] != 0xFCU) {
            return false;
        }
        for (uint8_t index = 0U; index < 7U; ++index) {
            if (frame.data[index] != 0xFFU) {
                return false;
            }
        }
        return true;
    }

    void mark_cube_request_sent(fdcan_device bus, const fdcan_frame &frame)
    {
        const uint32_t now_ms = k_uptime_get_32();
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (motor_entry &entry : motors) {
            if (!entry.used || entry.kind != motor_kind::cubemars || entry.bus != bus) {
                continue;
            }
            if (entry.request_count == entry.response_count) {
                entry.pending_since_ms = now_ms;
            }
            ++entry.request_count;
            if (cube_enable_frame(frame)) {
                entry.enable_pending = true;
            }
            break;
        }
        k_spin_unlock(&motor_lock, key);
    }

    void mark_cube_response(void *motor)
    {
        const uint32_t now_ms = k_uptime_get_32();
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (motor_entry &entry : motors) {
            if (entry.used && entry.kind == motor_kind::cubemars && entry.motor == motor &&
                entry.response_count < entry.request_count) {
                ++entry.response_count;
                // A response proves the motor is alive. If requests are still
                // outstanding, restart the no-response window from this frame.
                entry.pending_since_ms = entry.response_count < entry.request_count ? now_ms : 0U;
                if (entry.enable_pending) {
                    entry.enable_acknowledged = true;
                    entry.enable_pending = false;
                }
                break;
            }
        }
        k_spin_unlock(&motor_lock, key);
    }

    bool valid_bus(fdcan_device bus)
    {
        return bus >= FDCAN_DEVICE_CAN0 && bus < FDCAN_DEVICE_COUNT;
    }

    bool valid_frame(const fdcan_frame &frame)
    {
        if (frame.id_type == FDCAN_ID_STANDARD && frame.id > 0x7FFU) {
            return false;
        }
        if (frame.id_type == FDCAN_ID_EXTENDED && frame.id > 0x1FFFFFFFU) {
            return false;
        }
        if (frame.id_type != FDCAN_ID_STANDARD && frame.id_type != FDCAN_ID_EXTENDED) {
            return false;
        }
        if (frame.protocol == FDCAN_PROTOCOL_CLASSIC) {
            return frame.length <= 8U;
        }
        return frame.protocol == FDCAN_PROTOCOL_FD && frame.length <= FDCAN_MAX_DATA_LENGTH;
    }

    bool cube_control_blocked(const FdcanControlTopicData &control)
    {
        if (control.frame.id_type != FDCAN_ID_STANDARD || control.frame.id != 0x01U ||
            control.frame.protocol != FDCAN_PROTOCOL_CLASSIC) {
            return false;
        }
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (const motor_entry &entry : motors) {
            if (entry.used && entry.kind == motor_kind::cubemars && entry.bus == control.bus) {
                const bool blocked = entry.offline || !entry.enable_acknowledged;
                k_spin_unlock(&motor_lock, key);
                return blocked;
            }
        }
        k_spin_unlock(&motor_lock, key);
        return false;
    }

    void rx_callback(fdcan_device bus, const fdcan_rx_view *frame, void *)
    {
        if (!valid_bus(bus) || frame == nullptr || frame->id_type != FDCAN_ID_STANDARD ||
            frame->id >= standard_id_count || frame->length > 8U) {
            return;
        }
        uint8_t matched[max_rx_slots]{};
        size_t count = 0U;
        const k_spinlock_key_t motor_key = k_spin_lock(&motor_lock);
        uint8_t route = rx_routes[bus][frame->id];
        while (route != 0U) {
            const size_t index = route - 1U;
            bool accepts = false;
            if (index < max_motors) {
                const motor_entry &entry = motors[index];
                accepts = frame->length >= 6U && entry.used && entry.bus == bus &&
                    entry.feedback_id == frame->id && entry.protocol == frame->protocol &&
                    (entry.kind != motor_kind::dm || (frame->data[0] & 0x0FU) == entry.motor_id);
            } else {
                const raw_subscription &entry = raw_subscriptions[index - max_motors];
                accepts = entry.bus == bus && entry.id == frame->id &&
                    frame->protocol == FDCAN_PROTOCOL_CLASSIC;
            }
            if (accepts) {
                matched[count++] = static_cast<uint8_t>(index);
            }
            route = next_rx_route[index];
        }
        k_spin_unlock(&motor_lock, motor_key);
        if (count == 0U) {
            atomic_inc(&rx_dropped);
            return;
        }
        const uint32_t timestamp_ms = k_uptime_get_32();
        for (size_t matched_index = 0U; matched_index < count; ++matched_index) {
            const size_t slot = matched[matched_index];
            const k_spinlock_key_t rx_key = k_spin_lock(&rx_lock);
            raw_feedback &target = raw_feedbacks[slot];
            if (target.generation != processed_generation[slot]) {
                atomic_inc(&rx_dropped);
            }
            memcpy(target.data, frame->data, sizeof(target.data));
            target.length = frame->length;
            target.timestamp_ms = timestamp_ms;
            ++target.generation;
            k_spin_unlock(&rx_lock, rx_key);
            if (slot < max_motors) {
                atomic_inc(&motors[slot].received_count);
            }
        }
    }

    void timer_callback(struct k_timer *)
    {
        if (atomic_get(&ready_bus_mask) == 0) {
            return;
        }
        atomic_set(&tick_due, 1);
        k_sem_give(&port_sem);
    }

    void tx_callback(fdcan_device bus, int error, void *)
    {
        if (error != 0) {
            atomic_inc(&tx_errors);
        }
        atomic_set(&tx_result[bus], error == 0 ? 1 : 2);
        atomic_set(&tx_due, 1);
        k_sem_give(&port_sem);
    }

    int topic_control_sink(const FdcanControlTopicData &control)
    {
        return fdcan_port_submit(control.bus, control.frame);
    }

    void dispatch(const rx_item &item)
    {
        if (item.frame.id_type != FDCAN_ID_STANDARD || item.frame.length < 6U) {
            return;
        }

        motor_kind kind{};
        void *motor = nullptr;
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (motor_entry &entry : motors) {
            if (!entry.used || entry.bus != item.bus || entry.feedback_id != item.frame.id || entry.protocol != item.frame.protocol) {
                continue;
            }
            if (entry.kind == motor_kind::dm && (item.frame.data[0] & 0x0FU) != entry.motor_id) {
                continue;
            }

            kind = entry.kind;
            motor = entry.motor;
            break;
        }
        k_spin_unlock(&motor_lock, key);

        if (motor == nullptr) {
            return;
        }
        FdcanFeedbackTopicData feedback{};
        feedback.bus = item.bus;
        feedback.id = item.frame.id;
        feedback.timestamp_ms = item.timestamp_ms;
        int result = -ENOMSG;
        switch (kind) {
            case motor_kind::c610:
                result = static_cast<c610 *>(motor)->process_feedback(item.frame);
                if (result == 0) {
                    const C610Data data = static_cast<c610 *>(motor)->get_data();
                    feedback.kind = FdcanMotorKind::c610;
                    feedback.speed_rad_s = data.now_omega;
                    feedback.angle_rad = data.now_angle;
                    feedback.current_a = data.now_current;
                    feedback.valid_count = static_cast<c610 *>(motor)->get_feedback_count();
                }
                break;
            case motor_kind::c620:
                result = static_cast<c620 *>(motor)->process_feedback(item.frame);
                if (result == 0) {
                    const C620Data data = static_cast<c620 *>(motor)->get_data();
                    feedback.kind = FdcanMotorKind::c620;
                    feedback.speed_rad_s = data.now_omega;
                    feedback.angle_rad = data.now_angle;
                    feedback.current_a = data.now_current;
                    feedback.valid_count = static_cast<c620 *>(motor)->get_feedback_count();
                }
                break;
            case motor_kind::dm:
                result = static_cast<dm_motor *>(motor)->process_feedback(item.frame);
                if (result == 0) {
                    const DmData data = static_cast<dm_motor *>(motor)->get_data();
                    feedback.kind = FdcanMotorKind::dm;
                    feedback.speed_rad_s = data.now_omega;
                    feedback.angle_rad = data.now_angle;
                    feedback.torque_nm = data.now_torque;
                    feedback.valid_count = static_cast<dm_motor *>(motor)->get_feedback_count();
                }
                break;
            case motor_kind::cubemars:
                result = static_cast<cubemars *>(motor)->process_feedback(item.frame);
                if (result == 0) {
                    const CubemarsData data = static_cast<cubemars *>(motor)->get_data();
                    feedback.kind = FdcanMotorKind::cubemars;
                    feedback.speed_rad_s = data.now_omega;
                    feedback.angle_rad = data.now_total_angle;
                    feedback.torque_nm = data.now_torque;
                    feedback.valid_count = static_cast<cubemars *>(motor)->get_feedback_count();
                }
                break;
        }
        if (result == 0 && kind == motor_kind::cubemars) {
            mark_cube_response(motor);
        }
        if (result == 0 && !(kind == motor_kind::dm && item.frame.data[0] == static_cast<dm_motor *>(motor)->motor_id() && item.frame.data[1] == 0U && item.frame.data[2] == 0x55U && item.frame.data[3] == 0x0AU)) {
            fdcan_topic_publish_feedback(feedback);
        }
    }

    void refresh_feedback_locked(fdcan_device bus, uint32_t id, FdcanMotorKind kind)
    {
        size_t slot = max_motors;
        const k_spinlock_key_t motor_key = k_spin_lock(&motor_lock);
        for (size_t index = 0U; index < motor_count; ++index) {
            const motor_entry &entry = motors[index];
            if (entry.used && entry.bus == bus && entry.feedback_id == id &&
                static_cast<uint8_t>(entry.kind) == static_cast<uint8_t>(kind)) {
                slot = index;
                break;
            }
        }
        k_spin_unlock(&motor_lock, motor_key);
        if (slot != max_motors) {
            raw_feedback snapshot{};
            bool changed = false;
            const k_spinlock_key_t rx_key = k_spin_lock(&rx_lock);
            snapshot = raw_feedbacks[slot];
            changed = snapshot.generation != processed_generation[slot];
            if (changed) {
                processed_generation[slot] = snapshot.generation;
            }
            k_spin_unlock(&rx_lock, rx_key);
            if (changed) {
                rx_item item{};
                item.bus = bus;
                item.timestamp_ms = snapshot.timestamp_ms;
                item.frame.id = id;
                item.frame.id_type = FDCAN_ID_STANDARD;
                item.frame.protocol = motors[slot].protocol;
                item.frame.length = snapshot.length;
                memcpy(item.frame.data, snapshot.data, snapshot.length);
                dispatch(item);
            }
        }
    }

    void refresh_feedback(fdcan_device bus, uint32_t id, FdcanMotorKind kind)
    {
        k_mutex_lock(&feedback_decode_lock, K_FOREVER);
        refresh_feedback_locked(bus, id, kind);
        k_mutex_unlock(&feedback_decode_lock);
    }

    void refresh_feedback_batch(const FdcanFeedbackKey *keys, size_t count)
    {
        k_mutex_lock(&feedback_decode_lock, K_FOREVER);
        for (size_t index = 0; index < count; ++index) {
            refresh_feedback_locked(keys[index].bus, keys[index].id, keys[index].kind);
        }
        k_mutex_unlock(&feedback_decode_lock);
    }

    int stage_control(const FdcanControlTopicData &control)
    {
        if (cube_control_blocked(control)) {
            const k_spinlock_key_t key = k_spin_lock(&tx_lock);
            for (periodic_entry &entry : periodic) {
                if (entry.used && entry.bus == control.bus && entry.frame.id == control.frame.id &&
                    entry.frame.id_type == control.frame.id_type &&
                    entry.frame.protocol == control.frame.protocol) {
                    entry.pending = false;
                    entry.eligible = false;
                    ++entry.generation;
                    break;
                }
            }
            k_spin_unlock(&tx_lock, key);
            return 0;
        }
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        periodic_entry *slot = nullptr;
        periodic_entry *free_slot = nullptr;
        periodic_entry *idle_slot = nullptr;
        for (periodic_entry &entry : periodic) {
            if (!entry.used) {
                if (free_slot == nullptr) {
                    free_slot = &entry;
                }
            } else if (entry.bus == control.bus && entry.frame.id == control.frame.id &&
                       entry.frame.id_type == control.frame.id_type &&
                       entry.frame.protocol == control.frame.protocol) {
                slot = &entry;
                break;
            } else if (!entry.pending && idle_slot == nullptr) {
                idle_slot = &entry;
            }
        }
        if (slot == nullptr) {
            slot = free_slot != nullptr ? free_slot : idle_slot;
        }
        if (slot == nullptr) {
            k_spin_unlock(&tx_lock, key);
            return -ENOSPC;
        }
        slot->used = true;
        slot->pending = true;
        slot->eligible = true;
        slot->bus = control.bus;
        slot->frame = control.frame;
        ++slot->generation;
        atomic_or(&ready_bus_mask, BIT(control.bus));
        k_spin_unlock(&tx_lock, key);
        return 0;
    }

    void complete_tx(fdcan_device bus)
    {
        const int result = atomic_set(&tx_result[bus], 0);
        inflight_frame &sent = inflight[bus];
        if (result == 0 || !sent.used) {
            return;
        }
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        if (sent.admin) {
            if (one_shots[bus].count != 0U) {
                one_shot_queue &queue = one_shots[bus];
                queue.head = static_cast<uint8_t>((queue.head + 1U) % one_shot_depth);
                --queue.count;
            }
        } else {
            periodic_entry &entry = periodic[sent.slot];
            if (entry.generation == sent.generation) {
                entry.pending = false;
                entry.eligible = false;
            }
        }
        k_spin_unlock(&tx_lock, key);
        sent.used = false;
    }

    void discard_selected(fdcan_device bus, bool admin, size_t slot, uint32_t generation)
    {
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        if (admin) {
            one_shot_queue &queue = one_shots[bus];
            if (queue.count != 0U) {
                queue.head = static_cast<uint8_t>((queue.head + 1U) % one_shot_depth);
                --queue.count;
            }
        } else if (periodic[slot].generation == generation) {
            periodic[slot].pending = false;
            periodic[slot].eligible = false;
        }
        k_spin_unlock(&tx_lock, key);
    }

    void send_next(fdcan_device bus)
    {
        if (inflight[bus].used || submit_failure_active[bus]) {
            return;
        }
        fdcan_frame frame{};
        bool admin = false;
        size_t slot = max_periodic_frames;
        uint32_t generation = 0U;
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        if (one_shots[bus].count != 0U) {
            frame = one_shots[bus].frames[one_shots[bus].head];
            admin = true;
        }
        k_spin_unlock(&tx_lock, key);
        if (!admin) {
            size_t offset = 0U;
            while (offset < max_periodic_frames) {
                const k_spinlock_key_t scan_key = k_spin_lock(&tx_lock);
                for (; offset < max_periodic_frames; ++offset) {
                    const size_t index = (next_periodic[bus] + offset) % max_periodic_frames;
                    const periodic_entry &entry = periodic[index];
                    if (entry.used && entry.pending && entry.eligible && entry.bus == bus) {
                        frame = entry.frame;
                        generation = entry.generation;
                        slot = index;
                        ++offset;
                        break;
                    }
                }
                k_spin_unlock(&tx_lock, scan_key);
                if (slot == max_periodic_frames || !cube_control_blocked({bus, frame})) {
                    break;
                }
                slot = max_periodic_frames;
            }
            if (slot == max_periodic_frames) {
                const k_spinlock_key_t idle_key = k_spin_lock(&tx_lock);
                bool ready = one_shots[bus].count != 0U;
                for (const periodic_entry &entry : periodic) {
                    ready |= entry.used && entry.pending && entry.eligible && entry.bus == bus;
                }
                if (!ready) {
                    atomic_and(&ready_bus_mask, ~BIT(bus));
                }
                k_spin_unlock(&tx_lock, idle_key);
                return;
            }
        }

        if (bsp_fdcan_tx_pending(bus) != 0U) {
            discard_selected(bus, admin, slot, generation);
            submit_failure_active[bus] = true;
            submit_failure_since_ms[bus] = k_uptime_get_32();
            return;
        }
        const int ret = bsp_fdcan_transmit(bus, &frame, FDCAN_NO_WAIT);
        if (ret != 0) {
            atomic_inc(&tx_errors);
            discard_selected(bus, admin, slot, generation);
            submit_failure_active[bus] = true;
            submit_failure_since_ms[bus] = k_uptime_get_32();
            return;
        }
        inflight[bus] = {true, admin, frame, slot, generation, k_uptime_get_32()};
        if (!admin) {
            const k_spinlock_key_t staged_key = k_spin_lock(&tx_lock);
            if (periodic[slot].generation == generation) {
                periodic[slot].eligible = false;
            }
            next_periodic[bus] = (slot + 1U) % max_periodic_frames;
            k_spin_unlock(&tx_lock, staged_key);
        }
        if (frame.id_type == FDCAN_ID_STANDARD && frame.id == 0x01U) {
            mark_cube_request_sent(bus, frame);
        }
    }

    void recover_stuck_tx()
    {
        const uint32_t now_ms = k_uptime_get_32();
        for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
            if (!bus_registered[index] ||
                (!inflight[index].used && !submit_failure_active[index])) {
                continue;
            }
            const fdcan_device bus = static_cast<fdcan_device>(index);
            const bool timed_out = inflight[index].used &&
                now_ms - inflight[index].started_ms >= tx_timeout_ms;
            const bool submit_stalled = submit_failure_active[index] &&
                now_ms - submit_failure_since_ms[index] >= tx_timeout_ms;
            if (!timed_out && !submit_stalled) {
                continue;
            }
            if (bsp_fdcan_recover(bus, FDCAN_NO_WAIT) != 0) {
                atomic_inc(&tx_errors);
            } else {
                submit_failure_active[index] = false;
                if (inflight[index].used && atomic_cas(&tx_result[index], 0, 2)) {
                    k_sem_give(&port_sem);
                }
            }
        }
    }

    void send_ready()
    {
        const atomic_val_t ready = atomic_get(&ready_bus_mask);
        for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
            const fdcan_device bus = static_cast<fdcan_device>(index);
            if (!bus_registered[index] || (ready & BIT(index)) == 0) {
                continue;
            }
            send_next(bus);
        }
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&port_sem, K_FOREVER);
            for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
                complete_tx(static_cast<fdcan_device>(index));
            }
            const bool tick = atomic_set(&tick_due, 0) != 0;
            if (tick) {
                recover_stuck_tx();
            }
            const bool tx = atomic_set(&tx_due, 0) != 0;
            if (tick || tx) {
                send_ready();
            }
        }
    }

    int bind_motor(motor_kind kind, void *motor, fdcan_device bus, uint32_t feedback_id,
                   fdcan_protocol protocol, uint8_t motor_id, bool automatic_recovery = true)
    {
        if (motor == nullptr || !valid_bus(bus) || feedback_id > 0x7FFU) {
            return -EINVAL;
        }
        const int ret = fdcan_port_init();
        if (ret != 0) {
            return ret;
        }
        if (!bus_registered[bus]) {
            return -ENODEV;
        }
        if (protocol == FDCAN_PROTOCOL_FD && bsp_fdcan_get_data_bitrate(bus) == 0U) {
            return -ENOTSUP;
        }

        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        motor_entry *free_entry = nullptr;
        for (motor_entry &entry : motors) {
            if (!entry.used) {
                if (free_entry == nullptr) {
                    free_entry = &entry;
                }
                continue;
            }
            if (entry.motor == motor) {
                k_spin_unlock(&motor_lock, key);
                return -EALREADY;
            }
            if (entry.bus == bus && entry.feedback_id == feedback_id && entry.protocol == protocol && (entry.kind != motor_kind::dm || kind != motor_kind::dm || entry.motor_id == motor_id)) {
                k_spin_unlock(&motor_lock, key);
                return -EADDRINUSE;
            }
        }
        if (free_entry == nullptr) {
            k_spin_unlock(&motor_lock, key);
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
        free_entry->request_count = 0U;
        free_entry->response_count = 0U;
        free_entry->pending_since_ms = 0U;
        free_entry->offline = kind == motor_kind::cubemars;
        free_entry->enable_pending = false;
        free_entry->enable_acknowledged = false;
        free_entry->automatic_recovery = automatic_recovery;
        const size_t slot = static_cast<size_t>(free_entry - motors);
        next_rx_route[slot] = rx_routes[bus][feedback_id];
        rx_routes[bus][feedback_id] = static_cast<uint8_t>(slot + 1U);
        ++motor_count;
        k_spin_unlock(&motor_lock, key);
        return 0;
    }

    uint32_t received_count(const void *motor)
    {
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (const motor_entry &entry : motors) {
            if (entry.used && entry.motor == motor) {
                const uint32_t count = static_cast<uint32_t>(atomic_get(&entry.received_count));
                k_spin_unlock(&motor_lock, key);
                return count;
            }
        }
        k_spin_unlock(&motor_lock, key);
        return 0U;
    }
}

int fdcan_port_init()
{
    if (atomic_get(&initialized) != 0) {
        return 0;
    }
    k_mutex_lock(&init_lock, K_FOREVER);
    if (atomic_get(&initialized) != 0) {
        k_mutex_unlock(&init_lock);
        return 0;
    }

    fdcan_topic_init();

    for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
        const fdcan_device bus = static_cast<fdcan_device>(index);
        int bus_result = 0;
        if (!bsp_fdcan_is_ready(bus)) {
            const fdcan_config config{FDCAN_MODE_NORMAL, FDCAN_RETRANSMISSION_DISABLED};
            bus_result = bsp_fdcan_init(bus, &config);
            if (bus_result == -EALREADY) {
                bus_result = 0;
            }
        }
        if (bus_result == 0) {
            bus_result = bsp_fdcan_set_rx_callback(bus, rx_callback, nullptr);
        }
        if (bus_result == 0) {
            bus_result = bsp_fdcan_set_tx_callback(bus, tx_callback, nullptr);
        }
        if (bus_result != 0) {
            if (bus == FDCAN_DEVICE_CAN0) {
                k_mutex_unlock(&init_lock);
                return bus_result;
            }
            continue;
        }
        bus_registered[index] = true;
    }

    k_timer_init(&port_timer, timer_callback, nullptr);
    k_thread_create(&port_thread, port_stack, K_THREAD_STACK_SIZEOF(port_stack), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    fdcan_topic_set_control_sink(topic_control_sink);
    fdcan_topic_set_feedback_refresh(refresh_feedback);
    fdcan_topic_set_feedback_refresh_batch(refresh_feedback_batch);
    // Sample all latest-value control slots together at the control-loop cadence.
    k_timer_start(&port_timer, K_MSEC(1), K_MSEC(1));
    atomic_set(&initialized, 1);
    k_mutex_unlock(&init_lock);
    return 0;
}

int fdcan_port_subscribe_raw(fdcan_device bus, uint32_t id)
{
    if (!valid_bus(bus) || id >= standard_id_count) {
        return -EINVAL;
    }
    const int ret = fdcan_port_init();
    if (ret != 0) {
        return ret;
    }
    if (!bus_registered[bus]) {
        return -ENODEV;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    for (size_t index = 0U; index < raw_subscription_count; ++index) {
        if (raw_subscriptions[index].bus == bus && raw_subscriptions[index].id == id) {
            k_spin_unlock(&motor_lock, key);
            return -EALREADY;
        }
    }
    if (raw_subscription_count == max_raw_subscriptions) {
        k_spin_unlock(&motor_lock, key);
        return -ENOSPC;
    }
    const size_t slot = max_motors + raw_subscription_count;
    raw_subscriptions[raw_subscription_count++] = {bus, id};
    next_rx_route[slot] = rx_routes[bus][id];
    rx_routes[bus][id] = static_cast<uint8_t>(slot + 1U);
    k_spin_unlock(&motor_lock, key);
    return 0;
}

int fdcan_port_latest_raw(fdcan_device bus, uint32_t id, FdcanRawSnapshot &snapshot)
{
    if (!valid_bus(bus) || id >= standard_id_count) {
        return -EINVAL;
    }
    size_t slot = max_rx_slots;
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    for (size_t index = 0U; index < raw_subscription_count; ++index) {
        if (raw_subscriptions[index].bus == bus && raw_subscriptions[index].id == id) {
            slot = max_motors + index;
            break;
        }
    }
    k_spin_unlock(&motor_lock, key);
    if (slot == max_rx_slots) {
        return -ENOENT;
    }
    const k_spinlock_key_t rx_key = k_spin_lock(&rx_lock);
    const raw_feedback value = raw_feedbacks[slot];
    if (value.generation != 0U) {
        processed_generation[slot] = value.generation;
    }
    k_spin_unlock(&rx_lock, rx_key);
    if (value.generation == 0U) {
        return -ENODATA;
    }
    memcpy(snapshot.data, value.data, sizeof(snapshot.data));
    snapshot.length = value.length;
    snapshot.timestamp_ms = value.timestamp_ms;
    return 0;
}

int fdcan_port_bind(c610 &motor)
{
    return bind_motor(motor_kind::c610, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, motor.motor_id());
}

int fdcan_port_bind(c620 &motor)
{
    return bind_motor(motor_kind::c620, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, motor.motor_id());
}

int fdcan_port_bind(dm_motor &motor, bool automatic_recovery)
{
    return bind_motor(motor_kind::dm, &motor, motor.device(), motor.feedback_id(), motor.feedback_protocol(), motor.motor_id(), automatic_recovery);
}

int fdcan_port_bind(cubemars &motor)
{
    return bind_motor(motor_kind::cubemars, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, 0U);
}

int fdcan_port_motor_health(size_t index, FdcanMotorHealthData &data)
{
    if (index >= max_motors) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    motor_entry entry = motors[index];
    k_spin_unlock(&motor_lock, key);
    if (!entry.used) {
        return -ENOENT;
    }

    refresh_feedback(entry.bus, entry.feedback_id, static_cast<FdcanMotorKind>(entry.kind));
    const k_spinlock_key_t updated_key = k_spin_lock(&motor_lock);
    entry = motors[index];
    k_spin_unlock(&motor_lock, updated_key);

    data.bus = entry.bus;
    data.feedback_id = entry.feedback_id;
    switch (entry.kind) {
        case motor_kind::c610: {
            const auto &motor = *static_cast<c610 *>(entry.motor);
            data.kind = FdcanMotorKind::c610;
            data.valid_feedback_count = motor.get_feedback_count();
            data.control_frame_id = motor.command_frame_id();
            data.status = 0U;
            break;
        }
        case motor_kind::c620: {
            const auto &motor = *static_cast<c620 *>(entry.motor);
            data.kind = FdcanMotorKind::c620;
            data.valid_feedback_count = motor.get_feedback_count();
            data.control_frame_id = motor.command_frame_id();
            data.status = 0U;
            break;
        }
        case motor_kind::dm: {
            const auto &motor = *static_cast<dm_motor *>(entry.motor);
            data.kind = FdcanMotorKind::dm;
            data.valid_feedback_count = motor.get_feedback_count();
            data.control_frame_id = motor.control_frame_id();
            data.status = motor.get_data().status;
            break;
        }
        case motor_kind::cubemars: {
            const auto &motor = *static_cast<cubemars *>(entry.motor);
            data.kind = FdcanMotorKind::cubemars;
            data.valid_feedback_count = motor.get_feedback_count();
            data.control_frame_id = motor.control_frame_id();
            data.status = 0U;
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

size_t fdcan_port_motor_count()
{
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    const size_t count = motor_count;
    k_spin_unlock(&motor_lock, key);
    return count;
}

void fdcan_port_set_motor_offline(size_t index, bool offline)
{
    if (index >= max_motors) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    if (!motors[index].used) {
        k_spin_unlock(&motor_lock, key);
        return;
    }
    const motor_kind kind = motors[index].kind;
    const fdcan_device bus = motors[index].bus;
    const uint32_t control_id = kind == motor_kind::cubemars ? static_cast<cubemars *>(motors[index].motor)->control_frame_id() : 0U;
    const bool was_offline = motors[index].offline;
    motors[index].offline = offline;
    if (kind == motor_kind::cubemars && offline && !was_offline) {
        motors[index].enable_pending = false;
        motors[index].enable_acknowledged = false;
    }
    k_spin_unlock(&motor_lock, key);

    if (kind == motor_kind::cubemars) {
        const k_spinlock_key_t tx_key = k_spin_lock(&tx_lock);
        for (periodic_entry &entry : periodic) {
            if (entry.used && entry.bus == bus && entry.frame.id_type == FDCAN_ID_STANDARD && entry.frame.id == control_id) {
                entry.pending = false;
                entry.eligible = false;
            }
        }
        k_spin_unlock(&tx_lock, tx_key);
    }
}

int fdcan_port_build_offline_frame(size_t index, fdcan_frame &frame)
{
    return fdcan_port_build_recovery_frame(index, false, frame);
}

int fdcan_port_build_recovery_frame(size_t index, bool clear_error, fdcan_frame &frame)
{
    if (index >= max_motors) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    const motor_entry entry = motors[index];
    k_spin_unlock(&motor_lock, key);
    if (!entry.used) {
        return -ENOENT;
    }
    switch (entry.kind) {
        case motor_kind::c610:
        case motor_kind::c620:
            return -ENOTSUP;
        case motor_kind::dm:
            return clear_error ? static_cast<dm_motor *>(entry.motor)->build_clear_error_frame(frame) : static_cast<dm_motor *>(entry.motor)->build_enable_frame(frame);
        case motor_kind::cubemars:
            return static_cast<cubemars *>(entry.motor)->build_enable_frame(frame);
    }
    return -EINVAL;
}

int fdcan_port_build_control_frame(size_t index, fdcan_frame &frame)
{
    if (index >= max_motors) {
        return -EINVAL;
    }
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    const motor_entry entry = motors[index];
    k_spin_unlock(&motor_lock, key);
    if (!entry.used) {
        return -ENOENT;
    }
    switch (entry.kind) {
        case motor_kind::c610:
            return static_cast<c610 *>(entry.motor)->build_control_frame(frame);
        case motor_kind::c620:
            return static_cast<c620 *>(entry.motor)->build_control_frame(frame);
        case motor_kind::dm:
            return static_cast<dm_motor *>(entry.motor)->build_control_frame(frame);
        case motor_kind::cubemars:
            return static_cast<cubemars *>(entry.motor)->build_control_frame(frame);
    }
    return -EINVAL;
}

int fdcan_port_submit(fdcan_device bus, const fdcan_frame &frame)
{
    if (!valid_bus(bus) || !valid_frame(frame)) {
        atomic_inc(&control_send_dropped);
        return -EINVAL;
    }
    const int ret = fdcan_port_init();
    if (ret != 0) {
        atomic_inc(&control_send_dropped);
        return ret;
    }
    if (!bus_registered[bus]) {
        atomic_inc(&control_send_dropped);
        return -ENODEV;
    }

    const int stage_result = stage_control({bus, frame});
    if (stage_result != 0) {
        atomic_inc(&control_send_dropped);
        return stage_result;
    }
    return 0;
}

int fdcan_port_submit(const c610 &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped);
        return ret;
    }
    return fdcan_port_submit(motor.device(), frame);
}

int fdcan_port_submit(const c620 &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped);
        return ret;
    }
    return fdcan_port_submit(motor.device(), frame);
}

int fdcan_port_submit(const dm_motor &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped);
        return ret;
    }
    return fdcan_port_submit(motor.device(), frame);
}

int fdcan_port_submit(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    if (ret != 0) {
        atomic_inc(&control_send_dropped);
        return ret;
    }
    return fdcan_port_submit(motor.device(), frame);
}

int fdcan_port_send_once(fdcan_device bus, const fdcan_frame &frame)
{
    if (!valid_bus(bus) || !valid_frame(frame)) {
        return -EINVAL;
    }
    const int ret = fdcan_port_init();
    if (ret != 0) {
        return ret;
    }
    if (!bus_registered[bus]) {
        return -ENODEV;
    }

    const k_spinlock_key_t key = k_spin_lock(&tx_lock);
    one_shot_queue &queue = one_shots[bus];
    for (uint8_t offset = 0U; offset < queue.count; ++offset) {
        const uint8_t queued_index = static_cast<uint8_t>((queue.head + offset) % one_shot_depth);
        const fdcan_frame &queued = queue.frames[queued_index];
        if (queued.id == frame.id && queued.id_type == frame.id_type &&
            queued.protocol == frame.protocol && queued.length == frame.length &&
            memcmp(queued.data, frame.data, frame.length) == 0) {
            k_spin_unlock(&tx_lock, key);
            return 0;
        }
    }
    if (queue.count == one_shot_depth) {
        k_spin_unlock(&tx_lock, key);
        return -ENOBUFS;
    }
    // Recovery/admin frames are one-shot traffic. Keep the latest periodic
    // control frame staged so it can resume immediately after recovery.
    queue.frames[queue.tail] = frame;
    queue.tail = static_cast<uint8_t>((queue.tail + 1U) % one_shot_depth);
    ++queue.count;
    atomic_or(&ready_bus_mask, BIT(bus));
    k_spin_unlock(&tx_lock, key);
    atomic_set(&tx_due, 1);
    k_sem_give(&port_sem);
    return 0;
}

int fdcan_port_request_mode(dm_motor &motor, DmControlMode mode)
{
    fdcan_frame frame{};
    int ret = motor.build_mode_frame(mode, frame);
    if (ret != 0) {
        return ret;
    }

    const fdcan_device bus = motor.device();
    const uint32_t old_control_id = motor.control_frame_id();
    if (!valid_bus(bus)) {
        return -EINVAL;
    }
    ret = fdcan_port_init();
    if (ret != 0) {
        return ret;
    }
    if (!bus_registered[bus]) {
        return -ENODEV;
    }

    const k_spinlock_key_t key = k_spin_lock(&tx_lock);
    one_shot_queue &queue = one_shots[bus];
    if (queue.count == one_shot_depth) {
        k_spin_unlock(&tx_lock, key);
        return -ENOBUFS;
    }
    ret = motor.mark_mode_request(mode);
    if (ret != 0) {
        k_spin_unlock(&tx_lock, key);
        return ret;
    }
    for (periodic_entry &entry : periodic) {
        if (entry.used && entry.bus == bus && entry.frame.id == old_control_id &&
            entry.frame.id_type == FDCAN_ID_STANDARD) {
            entry.pending = false;
            entry.eligible = false;
        }
    }
    queue.frames[queue.tail] = frame;
    queue.tail = static_cast<uint8_t>((queue.tail + 1U) % one_shot_depth);
    ++queue.count;
    atomic_or(&ready_bus_mask, BIT(bus));
    k_spin_unlock(&tx_lock, key);
    atomic_set(&tx_due, 1);
    k_sem_give(&port_sem);
    return 0;
}

int fdcan_port_enable(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_enable_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

int fdcan_port_disable(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_disable_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

int fdcan_port_save_zero(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_save_zero_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

uint32_t fdcan_port_received_count(const c610 &motor)
{
    return received_count(&motor);
}

uint32_t fdcan_port_received_count(const c620 &motor)
{
    return received_count(&motor);
}

uint32_t fdcan_port_received_count(const dm_motor &motor)
{
    return received_count(&motor);
}

uint32_t fdcan_port_received_count(const cubemars &motor)
{
    return received_count(&motor);
}

uint32_t fdcan_port_rx_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&rx_dropped));
}

uint32_t fdcan_port_tx_error_count()
{
    return static_cast<uint32_t>(atomic_get(&tx_errors));
}

uint32_t fdcan_port_control_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&control_send_dropped));
}
