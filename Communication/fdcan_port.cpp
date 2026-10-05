#include "fdcan_port.hpp"

namespace
{
    constexpr size_t max_motors = 32U;
    constexpr size_t max_periodic_frames = 32U;
    constexpr size_t one_shot_depth = 8U;
    constexpr size_t rx_batch = 16U;
    constexpr size_t rx_depth = 32U;
    constexpr uint8_t tx_timeout_ticks = 3U;

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

    alignas(4) char rx_storage[sizeof(rx_item) * rx_depth];
    struct k_msgq rx_queue{};
    K_SEM_DEFINE(port_sem, 0, 1);
    K_MUTEX_DEFINE(init_lock);
    K_THREAD_STACK_DEFINE(port_stack, 2048);

    struct k_thread port_thread;
    struct k_timer port_timer;
    struct k_spinlock motor_lock;
    struct k_spinlock tx_lock;
    motor_entry motors[max_motors]{};
    periodic_entry periodic[max_periodic_frames]{};
    one_shot_queue one_shots[FDCAN_DEVICE_COUNT]{};
    size_t next_periodic[FDCAN_DEVICE_COUNT]{};
    uint8_t tx_pending_ticks[FDCAN_DEVICE_COUNT]{};
    atomic_t tick_due{};
    atomic_t rx_dropped{};
    atomic_t rx_queued{};
    atomic_t rx_paused[FDCAN_DEVICE_COUNT]{};
    atomic_t tx_errors{};
    atomic_t control_send_dropped{};
    bool initialized = false;
    bool bus_registered[FDCAN_DEVICE_COUNT]{};

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

    void pause_rx(fdcan_device bus)
    {
        atomic_inc(&rx_dropped);
        if (!valid_bus(bus)) {
            return;
        }
        atomic_set(&rx_paused[bus], 1);
        (void)bsp_fdcan_set_rx_interrupt(bus, false);
    }

    void rx_callback(fdcan_device bus, const fdcan_frame *frame, void *)
    {
        if (frame == nullptr) {
            return;
        }
        if (atomic_get(&rx_queued) >= rx_depth) {
            pause_rx(bus);
            return;
        }
        const rx_item item{bus, *frame};
        if (k_msgq_put(&rx_queue, &item, K_NO_WAIT) != 0) {
            pause_rx(bus);
            return;
        }
        atomic_inc(&rx_queued);
    }

    void timer_callback(struct k_timer *)
    {
        atomic_set(&tick_due, 1);
        k_sem_give(&port_sem);
    }

    void tx_callback(fdcan_device, int error, void *)
    {
        if (error != 0) {
            atomic_inc(&tx_errors);
        }
        k_sem_give(&port_sem);
    }

    void control_notify()
    {
        k_sem_give(&port_sem);
    }

    void dispatch(const rx_item &item)
    {
        if (item.frame.id_type != FDCAN_ID_STANDARD || item.frame.length != 8U) {
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

            atomic_inc(&entry.received_count);
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
                (void)static_cast<cubemars *>(motor)->process_feedback(item.frame);
                return;
        }
        if (result == 0 && !(kind == motor_kind::dm && item.frame.data[0] == static_cast<dm_motor *>(motor)->motor_id() && item.frame.data[1] == 0U && item.frame.data[2] == 0x55U && item.frame.data[3] == 0x0AU)) {
            (void)fdcan_topic_publish_feedback(feedback);
        }
    }

    void drain_rx()
    {
        rx_item item{};
        size_t processed = 0U;
        while (processed < rx_batch && k_msgq_get(&rx_queue, &item, K_NO_WAIT) == 0) {
            atomic_dec(&rx_queued);
            dispatch(item);
            ++processed;
        }
        if (atomic_get(&rx_queued) <= rx_depth / 2U) {
            for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
                if (atomic_cas(&rx_paused[index], 1, 0)) {
                    (void)bsp_fdcan_set_rx_interrupt(static_cast<fdcan_device>(index), true);
                }
            }
        }
        if (processed == rx_batch) {
            k_sem_give(&port_sem);
        }
    }

    void drain_control()
    {
        FdcanControlTopicData control{};
        FdcanControlTopicData latest[8]{};
        size_t latest_count = 0U;
        size_t processed = 0U;
        while (processed < 8U && fdcan_topic_receive_control(control) == 0) {
            ++processed;
            if (!valid_bus(control.bus) || !bus_registered[control.bus] || !valid_frame(control.frame)) {
                atomic_inc(&control_send_dropped);
                continue;
            }
            size_t slot = 0U;
            while (slot < latest_count && (latest[slot].bus != control.bus || latest[slot].frame.id != control.frame.id || latest[slot].frame.id_type != control.frame.id_type)) {
                ++slot;
            }
            if (slot == latest_count) {
                ++latest_count;
            }
            latest[slot] = control;
        }
        for (size_t index = 0U; index < latest_count; ++index) {
            const int ret = bsp_fdcan_transmit(latest[index].bus, &latest[index].frame, FDCAN_NO_WAIT);
            if (ret != 0) {
                atomic_inc(&control_send_dropped);
            }
        }
        if (processed == 8U) {
            k_sem_give(&port_sem);
        }
    }

    bool send_one_shot(fdcan_device bus)
    {
        fdcan_frame frame{};
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        one_shot_queue &queue = one_shots[bus];
        if (queue.count == 0U) {
            k_spin_unlock(&tx_lock, key);
            return false;
        }
        frame = queue.frames[queue.head];
        k_spin_unlock(&tx_lock, key);

        const int ret = bsp_fdcan_transmit(bus, &frame, FDCAN_NO_WAIT);
        if (ret == -EAGAIN || ret == -EBUSY) {
            return true;
        }

        const k_spinlock_key_t done_key = k_spin_lock(&tx_lock);
        queue.head = static_cast<uint8_t>((queue.head + 1U) % one_shot_depth);
        --queue.count;
        k_spin_unlock(&tx_lock, done_key);
        if (ret != 0) {
            atomic_inc(&tx_errors);
        }
        return true;
    }

    void send_periodic(fdcan_device bus)
    {
        fdcan_frame frame{};
        uint32_t generation = 0U;
        size_t selected = max_periodic_frames;

        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        for (size_t offset = 0U; offset < max_periodic_frames; ++offset) {
            const size_t index = (next_periodic[bus] + offset) % max_periodic_frames;
            if (periodic[index].used && periodic[index].pending && periodic[index].eligible && periodic[index].bus == bus) {
                selected = index;
                frame = periodic[index].frame;
                generation = periodic[index].generation;
                next_periodic[bus] = (index + 1U) % max_periodic_frames;
                break;
            }
        }
        k_spin_unlock(&tx_lock, key);
        if (selected == max_periodic_frames) {
            return;
        }

        const int ret = bsp_fdcan_transmit(bus, &frame, FDCAN_NO_WAIT);
        if (ret == -EAGAIN || ret == -EBUSY) {
            return;
        }

        const k_spinlock_key_t done_key = k_spin_lock(&tx_lock);
        periodic[selected].eligible = false;
        if (periodic[selected].generation == generation) {
            periodic[selected].pending = false;
        }
        k_spin_unlock(&tx_lock, done_key);
        if (ret != 0) {
            atomic_inc(&tx_errors);
        }
    }

    void release_periodic()
    {
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        for (periodic_entry &entry : periodic) {
            if (entry.used && entry.pending) {
                entry.eligible = true;
            }
        }
        k_spin_unlock(&tx_lock, key);
    }

    void recover_stuck_tx()
    {
        for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
            if (!bus_registered[index]) {
                continue;
            }
            const fdcan_device bus = static_cast<fdcan_device>(index);
            fdcan_bus_state state{};
            const int state_result = bsp_fdcan_get_state(bus, &state, nullptr);
            if (state_result == 0 && (state == FDCAN_BUS_OFF || state == FDCAN_BUS_STOPPED)) {
                if (++tx_pending_ticks[index] < tx_timeout_ticks) {
                    continue;
                }
                tx_pending_ticks[index] = 0U;
                if (bsp_fdcan_recover(bus, FDCAN_NO_WAIT) != 0) {
                    atomic_inc(&tx_errors);
                }
                continue;
            }
            if (bsp_fdcan_tx_pending(bus) == 0U) {
                tx_pending_ticks[index] = 0U;
                continue;
            }
            if (++tx_pending_ticks[index] < tx_timeout_ticks) {
                continue;
            }

            tx_pending_ticks[index] = 0U;
            if (bsp_fdcan_recover(bus, FDCAN_NO_WAIT) != 0) {
                atomic_inc(&tx_errors);
            }
        }
    }

    void send_ready()
    {
        for (uint8_t index = 0U; index < FDCAN_DEVICE_COUNT; ++index) {
            const fdcan_device bus = static_cast<fdcan_device>(index);
            if (!bus_registered[index]) {
                continue;
            }
            if (bsp_fdcan_tx_pending(bus) != 0U) {
                continue;
            }
            if (!send_one_shot(bus)) {
                send_periodic(bus);
            }
        }
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&port_sem, K_FOREVER);
            if (atomic_set(&tick_due, 0) != 0) {
                release_periodic();
                recover_stuck_tx();
            }
            drain_control();
            send_ready();
            drain_rx();
        }
    }

    int bind_motor(motor_kind kind, void *motor, fdcan_device bus, uint32_t feedback_id, fdcan_protocol protocol, uint8_t motor_id)
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
    k_mutex_lock(&init_lock, K_FOREVER);
    if (initialized) {
        k_mutex_unlock(&init_lock);
        return 0;
    }

    fdcan_topic_init();
    k_msgq_init(&rx_queue, rx_storage, sizeof(rx_item), rx_depth);

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
    k_thread_create(&port_thread, port_stack, K_THREAD_STACK_SIZEOF(port_stack),
                    thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    fdcan_topic_set_control_notify(control_notify);
    k_timer_start(&port_timer, K_MSEC(1), K_MSEC(1));
    initialized = true;
    k_mutex_unlock(&init_lock);
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

int fdcan_port_bind(dm_motor &motor)
{
    return bind_motor(motor_kind::dm, &motor, motor.device(), motor.feedback_id(), motor.feedback_protocol(), motor.motor_id());
}

int fdcan_port_bind(cubemars &motor)
{
    return bind_motor(motor_kind::cubemars, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, 0U);
}

int fdcan_port_submit(fdcan_device bus, const fdcan_frame &frame)
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
    periodic_entry *slot = nullptr;
    periodic_entry *free_slot = nullptr;
    for (periodic_entry &entry : periodic) {
        if (!entry.used) {
            if (free_slot == nullptr) {
                free_slot = &entry;
            }
            continue;
        }
        if (entry.bus == bus && entry.frame.id == frame.id && entry.frame.id_type == frame.id_type && entry.frame.protocol == frame.protocol) {
            slot = &entry;
            break;
        }
    }
    if (slot == nullptr) {
        slot = free_slot;
    }
    if (slot == nullptr) {
        k_spin_unlock(&tx_lock, key);
        return -ENOSPC;
    }
    slot->used = true;
    slot->pending = true;
    slot->bus = bus;
    slot->frame = frame;
    ++slot->generation;
    k_spin_unlock(&tx_lock, key);
    return 0;
}

int fdcan_port_submit(const c610 &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    return ret == 0 ? fdcan_port_submit(motor.device(), frame) : ret;
}

int fdcan_port_submit(const c620 &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    return ret == 0 ? fdcan_port_submit(motor.device(), frame) : ret;
}

int fdcan_port_submit(const dm_motor &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    return ret == 0 ? fdcan_port_submit(motor.device(), frame) : ret;
}

int fdcan_port_submit(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_control_frame(frame);
    return ret == 0 ? fdcan_port_submit(motor.device(), frame) : ret;
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
    if (queue.count == one_shot_depth) {
        k_spin_unlock(&tx_lock, key);
        return -ENOBUFS;
    }
    for (periodic_entry &entry : periodic) {
        if (entry.used && entry.bus == bus && entry.frame.id == frame.id && entry.frame.id_type == frame.id_type) {
            entry.pending = false;
            entry.eligible = false;
        }
    }
    queue.frames[queue.tail] = frame;
    queue.tail = static_cast<uint8_t>((queue.tail + 1U) % one_shot_depth);
    ++queue.count;
    k_spin_unlock(&tx_lock, key);
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
    k_spin_unlock(&tx_lock, key);
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
