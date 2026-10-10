#include "fdcan_port.hpp"

#include <string.h>

namespace
{
    constexpr size_t max_motors = FDCAN_PORT_MAX_MOTORS;
    constexpr size_t max_raw_subscriptions = 32;
    constexpr size_t max_rx_slots = max_motors + max_raw_subscriptions;
    static_assert(max_rx_slots <= UINT8_MAX);
    constexpr size_t max_periodic_frames = FDCAN_CONTROL_SLOT_COUNT;
    constexpr size_t admin_slot_count = 8;
    constexpr uint8_t max_admin_burst = 4;
    constexpr size_t standard_id_count = 0x800;
    constexpr uint32_t tx_timeout_ms = 5;

    // 支持的电机类型：C610/C620（大疆）、DM、Cubemars
    enum class motor_kind : uint8_t
    {
        c610,
        c620,
        dm,
        cubemars
    };

    // 一条中转待进的接收项，携带总线、数据帧与时间戳
    struct rx_item
    {
        fdcan_device bus;
        fdcan_frame frame;
        uint32_t timestamp_ms;
    };

    // 原始反馈缓冲项，保存最近一次接收到的数据与代数（用于检测更新）
    struct raw_feedback
    {
        uint8_t data[8];
        uint8_t length;
        uint32_t timestamp_ms;
        uint32_t generation;
    };

    // 一路未绑定电机的原始数据订阅记录
    struct raw_subscription
    {
        fdcan_device bus;
        uint32_t id;
    };

    // 一台已绑定电机的描述表项，含类型、总线、反馈 ID、收发统计与离线状态
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

    // 周期发送帧槽位，用于存放控制回路反复下发的帧
    struct periodic_entry
    {
        bool used;
        bool pending;
        bool eligible;
        fdcan_device bus;
        fdcan_frame frame;
        uint32_t generation;
    };

    // 管理帧槽位，用于下发一次性的命令帧（如模式切换、使能等）
    struct admin_entry
    {
        bool used;
        bool pending;
        bool eligible;
        uint8_t target;
        fdcan_frame frame;
        uint32_t generation;
    };

    // 正在硬件发送中的帧描述，用于发送完成或超时后的回收
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
    size_t raw_subscription_count = 0;
    size_t motor_count = 0;
    periodic_entry periodic[max_periodic_frames]{};
    admin_entry admin_slots[FDCAN_DEVICE_COUNT][admin_slot_count]{};
    inflight_frame inflight[FDCAN_DEVICE_COUNT]{};
    size_t next_periodic[FDCAN_DEVICE_COUNT]{};
    size_t next_admin[FDCAN_DEVICE_COUNT]{};
    uint8_t admin_burst[FDCAN_DEVICE_COUNT]{};
    atomic_t tx_result[FDCAN_DEVICE_COUNT]{};
    atomic_t tick_due{};
    atomic_t tx_due{};
    atomic_t ready_bus_mask{};
    atomic_t rx_dropped{};
    atomic_t tx_errors{};
    atomic_t control_send_dropped{};
    atomic_t initialized{};
    bool bus_registered[FDCAN_DEVICE_COUNT]{};

    /**
     * @brief 判断给定帧是否为 Cubemars 电机使能帧（标准 ID 0x01，数据全 0xFF 且末字节 0xFC）
     *
     * @param frame 待判断的 CAN 帧
     * @return 是使能帧返回 true，否则返回 false
    */
    bool cube_enable_frame(const fdcan_frame &frame)
    {
        if (frame.id_type != FDCAN_ID_STANDARD || frame.id != 0x01 ||
            frame.protocol != FDCAN_PROTOCOL_CLASSIC || frame.length != 8 || frame.data[7] != 0xFC) {
            return false;
        }
        for (uint8_t index = 0; index < 7; ++index) {
            if (frame.data[index] != 0xFF) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 记录一次 Cubemars 请求帧已发送，并标记未确认时开启无响应计时
     *
     * @param bus   请求所在的总线
     * @param frame 已发送的 Cubemars 请求帧
    */
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

    /**
     * @brief 收到来自 Cubemars 电机的响应时更新回复计数，并确认使能已被接受
     *
     * @param motor 对应 Cubemars 电机对象指针
    */
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
                entry.pending_since_ms = entry.response_count < entry.request_count ? now_ms : 0;
                if (entry.enable_pending) {
                    entry.enable_acknowledged = true;
                    entry.enable_pending = false;
                }
                break;
            }
        }
        k_spin_unlock(&motor_lock, key);
    }

    /**
     * @brief 判断总线号是否在合法范围内
     *
     * @param bus 待判断的总线
     * @return 合法返回 true，否则返回 false
    */
    bool valid_bus(fdcan_device bus)
    {
        return bus >= FDCAN_DEVICE_CAN0 && bus < FDCAN_DEVICE_COUNT;
    }

    /**
     * @brief 判断 CAN 帧的 ID 类型、协议类型与长度是否合法
     *
     * @param frame 待校验的 CAN 帧
     * @return 合法返回 true，否则返回 false
    */
    bool valid_frame(const fdcan_frame &frame)
    {
        if (frame.id_type == FDCAN_ID_STANDARD && frame.id > 0x7FF) {
            return false;
        }
        if (frame.id_type == FDCAN_ID_EXTENDED && frame.id > 0x1FFFFFFF) {
            return false;
        }
        if (frame.id_type != FDCAN_ID_STANDARD && frame.id_type != FDCAN_ID_EXTENDED) {
            return false;
        }
        if (frame.protocol == FDCAN_PROTOCOL_CLASSIC) {
            return frame.length <= 8;
        }
        return frame.protocol == FDCAN_PROTOCOL_FD && frame.length <= fdcan_max_data_length;
    }

    /**
     * @brief 判断总线上 Cubemars 电机的控制帧是否应被阻止（电机处于离线或使能未确认）
     *
     * @param control 控制话题数据
     * @return 应阻止返回 true，否则返回 false
    */
    bool cube_control_blocked(const FdcanControlTopicData &control)
    {
        if (control.frame.id_type != FDCAN_ID_STANDARD || control.frame.id != 0x01 ||
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

    /**
     * @brief CAN 接收回调：按帧 ID 路由到匹配的电机/原始订阅槽位并保存原始反馈
     *
     * @param bus   接收到的总线
     * @param frame 接收到的 CAN 帧视图
     * @param       回调上下文，未使用
    */
    void rx_callback(fdcan_device bus, const fdcan_rx_view *frame, void *)
    {
        if (!valid_bus(bus) || frame == nullptr || frame->id_type != FDCAN_ID_STANDARD ||
            frame->id >= standard_id_count || frame->length > 8) {
            return;
        }
        uint8_t matched[max_rx_slots]{};
        size_t count = 0;
        const k_spinlock_key_t motor_key = k_spin_lock(&motor_lock);
        uint8_t route = rx_routes[bus][frame->id];
        while (route != 0) {
            const size_t index = route - 1;
            bool accepts = false;
            if (index < max_motors) {
                const motor_entry &entry = motors[index];
                accepts = frame->length >= 6 && entry.used && entry.bus == bus &&
                    entry.feedback_id == frame->id && entry.protocol == frame->protocol &&
                    (entry.kind != motor_kind::dm || (frame->data[0] & 0x0F) == entry.motor_id);
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
        if (count == 0) {
            atomic_inc(&rx_dropped);
            return;
        }
        const uint32_t timestamp_ms = k_uptime_get_32();
        for (size_t matched_index = 0; matched_index < count; ++matched_index) {
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

    /**
     * @brief 周期定时器回调：置位滴答标志并唤醒发送线程
     *
     * @param 定时器对象，未使用
    */
    void timer_callback(struct k_timer *)
    {
        if (atomic_get(&ready_bus_mask) == 0) {
            return;
        }
        atomic_set(&tick_due, 1);
        k_sem_give(&port_sem);
    }

    /**
     * @brief 发送完成回调：按结果更新错误计数与发送结果，并唤醒发送线程
     *
     * @param bus   发送完成的总线
     * @param error 发送错误码，0 表示成功
     * @param       回调上下文，未使用
    */
    void tx_callback(fdcan_device bus, int error, void *)
    {
        if (error != 0) {
            atomic_inc(&tx_errors);
        }
        atomic_set(&tx_result[bus], error == 0 ? 1 : 2);
        atomic_set(&tx_due, 1);
        k_sem_give(&port_sem);
    }

    /**
     * @brief 控制话题接收回调：将控制帧提交到周期发送队列
     *
     * @param control 接收到的控制话题数据
     * @return 提交结果，0 表示成功
    */
    int topic_control_sink(const FdcanControlTopicData &control)
    {
        return fdcan_port_submit(control.bus, control.frame);
    }

    /**
     * @brief 把接收到的项分发给匹配电机并解析反馈，随后发布反馈话题
     *
     * @param item 待分发的接收项（含总线、帧与时间戳）
    */
    void dispatch(const rx_item &item)
    {
        if (item.frame.id_type != FDCAN_ID_STANDARD || item.frame.length < 6) {
            return;
        }

        motor_kind kind{};
        void *motor = nullptr;
        const k_spinlock_key_t key = k_spin_lock(&motor_lock);
        for (motor_entry &entry : motors) {
            if (!entry.used || entry.bus != item.bus || entry.feedback_id != item.frame.id || entry.protocol != item.frame.protocol) {
                continue;
            }
            if (entry.kind == motor_kind::dm && (item.frame.data[0] & 0x0F) != entry.motor_id) {
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
        if (result == 0 && !(kind == motor_kind::dm && item.frame.data[0] == static_cast<dm_motor *>(motor)->motor_id() && item.frame.data[1] == 0 && item.frame.data[2] == 0x55 && item.frame.data[3] == 0x0A)) {
            fdcan_topic_publish_feedback(feedback);
        }
    }

    /**
     * @brief 在持有解码锁的前提下，将某电机的最新原始反馈刷新为解析结果并发布
     *
     * @param bus  反馈所在的总线
     * @param id   反馈帧 ID
     * @param kind 电机类型
    */
    void refresh_feedback_locked(fdcan_device bus, uint32_t id, FdcanMotorKind kind)
    {
        size_t slot = max_motors;
        const k_spinlock_key_t motor_key = k_spin_lock(&motor_lock);
        for (size_t index = 0; index < motor_count; ++index) {
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

    /**
     * @brief 加解码锁后刷新单个电机的反馈
     *
     * @param bus  反馈所在的总线
     * @param id   反馈帧 ID
     * @param kind 电机类型
    */
    void refresh_feedback(fdcan_device bus, uint32_t id, FdcanMotorKind kind)
    {
        k_mutex_lock(&feedback_decode_lock, K_FOREVER);
        refresh_feedback_locked(bus, id, kind);
        k_mutex_unlock(&feedback_decode_lock);
    }

    /**
     * @brief 加解码锁后批量刷新一组电机的反馈
     *
     * @param keys  待刷新的电机键数组
     * @param count 数组元素个数
    */
    void refresh_feedback_batch(const FdcanFeedbackKey *keys, size_t count)
    {
        k_mutex_lock(&feedback_decode_lock, K_FOREVER);
        for (size_t index = 0; index < count; ++index) {
            refresh_feedback_locked(keys[index].bus, keys[index].id, keys[index].kind);
        }
        k_mutex_unlock(&feedback_decode_lock);
    }

    /**
     * @brief 将控制帧登记进周期发送槽位；被阻止的控制帧会被停发
     *
     * @param control 控制话题数据
     * @return 成功返回 0，无空闲槽位返回 -ENOSPC
    */
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

    /**
     * @brief 查找某个总线上可用的管理帧槽位（优先匹配已存在的目标帧）
     *
     * @param bus    目标总线
     * @param frame  待登记的管理帧
     * @param target 目标电机 ID
     * @return 找到的槽位指针；找不到返回 nullptr
    */
    admin_entry *find_admin_slot(fdcan_device bus, const fdcan_frame &frame, uint8_t target)
    {
        admin_entry *free_slot = nullptr;
        admin_entry *idle_slot = nullptr;
        for (admin_entry &entry : admin_slots[bus]) {
            if (entry.used && entry.target == target && entry.frame.id == frame.id &&
                entry.frame.id_type == frame.id_type && entry.frame.protocol == frame.protocol) {
                return &entry;
            }
            if (!entry.used && free_slot == nullptr) {
                free_slot = &entry;
            } else if (entry.used && !entry.pending && idle_slot == nullptr) {
                idle_slot = &entry;
            }
        }
        return free_slot != nullptr ? free_slot : idle_slot;
    }

    /**
     * @brief 将管理帧写入指定槽位并标记为待发送
     *
     * @param entry  目标槽位
     * @param bus    目标总线
     * @param frame  待登记的管理帧
     * @param target 目标电机 ID
    */
    void stage_admin(admin_entry &entry, fdcan_device bus, const fdcan_frame &frame, uint8_t target)
    {
        entry.used = true;
        entry.pending = true;
        entry.eligible = true;
        entry.target = target;
        entry.frame = frame;
        ++entry.generation;
        atomic_or(&ready_bus_mask, BIT(bus));
    }

    /**
     * @brief 处理某总线当前帧发送完成，清除对应槽位的挂起状态
     *
     * @param bus 发送完成的总线
    */
    void complete_tx(fdcan_device bus)
    {
        const int result = atomic_set(&tx_result[bus], 0);
        inflight_frame &sent = inflight[bus];
        if (result == 0 || !sent.used) {
            return;
        }
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        if (sent.admin) {
            admin_entry &entry = admin_slots[bus][sent.slot];
            if (entry.generation == sent.generation) {
                entry.pending = false;
                entry.eligible = false;
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

    /**
     * @brief 丢弃指定槽位中待发送的帧（发送失败时清理）
     *
     * @param bus        目标总线
     * @param admin      是否为管理帧
     * @param slot       槽位索引
     * @param generation 帧代数，用于避免误删新帧
    */
    void discard_selected(fdcan_device bus, bool admin, size_t slot, uint32_t generation)
    {
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        if (admin) {
            admin_entry &entry = admin_slots[bus][slot];
            if (entry.generation == generation) {
                entry.pending = false;
                entry.eligible = false;
            }
            next_admin[bus] = (slot + 1) % admin_slot_count;
        } else {
            if (periodic[slot].generation == generation) {
                periodic[slot].pending = false;
                periodic[slot].eligible = false;
            }
            next_periodic[bus] = (slot + 1) % max_periodic_frames;
        }
        k_spin_unlock(&tx_lock, key);
    }

    /**
     * @brief 轮询选择一个待发送的管理帧并输出其信息
     *
     * @param bus        目标总线
     * @param frame      输出选中的帧
     * @param slot       输出槽位索引
     * @param generation 输出帧代数
     * @return 选中到管理帧返回 true，否则返回 false
    */
    bool select_admin(fdcan_device bus, fdcan_frame &frame, size_t &slot, uint32_t &generation)
    {
        const k_spinlock_key_t key = k_spin_lock(&tx_lock);
        for (size_t offset = 0; offset < admin_slot_count; ++offset) {
            const size_t index = (next_admin[bus] + offset) % admin_slot_count;
            const admin_entry &entry = admin_slots[bus][index];
            if (entry.used && entry.pending && entry.eligible) {
                frame = entry.frame;
                generation = entry.generation;
                slot = index;
                k_spin_unlock(&tx_lock, key);
                return true;
            }
        }
        k_spin_unlock(&tx_lock, key);
        return false;
    }

    /**
     * @brief 尝试为某总线调度并发送下一帧（管理帧或周期帧）
     *
     * @param bus 目标总线
    */
    void send_next(fdcan_device bus)
    {
        if (inflight[bus].used) {
            return;
        }
        fdcan_frame frame{};
        bool admin = false;
        size_t slot = max_periodic_frames;
        uint32_t generation = 0;
        if (admin_burst[bus] < max_admin_burst) {
            admin = select_admin(bus, frame, slot, generation);
        }
        if (!admin) {
            slot = max_periodic_frames;
            size_t offset = 0;
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
                admin = select_admin(bus, frame, slot, generation);
            }
            if (slot == max_periodic_frames) {
                const k_spinlock_key_t idle_key = k_spin_lock(&tx_lock);
                bool ready = false;
                for (const admin_entry &entry : admin_slots[bus]) {
                    ready |= entry.used && entry.pending && entry.eligible;
                }
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

        if (admin) {
            if (admin_burst[bus] < max_admin_burst) {
                ++admin_burst[bus];
            }
        } else {
            admin_burst[bus] = 0;
        }
        const int ret = bsp_fdcan_transmit(bus, &frame, fdcan_no_wait);
        if (ret != 0) {
            atomic_inc(&tx_errors);
            discard_selected(bus, admin, slot, generation);
            return;
        }
        inflight[bus] = {true, admin, frame, slot, generation, k_uptime_get_32()};
        const k_spinlock_key_t staged_key = k_spin_lock(&tx_lock);
        if (admin) {
            if (admin_slots[bus][slot].generation == generation) {
                admin_slots[bus][slot].eligible = false;
            }
            next_admin[bus] = (slot + 1) % admin_slot_count;
        } else {
            if (periodic[slot].generation == generation) {
                periodic[slot].eligible = false;
            }
            next_periodic[bus] = (slot + 1) % max_periodic_frames;
        }
        k_spin_unlock(&tx_lock, staged_key);
        if (frame.id_type == FDCAN_ID_STANDARD && frame.id == 0x01) {
            mark_cube_request_sent(bus, frame);
        }
    }

    /**
     * @brief 巡检并恢复长时间未完成的硬件发送
     *
    */
    void recover_stuck_tx()
    {
        const uint32_t now_ms = k_uptime_get_32();
        for (uint8_t index = 0; index < FDCAN_DEVICE_COUNT; ++index) {
            if (!bus_registered[index] || !inflight[index].used) {
                continue;
            }
            const fdcan_device bus = static_cast<fdcan_device>(index);
            if (now_ms - inflight[index].started_ms < tx_timeout_ms) {
                continue;
            }
            if (bsp_fdcan_recover(bus, fdcan_no_wait) != 0) {
                atomic_inc(&tx_errors);
            } else if (atomic_cas(&tx_result[index], 0, 2)) {
                k_sem_give(&port_sem);
            }
        }
    }

    /**
     * @brief 遍历所有就绪总线，为每条总线调用发送下一帧
     *
    */
    void send_ready()
    {
        const atomic_val_t ready = atomic_get(&ready_bus_mask);
        for (uint8_t index = 0; index < FDCAN_DEVICE_COUNT; ++index) {
            const fdcan_device bus = static_cast<fdcan_device>(index);
            if (bus_registered[index] && (ready & BIT(index)) != 0) {
                send_next(bus);
            }
        }
    }

    /**
     * @brief 发送线程入口：处理发送完成、超时巡检与就绪总线的帧发送
     *
    */
    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&port_sem, K_FOREVER);
            for (uint8_t index = 0; index < FDCAN_DEVICE_COUNT; ++index) {
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

    /**
     * @brief 将电机注册进电机表并建立接收路由，返回槽位索引
     *
     * @param kind              电机类型
     * @param motor             电机对象指针
     * @param bus               绑定总线
     * @param feedback_id       反馈帧 ID
     * @param protocol          通信协议
     * @param motor_id          电机 ID（DM 电机使用）
     * @param automatic_recovery 是否启用离线自动恢复
     * @return 成功返回 0，否则返回对应负错误码
    */
    int bind_motor(motor_kind kind, void *motor, fdcan_device bus, uint32_t feedback_id,
                   fdcan_protocol protocol, uint8_t motor_id, bool automatic_recovery = true)
    {
        if (motor == nullptr || !valid_bus(bus) || feedback_id > 0x7FF) {
            return -EINVAL;
        }
        const int ret = fdcan_port_init();
        if (ret != 0) {
            return ret;
        }
        if (!bus_registered[bus]) {
            return -ENODEV;
        }
        if (protocol == FDCAN_PROTOCOL_FD && bsp_fdcan_get_data_bitrate(bus) == 0) {
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
        free_entry->request_count = 0;
        free_entry->response_count = 0;
        free_entry->pending_since_ms = 0;
        free_entry->offline = kind == motor_kind::cubemars;
        free_entry->enable_pending = false;
        free_entry->enable_acknowledged = false;
        free_entry->automatic_recovery = automatic_recovery;
        const size_t slot = static_cast<size_t>(free_entry - motors);
        next_rx_route[slot] = rx_routes[bus][feedback_id];
        rx_routes[bus][feedback_id] = static_cast<uint8_t>(slot + 1);
        ++motor_count;
        k_spin_unlock(&motor_lock, key);
        return 0;
    }

    /**
     * @brief 查询某电机累计收到的有效反馈帧数量
     *
     * @param motor 电机对象指针
     * @return 累计反馈帧数量；未找到电机返回 0
    */
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
        return 0;
    }
}

/**
 * @brief 初始化 FDCAN 总线的发送/接收回调并启动发送线程（线程安全，仅初始化一次）
 *
 * @return 成功返回 0，否则返回对应负错误码
*/
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

    for (uint8_t index = 0; index < FDCAN_DEVICE_COUNT; ++index) {
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
    fdcan_topic_set_control_sink(topic_control_sink);
    fdcan_topic_set_feedback_refresh(refresh_feedback);
    fdcan_topic_set_feedback_refresh_batch(refresh_feedback_batch);
    // Sample all latest-value control slots together at the control-loop cadence.
    k_timer_start(&port_timer, K_MSEC(1), K_MSEC(1));
    atomic_set(&initialized, 1);
    k_mutex_unlock(&init_lock);
    return 0;
}

/**
 * @brief 订阅一路未绑定电机的原始 CAN 数据（标准帧，数据长度不超过 8 字节）
 *
 * @param bus 订阅所在的总线
 * @param id  订阅的 CAN 帧 ID
 * @return 成功返回 0；参数非法返回 -EINVAL，重复订阅返回 -EALREADY，槽位满返回 -ENOSPC
*/
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
    for (size_t index = 0; index < raw_subscription_count; ++index) {
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
    rx_routes[bus][id] = static_cast<uint8_t>(slot + 1);
    k_spin_unlock(&motor_lock, key);
    return 0;
}

/**
 * @brief 读取指定原始订阅的最新一帧快照
 *
 * @param bus      订阅所在的总线
 * @param id       订阅的 CAN 帧 ID
 * @param snapshot 输出最新帧快照
 * @return 成功返回 0；尚未收到数据返回 -ENODATA，未订阅返回 -ENOENT
*/
int fdcan_port_latest_raw(fdcan_device bus, uint32_t id, FdcanRawSnapshot &snapshot)
{
    if (!valid_bus(bus) || id >= standard_id_count) {
        return -EINVAL;
    }
    size_t slot = max_rx_slots;
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    for (size_t index = 0; index < raw_subscription_count; ++index) {
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
    if (value.generation != 0) {
        processed_generation[slot] = value.generation;
    }
    k_spin_unlock(&rx_lock, rx_key);
    if (value.generation == 0) {
        return -ENODATA;
    }
    memcpy(snapshot.data, value.data, sizeof(snapshot.data));
    snapshot.length = value.length;
    snapshot.timestamp_ms = value.timestamp_ms;
    return 0;
}

/**
 * @brief 绑定一台 C610 电机（经典 CAN，标准帧）
 *
 * @param motor C610 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_bind(c610 &motor)
{
    return bind_motor(motor_kind::c610, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, motor.motor_id());
}

/**
 * @brief 绑定一台 C620 电机（经典 CAN，标准帧）
 *
 * @param motor C620 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_bind(c620 &motor)
{
    return bind_motor(motor_kind::c620, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, motor.motor_id());
}

/**
 * @brief 绑定一台 DM 电机
 *
 * @param motor               DM 电机对象引用
 * @param automatic_recovery 是否启用离线自动恢复
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_bind(dm_motor &motor, bool automatic_recovery)
{
    return bind_motor(motor_kind::dm, &motor, motor.device(), motor.feedback_id(), motor.feedback_protocol(), motor.motor_id(), automatic_recovery);
}

/**
 * @brief 绑定一台 Cubemars 电机（经典 CAN，标准帧）
 *
 * @param motor Cubemars 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_bind(cubemars &motor)
{
    return bind_motor(motor_kind::cubemars, &motor, motor.device(), motor.feedback_id(), FDCAN_PROTOCOL_CLASSIC, 0);
}

/**
 * @brief 查询指定下标电机的健康状态数据
 *
 * @param index 电机表下标
 * @param data  输出健康状态数据
 * @return 成功返回 0；下标越界返回 -EINVAL，未绑定返回 -ENOENT
*/
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
            data.status = 0;
            break;
        }
        case motor_kind::c620: {
            const auto &motor = *static_cast<c620 *>(entry.motor);
            data.kind = FdcanMotorKind::c620;
            data.valid_feedback_count = motor.get_feedback_count();
            data.control_frame_id = motor.command_frame_id();
            data.status = 0;
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

/**
 * @brief 获取当前已绑定的电机数量
 *
 * @return 已绑定的电机数量
*/
size_t fdcan_port_motor_count()
{
    const k_spinlock_key_t key = k_spin_lock(&motor_lock);
    const size_t count = motor_count;
    k_spin_unlock(&motor_lock, key);
    return count;
}

/**
 * @brief 设置某电机为离线/在线状态，并清理其周期控制帧、重置使能确认
 *
 * @param index   电机表下标
 * @param offline 为 true 表示设置离线，为 false 表示恢复在线
*/
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
    const uint32_t control_id = kind == motor_kind::cubemars ? static_cast<cubemars *>(motors[index].motor)->control_frame_id() : 0;
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

/**
 * @brief 构建电机的离线帧（即不带清错标志的恢复帧）
 *
 * @param index 电机表下标
 * @param frame 输出离线帧
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_build_offline_frame(size_t index, fdcan_frame &frame)
{
    return fdcan_port_build_recovery_frame(index, false, frame);
}

/**
 * @brief 构建电机的恢复帧（使能帧，或按需生成清错帧）
 *
 * @param index       电机表下标
 * @param clear_error 是否生成清除错误帧
 * @param frame       输出恢复帧
 * @return 成功返回 0；C610/C620 不支持恢复返回 -ENOTSUP，否则返回对应错误码
*/
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

/**
 * @brief 构建指定电机的控制帧
 *
 * @param index 电机表下标
 * @param frame 输出控制帧
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 将一帧控制数据提交到指定总线的周期发送队列
 *
 * @param bus   目标总线
 * @param frame 待提交的控制帧
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 构建并提交 C610 电机的控制帧到其所在总线
 *
 * @param motor C610 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 构建并提交 C620 电机的控制帧到其所在总线
 *
 * @param motor C620 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 构建并提交 DM 电机的控制帧到其所在总线
 *
 * @param motor DM 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 构建并提交 Cubemars 电机的控制帧到其所在总线
 *
 * @param motor Cubemars 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
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

/**
 * @brief 通过管理帧通道立即发送一帧（不走周期发送队列）
 *
 * @param bus   目标总线
 * @param frame 待发送的帧
 * @return 成功返回 0，否则返回对应负错误码
*/
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
    admin_entry *slot = find_admin_slot(bus, frame, 0);
    if (slot == nullptr) {
        k_spin_unlock(&tx_lock, key);
        return -ENOBUFS;
    }
    if (slot->pending && slot->frame.length == frame.length &&
        slot->frame.bitrate_switch == frame.bitrate_switch &&
        memcmp(slot->frame.data, frame.data, frame.length) == 0) {
        k_spin_unlock(&tx_lock, key);
        return 0;
    }
    stage_admin(*slot, bus, frame, 0);
    k_spin_unlock(&tx_lock, key);
    atomic_set(&tx_due, 1);
    k_sem_give(&port_sem);
    return 0;
}

/**
 * @brief 请求 DM 电机切换到指定控制模式，并停发旧模式的周期控制帧
 *
 * @param motor DM 电机对象引用
 * @param mode  目标控制模式
 * @return 成功返回 0，否则返回对应负错误码
*/
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
    admin_entry *slot = find_admin_slot(bus, frame, motor.motor_id());
    if (slot == nullptr) {
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
    stage_admin(*slot, bus, frame, motor.motor_id());
    k_spin_unlock(&tx_lock, key);
    atomic_set(&tx_due, 1);
    k_sem_give(&port_sem);
    return 0;
}

/**
 * @brief 通过管理帧通道使能 Cubemars 电机
 *
 * @param motor Cubemars 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_enable(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_enable_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

/**
 * @brief 通过管理帧通道失能 Cubemars 电机
 *
 * @param motor Cubemars 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_disable(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_disable_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

/**
 * @brief 通过管理帧通道让 Cubemars 电机保存当前零位
 *
 * @param motor Cubemars 电机对象引用
 * @return 成功返回 0，否则返回对应负错误码
*/
int fdcan_port_save_zero(const cubemars &motor)
{
    fdcan_frame frame{};
    const int ret = motor.build_save_zero_frame(frame);
    return ret == 0 ? fdcan_port_send_once(motor.device(), frame) : ret;
}

/**
 * @brief 查询 C610 电机累计收到的反馈帧数量
 *
 * @param motor C610 电机对象引用
 * @return 累计反馈帧数量
*/
uint32_t fdcan_port_received_count(const c610 &motor)
{
    return received_count(&motor);
}

/**
 * @brief 查询 C620 电机累计收到的反馈帧数量
 *
 * @param motor C620 电机对象引用
 * @return 累计反馈帧数量
*/
uint32_t fdcan_port_received_count(const c620 &motor)
{
    return received_count(&motor);
}

/**
 * @brief 查询 DM 电机累计收到的反馈帧数量
 *
 * @param motor DM 电机对象引用
 * @return 累计反馈帧数量
*/
uint32_t fdcan_port_received_count(const dm_motor &motor)
{
    return received_count(&motor);
}

/**
 * @brief 查询 Cubemars 电机累计收到的反馈帧数量
 *
 * @param motor Cubemars 电机对象引用
 * @return 累计反馈帧数量
*/
uint32_t fdcan_port_received_count(const cubemars &motor)
{
    return received_count(&motor);
}

/**
 * @brief 获取接收过程中因路由缺失或覆盖而丢弃的帧总数
 *
 * @return 丢弃帧数量
*/
uint32_t fdcan_port_rx_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&rx_dropped));
}

/**
 * @brief 获取累计发生的发送错误次数
 *
 * @return 发送错误次数
*/
uint32_t fdcan_port_tx_error_count()
{
    return static_cast<uint32_t>(atomic_get(&tx_errors));
}

/**
 * @brief 获取因提交失败或阻塞而被丢弃的控制帧数量
 *
 * @return 丢弃的控制帧数量
*/
uint32_t fdcan_port_control_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&control_send_dropped));
}