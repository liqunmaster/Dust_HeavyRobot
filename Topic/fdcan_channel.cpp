#include "fdcan_channel.hpp"

namespace
{
    fdcan_control_sink_t control_sink = nullptr;
    fdcan_feedback_refresh_t feedback_refresh = nullptr;
    fdcan_feedback_refresh_batch_t feedback_refresh_batch = nullptr;

    atomic_t feedback_dropped{};

    atomic_t control_dropped{};

    // 缓存目标电机最新反馈的槽位项
    struct latest_entry
    {
        bool used;
        FdcanFeedbackTopicData feedback;
    };

    latest_entry latest[32]{};
    struct k_spinlock latest_lock;

}

    /**
     * @brief 初始化 FDCAN 主题通道
    */
    void fdcan_topic_init()
    {
    control_sink = nullptr;
}

    /**
     * @brief 发布一帧电机反馈并缓存到对应槽位
     *
     * @param data 电机反馈数据
    */
    void fdcan_topic_publish_feedback(const FdcanFeedbackTopicData &data)
    {
    const k_spinlock_key_t key = k_spin_lock(&latest_lock);
    latest_entry *slot = nullptr;
    for (latest_entry &entry : latest) {
        if (entry.used && entry.feedback.bus == data.bus && entry.feedback.id == data.id &&
            entry.feedback.kind == data.kind) {
            slot = &entry;
            break;
        }
        if (!entry.used && slot == nullptr) {
            slot = &entry;
        }
    }
    if (slot != nullptr) {
        slot->used = true;
        slot->feedback = data;
    } else {
        atomic_inc(&feedback_dropped);
    }
    k_spin_unlock(&latest_lock, key);
}

    /**
     * @brief 按总线/ID/类型读取指定电机的最新反馈
     *
     * @param bus 总线标识
     * @param id 电机 ID
     * @param kind 电机类型
     * @param data 数据输出参数
     * @return 成功返回 0，未找到返回 -ENODATA
    */
    int fdcan_topic_latest_feedback(fdcan_device bus, uint32_t id, FdcanMotorKind kind, FdcanFeedbackTopicData &data)
    {
    if (feedback_refresh != nullptr) {
        feedback_refresh(bus, id, kind);
    }
    const k_spinlock_key_t key = k_spin_lock(&latest_lock);
    for (const latest_entry &entry : latest) {
        if (entry.used && entry.feedback.bus == bus && entry.feedback.id == id &&
            entry.feedback.kind == kind) {
            data = entry.feedback;
            k_spin_unlock(&latest_lock, key);
            return 0;
        }
    }
    k_spin_unlock(&latest_lock, key);
    return -ENODATA;
}

    /**
     * @brief 批量读取多台电机的最新反馈
     *
     * @param keys 查询键数组
     * @param count 查询数量
     * @param data 反馈输出数组
     * @param found 各查询是否找到的标识数组
    */
    void fdcan_topic_latest_feedback_batch(const FdcanFeedbackKey *keys, size_t count, FdcanFeedbackTopicData *data, bool *found)
    {
    if (keys == nullptr || data == nullptr || found == nullptr) {
        return;
    }
    if (feedback_refresh_batch != nullptr) {
        feedback_refresh_batch(keys, count);
    } else if (feedback_refresh != nullptr) {
        for (size_t index = 0; index < count; ++index) {
            feedback_refresh(keys[index].bus, keys[index].id, keys[index].kind);
        }
    }
    const k_spinlock_key_t key = k_spin_lock(&latest_lock);
    for (size_t index = 0; index < count; ++index) {
        found[index] = false;
        for (const latest_entry &entry : latest) {
            if (entry.used && entry.feedback.bus == keys[index].bus &&
                entry.feedback.id == keys[index].id && entry.feedback.kind == keys[index].kind) {
                data[index] = entry.feedback;
                found[index] = true;
                break;
            }
        }
    }
    k_spin_unlock(&latest_lock, key);
}

    /**
     * @brief 发布一帧控制指令给底层发送回调
     *
     * @param data 控制指令数据
     * @return 发送结果，无回调时返回 -ENODEV
    */
    int fdcan_topic_publish_control(const FdcanControlTopicData &data)
    {
    const int result = control_sink != nullptr ? control_sink(data) : -ENODEV;
    if (result != 0) {
        atomic_inc(&control_dropped);
    }
    return result;
}

    /**
     * @brief 注册控制指令发送回调
     *
     * @param sink 发送回调函数
    */
    void fdcan_topic_set_control_sink(fdcan_control_sink_t sink)
    {
    control_sink = sink;
}

    /**
     * @brief 注册单台反馈刷新回调
     *
     * @param refresh 刷新回调函数
    */
    void fdcan_topic_set_feedback_refresh(fdcan_feedback_refresh_t refresh)
    {
    feedback_refresh = refresh;
}

    /**
     * @brief 注册批量反馈刷新回调
     *
     * @param refresh 批量刷新回调函数
    */
    void fdcan_topic_set_feedback_refresh_batch(fdcan_feedback_refresh_batch_t refresh)
    {
    feedback_refresh_batch = refresh;
}

    /**
     * @brief 获取因槽位满而丢弃的反馈帧计数
     *
     * @return 丢弃计数
    */
    uint32_t fdcan_topic_feedback_dropped_count()
    {
    return static_cast<uint32_t>(atomic_get(&feedback_dropped));
}

    /**
     * @brief 获取因发送失败而丢弃的控制指令计数
     *
     * @return 丢弃计数
    */
    uint32_t fdcan_topic_control_dropped_count()
    {
    return static_cast<uint32_t>(atomic_get(&control_dropped));
}