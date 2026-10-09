#include "fdcan_channel.hpp"

namespace
{
    fdcan_control_sink_t control_sink = nullptr;
    fdcan_feedback_refresh_t feedback_refresh = nullptr;
    fdcan_feedback_refresh_batch_t feedback_refresh_batch = nullptr;

    atomic_t feedback_dropped{};

    atomic_t control_dropped{};

    struct latest_entry
    {
        bool used;
        FdcanFeedbackTopicData feedback;
    };

    latest_entry latest[32]{};
    struct k_spinlock latest_lock;

}

void fdcan_topic_init()
{
    control_sink = nullptr;
}

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

int fdcan_topic_publish_control(const FdcanControlTopicData &data)
{
    const int result = control_sink != nullptr ? control_sink(data) : -ENODEV;
    if (result != 0) {
        atomic_inc(&control_dropped);
    }
    return result;
}

void fdcan_topic_set_control_sink(fdcan_control_sink_t sink)
{
    control_sink = sink;
}

void fdcan_topic_set_feedback_refresh(fdcan_feedback_refresh_t refresh)
{
    feedback_refresh = refresh;
}

void fdcan_topic_set_feedback_refresh_batch(fdcan_feedback_refresh_batch_t refresh)
{
    feedback_refresh_batch = refresh;
}

uint32_t fdcan_topic_feedback_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&feedback_dropped));
}

uint32_t fdcan_topic_control_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&control_dropped));
}
