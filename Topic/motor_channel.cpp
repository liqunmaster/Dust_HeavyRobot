#include "motor_channel.hpp"

namespace
{
    MotorTopic motor_topic;
}

void MotorTopic::Init()
{
    control_sink_ = nullptr;
}

void MotorTopic::PublishFeedback(const MotorFeedbackTopicData &data)
{
    const k_spinlock_key_t key = k_spin_lock(&latest_lock_);
    LatestEntry *slot = nullptr;
    for (LatestEntry &entry : latest_) {
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
        atomic_inc(&feedback_dropped_);
    }
    k_spin_unlock(&latest_lock_, key);
}

int MotorTopic::LatestFeedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackTopicData &data)
{
    if (feedback_refresh_ != nullptr) {
        feedback_refresh_(bus, id, kind);
    }
    const k_spinlock_key_t key = k_spin_lock(&latest_lock_);
    for (const LatestEntry &entry : latest_) {
        if (entry.used && entry.feedback.bus == bus && entry.feedback.id == id &&
            entry.feedback.kind == kind) {
            data = entry.feedback;
            k_spin_unlock(&latest_lock_, key);
            return 0;
        }
    }
    k_spin_unlock(&latest_lock_, key);
    return -ENODATA;
}

void MotorTopic::LatestFeedbackBatch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackTopicData *data, bool *found)
{
    if (keys == nullptr || data == nullptr || found == nullptr) {
        return;
    }
    if (feedback_refresh_batch_ != nullptr) {
        feedback_refresh_batch_(keys, count);
    } else if (feedback_refresh_ != nullptr) {
        for (size_t index = 0; index < count; ++index) {
            feedback_refresh_(keys[index].bus, keys[index].id, keys[index].kind);
        }
    }
    const k_spinlock_key_t key = k_spin_lock(&latest_lock_);
    for (size_t index = 0; index < count; ++index) {
        found[index] = false;
        for (const LatestEntry &entry : latest_) {
            if (entry.used && entry.feedback.bus == keys[index].bus &&
                entry.feedback.id == keys[index].id && entry.feedback.kind == keys[index].kind) {
                data[index] = entry.feedback;
                found[index] = true;
                break;
            }
        }
    }
    k_spin_unlock(&latest_lock_, key);
}

int MotorTopic::PublishControl(const MotorControlTopicData &data)
{
    const int result = control_sink_ != nullptr ? control_sink_(data) : -ENODEV;
    if (result != 0) {
        atomic_inc(&control_dropped_);
    }
    return result;
}

void MotorTopic::SetControlSink(MotorControlSink sink)
{
    control_sink_ = sink;
}

void MotorTopic::SetFeedbackRefresh(MotorFeedbackRefresh refresh)
{
    feedback_refresh_ = refresh;
}

void MotorTopic::SetFeedbackRefreshBatch(MotorFeedbackRefreshBatch refresh)
{
    feedback_refresh_batch_ = refresh;
}

uint32_t MotorTopic::FeedbackDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&feedback_dropped_));
}

uint32_t MotorTopic::ControlDroppedCount()
{
    return static_cast<uint32_t>(atomic_get(&control_dropped_));
}

void motor_topic_init()
{
    motor_topic.Init();
}

void motor_topic_publish_feedback(const MotorFeedbackTopicData &data)
{
    motor_topic.PublishFeedback(data);
}

int motor_topic_latest_feedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackTopicData &data)
{
    return motor_topic.LatestFeedback(bus, id, kind, data);
}

void motor_topic_latest_feedback_batch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackTopicData *data, bool *found)
{
    motor_topic.LatestFeedbackBatch(keys, count, data, found);
}

int motor_topic_publish_control(const MotorControlTopicData &data)
{
    return motor_topic.PublishControl(data);
}

void motor_topic_set_control_sink(MotorControlSink sink)
{
    motor_topic.SetControlSink(sink);
}

void motor_topic_set_feedback_refresh(MotorFeedbackRefresh refresh)
{
    motor_topic.SetFeedbackRefresh(refresh);
}

void motor_topic_set_feedback_refresh_batch(MotorFeedbackRefreshBatch refresh)
{
    motor_topic.SetFeedbackRefreshBatch(refresh);
}

uint32_t motor_topic_feedback_dropped_count()
{
    return motor_topic.FeedbackDroppedCount();
}

uint32_t motor_topic_control_dropped_count()
{
    return motor_topic.ControlDroppedCount();
}
