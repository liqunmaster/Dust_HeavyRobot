#include "fdcan_channel.hpp"

namespace
{
    alignas(4) char feedback_storage[sizeof(FdcanFeedbackTopicData) * 32U];

    alignas(4) char control_storage[sizeof(FdcanControlTopicData) * 8U];

    struct k_msgq feedback_queue{};

    struct k_msgq control_queue{};

    fdcan_topic_notify_t control_notify = nullptr;

    atomic_t feedback_dropped{};

    atomic_t control_dropped{};

}

void fdcan_topic_init()
{
    k_msgq_init(&feedback_queue, feedback_storage, sizeof(FdcanFeedbackTopicData), 32U);
    k_msgq_init(&control_queue, control_storage, sizeof(FdcanControlTopicData), 8U);
}

int fdcan_topic_publish_feedback(const FdcanFeedbackTopicData &data)
{
    const int ret = k_msgq_put(&feedback_queue, &data, K_NO_WAIT);
    if (ret != 0) {
        atomic_inc(&feedback_dropped);
    }
    return ret;
}

int fdcan_topic_receive_feedback(FdcanFeedbackTopicData &data)
{
    return k_msgq_get(&feedback_queue, &data, K_NO_WAIT);
}

int fdcan_topic_publish_control(const FdcanControlTopicData &data)
{
    const int ret = k_msgq_put(&control_queue, &data, K_NO_WAIT);
    if (ret != 0) {
        atomic_inc(&control_dropped);
        return ret;
    }
    if (control_notify != nullptr) {
        control_notify();
    }
    return 0;
}

int fdcan_topic_receive_control(FdcanControlTopicData &data)
{
    return k_msgq_get(&control_queue, &data, K_NO_WAIT);
}

void fdcan_topic_set_control_notify(fdcan_topic_notify_t notify)
{
    control_notify = notify;
}

uint32_t fdcan_topic_feedback_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&feedback_dropped));
}

uint32_t fdcan_topic_control_dropped_count()
{
    return static_cast<uint32_t>(atomic_get(&control_dropped));
}
