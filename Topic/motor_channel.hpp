#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "bsp_canfd.hpp"

enum class MotorChannelKind : uint8_t
{
    c610,
    c620,
    dm,
    cubemars,
};

constexpr size_t MOTOR_CONTROL_SLOT_COUNT = 32;

struct MotorFeedbackTopicData
{
    CanfdDevice bus;
    uint32_t id;
    MotorChannelKind kind;
    float speed_rad_s;
    float angle_rad;
    float current_a;
    float torque_nm;
    uint32_t valid_count;
    uint32_t timestamp_ms;
};

struct MotorFeedbackKey
{
    CanfdDevice bus;
    uint32_t id;
    MotorChannelKind kind;
};

struct MotorControlTopicData
{
    CanfdDevice bus;
    CanfdFrame frame;
};

using MotorControlSink = int (*)(const MotorControlTopicData &);

using MotorFeedbackRefresh = void (*)(CanfdDevice, uint32_t, MotorChannelKind);

using MotorFeedbackRefreshBatch = void (*)(const MotorFeedbackKey *, size_t);

class MotorTopic
{
    public:
    void Init();

    void PublishFeedback(const MotorFeedbackTopicData &data);

    int LatestFeedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackTopicData &data);

    void LatestFeedbackBatch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackTopicData *data, bool *found);

    int PublishControl(const MotorControlTopicData &data);

    void SetControlSink(MotorControlSink sink);

    void SetFeedbackRefresh(MotorFeedbackRefresh refresh);

    void SetFeedbackRefreshBatch(MotorFeedbackRefreshBatch refresh);

    uint32_t FeedbackDroppedCount();

    uint32_t ControlDroppedCount();

    private:
    struct LatestEntry
    {
        bool used;
        MotorFeedbackTopicData feedback;
    };

    MotorControlSink control_sink_ = nullptr;
    MotorFeedbackRefresh feedback_refresh_ = nullptr;
    MotorFeedbackRefreshBatch feedback_refresh_batch_ = nullptr;
    atomic_t feedback_dropped_{};
    atomic_t control_dropped_{};
    LatestEntry latest_[32]{};
    struct k_spinlock latest_lock_{};
};

void motor_topic_init();

void motor_topic_publish_feedback(const MotorFeedbackTopicData &data);

int motor_topic_latest_feedback(CanfdDevice bus, uint32_t id, MotorChannelKind kind, MotorFeedbackTopicData &data);

void motor_topic_latest_feedback_batch(const MotorFeedbackKey *keys, size_t count, MotorFeedbackTopicData *data, bool *found);

int motor_topic_publish_control(const MotorControlTopicData &data);

void motor_topic_set_control_sink(MotorControlSink sink);

void motor_topic_set_feedback_refresh(MotorFeedbackRefresh refresh);

void motor_topic_set_feedback_refresh_batch(MotorFeedbackRefreshBatch refresh);

uint32_t motor_topic_feedback_dropped_count();

uint32_t motor_topic_control_dropped_count();
