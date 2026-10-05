#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

enum class FdcanMotorKind : uint8_t
{
    c610,
    c620,
    dm,
};

struct FdcanFeedbackTopicData
{
    fdcan_device bus;
    uint32_t id;
    FdcanMotorKind kind;
    float speed_rad_s;
    float angle_rad;
    float current_a;
    float torque_nm;
    uint32_t valid_count;
};

struct FdcanControlTopicData
{
    fdcan_device bus;
    fdcan_frame frame;
};

using fdcan_topic_notify_t = void (*)();

void fdcan_topic_init();

int fdcan_topic_publish_feedback(const FdcanFeedbackTopicData &data);

int fdcan_topic_receive_feedback(FdcanFeedbackTopicData &data);

int fdcan_topic_publish_control(const FdcanControlTopicData &data);

int fdcan_topic_receive_control(FdcanControlTopicData &data);

void fdcan_topic_set_control_notify(fdcan_topic_notify_t notify);

uint32_t fdcan_topic_feedback_dropped_count();

uint32_t fdcan_topic_control_dropped_count();
