#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

// 电机类型枚举
enum class FdcanMotorKind : uint8_t
{
    c610,
    c620,
    dm,
    cubemars,
};

constexpr size_t FDCAN_CONTROL_SLOT_COUNT = 32;

// 电机反馈主题数据
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
    uint32_t timestamp_ms;
};

// 电机查询键（总线/ID/类型）
struct FdcanFeedbackKey
{
    fdcan_device bus;
    uint32_t id;
    FdcanMotorKind kind;
};

// 电机控制指令主题数据
struct FdcanControlTopicData
{
    fdcan_device bus;
    fdcan_frame frame;
};

using fdcan_control_sink_t = int (*)(const FdcanControlTopicData &);

using fdcan_feedback_refresh_t = void (*)(fdcan_device, uint32_t, FdcanMotorKind);

using fdcan_feedback_refresh_batch_t = void (*)(const FdcanFeedbackKey *, size_t);

void fdcan_topic_init();

void fdcan_topic_publish_feedback(const FdcanFeedbackTopicData &data);

int fdcan_topic_latest_feedback(fdcan_device bus, uint32_t id, FdcanMotorKind kind, FdcanFeedbackTopicData &data);

void fdcan_topic_latest_feedback_batch(const FdcanFeedbackKey *keys, size_t count, FdcanFeedbackTopicData *data, bool *found);

int fdcan_topic_publish_control(const FdcanControlTopicData &data);

void fdcan_topic_set_control_sink(fdcan_control_sink_t sink);

void fdcan_topic_set_feedback_refresh(fdcan_feedback_refresh_t refresh);

void fdcan_topic_set_feedback_refresh_batch(fdcan_feedback_refresh_batch_t refresh);

uint32_t fdcan_topic_feedback_dropped_count();

uint32_t fdcan_topic_control_dropped_count();
