#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

// 电机类型枚举
enum class fdcan_motor_kind : uint8_t
{
    c610,
    c620,
    dm,
    cubemars,
};

constexpr size_t FDCAN_CONTROL_SLOT_COUNT = 32;

// 电机反馈主题数据
struct fdcan_feedback_topic_data
{
    fdcan_device bus;
    uint32_t id;
    fdcan_motor_kind kind;
    float speed_rad_s;
    float angle_rad;
    float current_a;
    float torque_nm;
    uint32_t valid_count;
    uint32_t timestamp_ms;
};

// 电机查询键
struct fdcan_feedback_key
{
    fdcan_device bus;
    uint32_t id;
    fdcan_motor_kind kind;
};

// 电机控制指令主题数据
struct fdcan_control_topic_data
{
    fdcan_device bus;
    fdcan_frame frame;
};

using fdcan_control_sink_t = int (*)(const fdcan_control_topic_data &);

using fdcan_feedback_refresh_t = void (*)(fdcan_device, uint32_t, fdcan_motor_kind);

using fdcan_feedback_refresh_batch_t = void (*)(const fdcan_feedback_key *, size_t);

void fdcan_topic_init();

void fdcan_topic_publish_feedback(const fdcan_feedback_topic_data &data);

int fdcan_topic_latest_feedback(fdcan_device bus, uint32_t id, fdcan_motor_kind kind, fdcan_feedback_topic_data &data);

void fdcan_topic_latest_feedback_batch(const fdcan_feedback_key *keys, size_t count, fdcan_feedback_topic_data *data, bool *found);

int fdcan_topic_publish_control(const fdcan_control_topic_data &data);

void fdcan_topic_set_control_sink(fdcan_control_sink_t sink);

void fdcan_topic_set_feedback_refresh(fdcan_feedback_refresh_t refresh);

void fdcan_topic_set_feedback_refresh_batch(fdcan_feedback_refresh_batch_t refresh);

uint32_t fdcan_topic_feedback_dropped_count();

uint32_t fdcan_topic_control_dropped_count();

