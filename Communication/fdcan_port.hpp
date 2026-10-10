#pragma once

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"
#include "cubemars.hpp"
#include "dji_c610.hpp"
#include "dji_c620.hpp"
#include "dm_motor.hpp"
#include "fdcan_channel.hpp"

class c610;
class c620;

constexpr size_t FDCAN_PORT_MAX_MOTORS = 32;

// 一次原始 CAN 帧的快照 保存数据字节、帧长与时间戳
struct fdcan_raw_snapshot
{
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp_ms;
};

// 单台电机在健康监控视角下的状态数据 用于判断在线/离线与自动恢复
struct fdcan_motor_health_data
{
    fdcan_motor_kind kind;

    fdcan_device bus;

    uint32_t feedback_id;

    uint32_t valid_feedback_count;

    uint32_t control_frame_id;

    uint8_t status;

    uint32_t request_count;

    uint32_t response_count;

    uint32_t pending_since_ms;

    bool enable_acknowledged;

    bool automatic_recovery;
};

// Classic CAN, standard ID, payload up to eight bytes. Register during startup.
int fdcan_port_subscribe_raw(fdcan_device bus, uint32_t id);

int fdcan_port_latest_raw(fdcan_device bus, uint32_t id, fdcan_raw_snapshot &snapshot);

int fdcan_port_init();

int fdcan_port_bind(c610 &motor);

int fdcan_port_bind(c620 &motor);

int fdcan_port_bind(dm_motor &motor, bool automatic_recovery = true);

int fdcan_port_bind(cubemars &motor);

int fdcan_port_motor_health(size_t index, fdcan_motor_health_data &data);

size_t fdcan_port_motor_count();

void fdcan_port_set_motor_offline(size_t index, bool offline);

int fdcan_port_build_offline_frame(size_t index, fdcan_frame &frame);

int fdcan_port_build_recovery_frame(size_t index, bool clear_error, fdcan_frame &frame);

int fdcan_port_build_control_frame(size_t index, fdcan_frame &frame);

int fdcan_port_submit(fdcan_device device, const fdcan_frame &frame);

int fdcan_port_submit(const c610 &motor);

int fdcan_port_submit(const c620 &motor);

int fdcan_port_submit(const dm_motor &motor);

int fdcan_port_submit(const cubemars &motor);

int fdcan_port_send_once(fdcan_device device, const fdcan_frame &frame);

int fdcan_port_request_mode(dm_motor &motor, dm_control_mode mode);

int fdcan_port_enable(const cubemars &motor);

int fdcan_port_disable(const cubemars &motor);

int fdcan_port_save_zero(const cubemars &motor);

uint32_t fdcan_port_received_count(const c610 &motor);

uint32_t fdcan_port_received_count(const c620 &motor);

uint32_t fdcan_port_received_count(const dm_motor &motor);

uint32_t fdcan_port_received_count(const cubemars &motor);

uint32_t fdcan_port_rx_dropped_count();

uint32_t fdcan_port_tx_error_count();

uint32_t fdcan_port_control_dropped_count();
