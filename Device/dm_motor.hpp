#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "bsp_fdcan.hpp"

// 达妙电机控制模式枚举
enum dm_control_mode
{
    MOTOR_DM_CONTROL_METHOD_NORMAL_MIT = 1,
    MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA = 2,
    MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA = 3,
    MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT = 4,
};

// 达妙电机 MIT 模式的量程限制结构体
struct dm_mit_limits
{
    float position;
    float velocity;
    float torque;
};

// 达妙电机原始反馈数据解包结构体
struct dm_rx_data
{
    uint8_t motor_id;
    uint8_t status;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
};

// 达妙电机处理后的数据结构体
struct dm_data
{
    float now_angle;
    float now_omega;
    float now_torque;
    float now_mos_temperature;
    float now_rotor_temperature;
    uint8_t status;
};

// 达妙系列无刷电机驱动类
class dm_motor
{
    public:
    int init(fdcan_device device, uint8_t motor_id, dm_control_mode mode, dm_mit_limits limits,
    fdcan_protocol protocol, uint16_t master_id = 0, uint16_t tx_id_base = 0);

    int set_mit(float position, float velocity, float kp, float kd, float torque);

    int set_position_velocity(float position, float max_velocity);

    int set_velocity(float velocity);

    int set_position_torque(float position, float max_velocity, float current_ratio);

    int build_control_frame(fdcan_frame &frame) const;

    int build_enable_frame(fdcan_frame &frame) const;

    int build_disable_frame(fdcan_frame &frame) const;

    int build_clear_error_frame(fdcan_frame &frame) const;

    int build_save_zero_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    uint32_t get_feedback_count() const;

    dm_rx_data get_rx_data() const;

    dm_data get_data() const;

    dm_control_mode get_mode() const;

    /**
     * @brief 获取绑定的 CAN 通道编号
     *
     * @return 通道编号
     */
    fdcan_device device() const { return device_; }

    /**
     * @brief 获取电机 ID
     *
     * @return 电机 ID
     */
    uint8_t motor_id() const { return motor_id_; }

    /**
     * @brief 获取反馈报文 ID
     *
     * @return 反馈报文 ID
     */
    uint32_t feedback_id() const { return master_id_; }

    /**
     * @brief 获取反馈报文使用的协议类型
     *
     * @return 协议类型
     */
    fdcan_protocol feedback_protocol() const { return protocol_; }

    /**
     * @brief 获取控制报文 ID
     *
     * @return 控制报文 ID
     */
    uint32_t control_frame_id() const;

    private:
    friend int fdcan_port_request_mode(dm_motor &motor, dm_control_mode mode);

    int build_mode_frame(dm_control_mode mode, fdcan_frame &frame) const;

    int mark_mode_request(dm_control_mode mode);

    static bool valid_mode(dm_control_mode mode);

    static uint16_t control_id(uint16_t tx_id_base, dm_control_mode mode);

    static uint16_t encode(float value, float minimum, float maximum, uint16_t maximum_raw);

    static float decode(uint16_t raw, float limit, uint16_t maximum_raw);

    static void write_f32(uint8_t *dst, float value);

    static void write_u16(uint8_t *dst, uint16_t value);

    int build_command_frame(uint8_t command, fdcan_frame &frame) const;

    static int make_frame(uint16_t id, const uint8_t *data, uint8_t length, fdcan_frame &frame);

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    uint8_t motor_id_ = 0;

    uint16_t master_id_ = 0;

    uint16_t tx_id_base_ = 0;

    dm_control_mode mode_ = dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    fdcan_protocol protocol_ = FDCAN_PROTOCOL_CLASSIC;

    dm_control_mode requested_mode_ = dm_control_mode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    bool mode_change_pending_ = false;

    bool initialized_ = false;

    dm_mit_limits limits_{};

    uint8_t tx_data_[8]{};

    uint8_t tx_length_ = 0;

    dm_rx_data rx_data_{};

    dm_data data_{};

    mutable struct k_spinlock lock_{};

    atomic_t feedback_count_{};
};
