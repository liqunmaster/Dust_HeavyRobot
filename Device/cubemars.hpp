#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

// CubeMars 电机反馈数据，解算后的角度/角速度/力矩
struct CubemarsData
{
    float now_angle;
    float now_total_angle;
    float now_omega;
    float now_torque;
};

// CubeMars 电机控制类，支持 MIT 模式指令打包与反馈解算
class cubemars
{
    public:
    int init(fdcan_device device, float angle_max = 12.5f, float omega_max = 50.0f, float torque_max = 65.0f);

    int set_mit(float position, float velocity, float kp, float kd, float torque);

    int build_control_frame(fdcan_frame &frame) const;

    int build_enable_frame(fdcan_frame &frame) const;

    int build_disable_frame(fdcan_frame &frame) const;

    int build_save_zero_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    CubemarsData get_data() const;

    uint32_t get_feedback_count() const;

    /**
     * @brief 获取电机绑定的 FDCAN 通道
     *
     * @return FDCAN 通道枚举值
    */
    fdcan_device device() const { return device_; }

    /**
     * @brief 获取反馈帧的标准 ID
     *
     * @return 固定的反馈 ID 0x00
    */
    uint32_t feedback_id() const { return 0x00; }

    /**
     * @brief 获取控制帧的标准 ID
     *
     * @return 固定的控制 ID 0x01
    */
    uint32_t control_frame_id() const { return 0x01; }

    static void pack_mit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max);

    private:
    static constexpr float kp_max_ = 500.0f;
    static constexpr float kd_max_ = 5.0f;

    static uint16_t encode(float value, float minimum, float maximum, uint8_t bits);

    static float decode(uint16_t raw, float limit, uint8_t bits);

    static float wrap_delta(float delta, float angle_max);

    int build_command_frame(uint8_t tail, fdcan_frame &frame) const;

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    float angle_max_ = 12.5f;

    float omega_max_ = 50.0f;

    float torque_max_ = 65.0f;

    float target_angle_ = 0.0f;

    float target_omega_ = 0.0f;

    float target_torque_ = 0.0f;

    float target_kp_ = 0.0f;

    float target_kd_ = 0.0f;

    bool initialized_ = false;

    bool raw_angle_initialized_ = false;

    float last_raw_angle_ = 0.0f;

    CubemarsData data_{};

    mutable struct k_spinlock lock_{};

    atomic_t feedback_count_{};
};

using MotorCubemarsMit = cubemars;
using MotorCubemars = cubemars;
using MotorCubeMars = cubemars;