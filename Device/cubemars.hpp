#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_fdcan.hpp"

struct CubemarsData
{
    float now_angle;
    float now_total_angle;
    float now_omega;
    float now_torque;
};

class cubemars
{
    public:
    int init(fdcan_device device, float angle_max = 12.5F, float omega_max = 50.0F, float torque_max = 65.0F);

    int set_mit(float position, float velocity, float kp, float kd, float torque);

    int build_control_frame(fdcan_frame &frame) const;

    int build_enable_frame(fdcan_frame &frame) const;

    int build_disable_frame(fdcan_frame &frame) const;

    int build_save_zero_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    CubemarsData get_data() const;

    uint32_t get_feedback_count() const;

    fdcan_device device() const { return device_; }

    uint32_t feedback_id() const { return 0x00U; }

    uint32_t control_frame_id() const { return 0x01U; }

    static void pack_mit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max);

    private:
    static constexpr float kp_max_ = 500.0F;
    static constexpr float kd_max_ = 5.0F;

    static uint16_t encode(float value, float minimum, float maximum, uint8_t bits);

    static float decode(uint16_t raw, float limit, uint8_t bits);

    static float wrap_delta(float delta, float angle_max);

    int build_command_frame(uint8_t tail, fdcan_frame &frame) const;

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    float angle_max_ = 12.5F;

    float omega_max_ = 50.0F;

    float torque_max_ = 65.0F;

    float target_angle_ = 0.0F;

    float target_omega_ = 0.0F;

    float target_torque_ = 0.0F;

    float target_kp_ = 0.0F;

    float target_kd_ = 0.0F;

    bool initialized_ = false;

    bool raw_angle_initialized_ = false;

    float last_raw_angle_ = 0.0F;

    CubemarsData data_{};

    mutable struct k_spinlock lock_{};

    atomic_t feedback_count_{};
};

using MotorCubemarsMit = cubemars;
using MotorCubemars = cubemars;
using MotorCubeMars = cubemars;
