#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bsp_canfd.hpp"

struct CubemarsData
{
    float now_angle;
    float now_total_angle;
    float now_omega;
    float now_torque;
};

class Cubemars
{
    public:
    int Init(CanfdDevice device, float angle_max = 12.5f, float omega_max = 50.0f, float torque_max = 65.0f);

    int SetMit(float position, float velocity, float kp, float kd, float torque);

    int BuildControlFrame(CanfdFrame &frame) const;

    int BuildEnableFrame(CanfdFrame &frame) const;

    int BuildDisableFrame(CanfdFrame &frame) const;

    int BuildSaveZeroFrame(CanfdFrame &frame) const;

    int ProcessFeedback(const CanfdFrame &frame);

    CubemarsData GetData() const;

    uint32_t GetFeedbackCount() const;

    CanfdDevice GetDevice() const { return device_; }

    uint32_t GetFeedbackId() const { return 0x00; }

    uint32_t GetControlFrameId() const { return 0x01; }

    static void PackMit(float position, float velocity, float kp, float kd, float torque, uint8_t out[8], float position_max, float velocity_max, float kp_max, float kd_max, float torque_max);

    private:
    struct Feedback
    {
        uint8_t motor_id;
        uint16_t position;
        uint16_t velocity;
        uint16_t torque;
    };

    static constexpr float kp_max_ = 500.0f;
    static constexpr float kd_max_ = 5.0f;

    static float Decode(uint16_t raw, float limit, uint8_t bits);

    static uint16_t Encode(float value, float minimum, float maximum, uint8_t bits);

    static bool DecodeFeedback(const uint8_t data[8], uint8_t length, Feedback &decoded);

    static void CommandPayload(uint8_t tail, uint8_t data[8]);

    static float WrapDelta(float delta, float angle_max);

    int BuildCommandFrame(uint8_t tail, CanfdFrame &frame) const;

    CanfdDevice device_ = CANFD_DEVICE_COUNT;

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

using MotorCubemarsMit = Cubemars;
using MotorCubemars = Cubemars;
using MotorCubeMars = Cubemars;
