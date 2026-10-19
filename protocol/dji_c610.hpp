#pragma once

#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "bsp_canfd.hpp"
#include "dji_motor.hpp"
#include "type_math.hpp"

enum C610Id
{
    C610_ID_0x201 = 1,
    C610_ID_0x202 = 2,
    C610_ID_0x203 = 3,
    C610_ID_0x204 = 4,
    C610_ID_0x205 = 5,
    C610_ID_0x206 = 6,
    C610_ID_0x207 = 7,
    C610_ID_0x208 = 8,
};

struct C610RxData
{
    uint16_t encoder;
    int16_t omega;
    uint16_t current;
    uint8_t reserved;
    uint8_t error;
} __attribute__((packed));

struct C610Data
{
    float now_angle;
    float now_omega;
    float now_current;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
};

class C610
{
    public:
    int Init(CanfdDevice device, C610Id id, float gear_ratio = 1.0f);

    int SetCurrent(float current);

    int BuildControlFrame(CanfdFrame &frame) const;

    int ProcessFeedback(const CanfdFrame &frame);

    uint32_t GetFeedbackCount() const;

    C610RxData GetRxData() const;

    C610Data GetData() const;

    CanfdDevice GetDevice() const
    {
        return device_;
    }

    uint8_t GetMotorId() const { return static_cast<uint8_t>(id_); }

    uint32_t GetFeedbackId() const
    {
        return 0x200 + GetMotorId();
    }

    uint32_t GetCommandFrameId() const
    {
        return GetMotorId() <= 4 ? 0x200 : 0x1FF;
    }

    private:
    static constexpr uint16_t encoder_resolution_ = 8192;

    static constexpr int16_t current_raw_limit_ = 10000;

    static constexpr float current_limit_ = 10.0f;

    static C610RxData DecodeFeedback(const uint8_t *data);

    void UnpackFeedback(const uint8_t *data);

    CanfdDevice device_ = CANFD_DEVICE_COUNT;

    C610Id id_ = C610_ID_0x201;

    float gear_ratio_ = 1.0f;

    bool initialized_ = false;

    C610RxData rx_data_{};

    C610Data data_{};

    int64_t total_encoder_ = 0;

    bool encoder_initialized_ = false;

    mutable struct k_spinlock feedback_lock_{};

    atomic_t feedback_count_{};
};
