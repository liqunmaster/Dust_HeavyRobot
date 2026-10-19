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

enum C620Id
{
    C620_ID_0x201 = 1,
    C620_ID_0x202 = 2,
    C620_ID_0x203 = 3,
    C620_ID_0x204 = 4,
    C620_ID_0x205 = 5,
    C620_ID_0x206 = 6,
    C620_ID_0x207 = 7,
    C620_ID_0x208 = 8,
};

struct C620RxData
{
    uint16_t encoder;
    int16_t omega;
    uint16_t current;
    uint8_t temperature;
    uint8_t error;
} __attribute__((packed));

struct C620Data
{
    float now_angle;
    float now_omega;
    float now_current;
    float now_temperature;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
};

class C620
{
    public:
    int Init(CanfdDevice device, C620Id id, float gear_ratio = 1.0f);

    int SetCurrent(float current);

    int BuildControlFrame(CanfdFrame &frame) const;

    int ProcessFeedback(const CanfdFrame &frame);

    uint32_t GetFeedbackCount() const;

    C620RxData GetRxData() const;

    C620Data GetData() const;

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

    static constexpr int16_t current_raw_limit_ = 16384;

    static constexpr float current_limit_ = 20.0f;

    static C620RxData DecodeFeedback(const uint8_t *data);

    void UnpackFeedback(const uint8_t *data);

    CanfdDevice device_ = CANFD_DEVICE_COUNT;

    C620Id id_ = C620_ID_0x201;

    float gear_ratio_ = 1.0f;

    bool initialized_ = false;

    C620RxData rx_data_{};

    C620Data data_{};

    int64_t total_encoder_ = 0;

    bool encoder_initialized_ = false;

    mutable struct k_spinlock feedback_lock_{};

    atomic_t feedback_count_{};
};
