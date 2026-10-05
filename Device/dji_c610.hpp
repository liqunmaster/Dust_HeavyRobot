#pragma once

#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "bsp_fdcan.hpp"
#include "dji_motor.hpp"
#include "type_math.hpp"

enum C610_ID
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

class c610
{
    public:
    int init(fdcan_device device, C610_ID id, float gear_ratio = 1.0F);

    int set_current(float current);

    int build_control_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    uint32_t get_feedback_count() const;

    C610RxData get_rx_data() const;

    C610Data get_data() const;

    fdcan_device device() const
    {
        return device_;
    }

    uint8_t motor_id() const { return static_cast<uint8_t>(id_); }

    uint32_t feedback_id() const
    {
        return 0x200U + motor_id();
    }

    uint32_t command_frame_id() const
    {
        return motor_id() <= 4U ? 0x200U : 0x1FFU;
    }

    private:
    static constexpr uint16_t encoder_resolution_ = 8192U;

    static constexpr int16_t current_raw_limit_ = 10000;

    static constexpr float current_limit_ = 10.0F;

    static C610RxData decode_feedback(const uint8_t *data);

    void unpack_feedback(const uint8_t *data);

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    C610_ID id_ = C610_ID_0x201;

    float gear_ratio_ = 1.0F;

    bool initialized_ = false;

    C610RxData rx_data_{};

    C610Data data_{};

    int64_t total_encoder_ = 0;

    bool encoder_initialized_ = false;

    mutable struct k_spinlock feedback_lock_{};

    atomic_t feedback_count_{};
};
