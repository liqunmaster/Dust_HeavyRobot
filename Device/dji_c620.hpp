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

// C620 电机 ID 枚举，对应标准帧反馈 ID（0x201~0x208）
enum C620_ID
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

// C620 原始反馈数据解包结构体
struct C620RxData
{
    uint16_t encoder;
    int16_t omega;
    uint16_t current;
    uint8_t temperature;
    uint8_t error;
} __attribute__((packed));

// C620 处理后的数据结构体（角度、角速度、电流、温度等）
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

// 大疆 C620 无刷电机驱动类
class c620
{
    public:
    int init(fdcan_device device, C620_ID id, float gear_ratio = 1.0f);

    int set_current(float current);

    int build_control_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    uint32_t get_feedback_count() const;

    C620RxData get_rx_data() const;

    C620Data get_data() const;

    /**
     * @brief 获取绑定的 CAN 通道编号
     *
     * @return 通道编号
    */
    fdcan_device device() const
    {
        return device_;
    }

    /**
     * @brief 获取电机标准 ID 对应的编号
     *
     * @return 电机编号（1~8）
    */
    uint8_t motor_id() const { return static_cast<uint8_t>(id_); }

    /**
     * @brief 获取反馈报文 ID
     *
     * @return 反馈报文 ID
    */
    uint32_t feedback_id() const
    {
        return 0x200 + motor_id();
    }

    /**
     * @brief 获取控制报文 ID（前 4 个用 0x200，后 4 个用 0x1FF）
     *
     * @return 控制报文 ID
    */
    uint32_t command_frame_id() const
    {
        return motor_id() <= 4 ? 0x200 : 0x1FF;
    }

    private:
    static constexpr uint16_t encoder_resolution_ = 8192;

    static constexpr int16_t current_raw_limit_ = 16384;

    static constexpr float current_limit_ = 20.0f;

    static C620RxData decode_feedback(const uint8_t *data);

    void unpack_feedback(const uint8_t *data);

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    C620_ID id_ = C620_ID_0x201;

    float gear_ratio_ = 1.0f;

    bool initialized_ = false;

    C620RxData rx_data_{};

    C620Data data_{};

    int64_t total_encoder_ = 0;

    bool encoder_initialized_ = false;

    mutable struct k_spinlock feedback_lock_{};

    atomic_t feedback_count_{};
};