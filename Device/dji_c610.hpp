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

// C610 电机 ID 枚举 对应标准帧反馈 ID
enum c610_id
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

// C610 原始反馈数据解包结构体
struct c610_rx_data
{
    uint16_t encoder;
    int16_t omega;
    uint16_t current;
    uint8_t reserved;
    uint8_t error;
} __attribute__((packed));

// C610 处理后的数据结构体
struct c610_data
{
    float now_angle;
    float now_omega;
    float now_current;
    uint32_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
};

// 大疆 C610 无刷电机驱动类
class c610
{
    public:
    int init(fdcan_device device, c610_id id, float gear_ratio = 1.0f);

    int set_current(float current);

    int build_control_frame(fdcan_frame &frame) const;

    int process_feedback(const fdcan_frame &frame);

    uint32_t get_feedback_count() const;

    c610_rx_data get_rx_data() const;

    c610_data get_data() const;

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
     * @return 电机编号
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
     * @brief 获取控制报文 ID
     *
     * @return 控制报文 ID
     */
    uint32_t command_frame_id() const
    {
        return motor_id() <= 4 ? 0x200 : 0x1FF;
    }

    private:
    static constexpr uint16_t encoder_resolution_ = 8192;

    static constexpr int16_t current_raw_limit_ = 10000;

    static constexpr float current_limit_ = 10.0f;

    static c610_rx_data decode_feedback(const uint8_t *data);

    void unpack_feedback(const uint8_t *data);

    fdcan_device device_ = FDCAN_DEVICE_COUNT;

    c610_id id_ = C610_ID_0x201;

    float gear_ratio_ = 1.0f;

    bool initialized_ = false;

    c610_rx_data rx_data_{};

    c610_data data_{};

    int64_t total_encoder_ = 0;

    bool encoder_initialized_ = false;

    mutable struct k_spinlock feedback_lock_{};

    atomic_t feedback_count_{};
};
