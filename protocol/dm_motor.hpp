#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "bsp_canfd.hpp"

enum DmControlMode
{
    MOTOR_DM_CONTROL_METHOD_NORMAL_MIT = 1,
    MOTOR_DM_CONTROL_METHOD_NORMAL_ANGLE_OMEGA = 2,
    MOTOR_DM_CONTROL_METHOD_NORMAL_OMEGA = 3,
    MOTOR_DM_CONTROL_METHOD_NORMAL_EMIT = 4,
};

struct DmMitLimits
{
    float position;
    float velocity;
    float torque;
};

struct DmRxData
{
    uint8_t motor_id;
    uint8_t status;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
};

struct DmData
{
    float now_angle;
    float now_omega;
    float now_torque;
    float now_mos_temperature;
    float now_rotor_temperature;
    uint8_t status;
};

class DmMotor
{
    public:
    int Init(CanfdDevice device, uint8_t motor_id, DmControlMode mode, DmMitLimits limits,
             CanfdProtocol protocol, uint16_t master_id = 0, uint16_t tx_id_base = 0);

    int SetMit(float position, float velocity, float kp, float kd, float torque);

    int SetPositionVelocity(float position, float max_velocity);

    int SetVelocity(float velocity);

    int SetPositionTorque(float position, float max_velocity, float current_ratio);

    int BuildControlFrame(CanfdFrame &frame) const;

    int BuildEnableFrame(CanfdFrame &frame) const;

    int BuildDisableFrame(CanfdFrame &frame) const;

    int BuildClearErrorFrame(CanfdFrame &frame) const;

    int BuildSaveZeroFrame(CanfdFrame &frame) const;

    int ProcessFeedback(const CanfdFrame &frame);

    uint32_t GetFeedbackCount() const;

    DmRxData GetRxData() const;

    DmData GetData() const;

    DmControlMode GetMode() const;

    CanfdDevice GetDevice() const { return device_; }

    uint8_t GetMotorId() const { return motor_id_; }

    uint32_t GetFeedbackId() const { return master_id_; }

    CanfdProtocol GetFeedbackProtocol() const { return protocol_; }

    uint32_t GetControlFrameId() const;

    private:
    friend int canfd_port_request_mode(DmMotor &motor, DmControlMode mode);
    friend class MotorPort;

    int BuildModeFrame(DmControlMode mode, CanfdFrame &frame) const;

    int MarkModeRequest(DmControlMode mode);

    static bool ValidMode(DmControlMode mode);

    static uint16_t ControlId(uint16_t tx_id_base, DmControlMode mode);

    static uint16_t Encode(float value, float minimum, float maximum, uint16_t maximum_raw);

    static float Decode(uint16_t raw, float limit, uint16_t maximum_raw);

    static void WriteF32(uint8_t *dst, float value);

    static void WriteU16(uint8_t *dst, uint16_t value);

    static void PackMit(float position, float velocity, float kp, float kd, float torque,
                        uint8_t data[8], const DmMitLimits &limits);

    static void CommandPayload(uint8_t command, uint8_t data[8]);

    static bool DecodeFeedback(const uint8_t data[8], uint8_t motor_id, DmRxData &feedback);

    int BuildCommandFrame(uint8_t command, CanfdFrame &frame) const;

    static int MakeFrame(uint16_t id, const uint8_t *data, uint8_t length, CanfdFrame &frame);

    CanfdDevice device_ = CANFD_DEVICE_COUNT;

    uint8_t motor_id_ = 0;

    uint16_t master_id_ = 0;

    uint16_t tx_id_base_ = 0;

    DmControlMode mode_ = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    CanfdProtocol protocol_ = CANFD_PROTOCOL_CLASSIC;

    DmControlMode requested_mode_ = DmControlMode::MOTOR_DM_CONTROL_METHOD_NORMAL_MIT;

    bool mode_change_pending_ = false;

    bool initialized_ = false;

    DmMitLimits limits_{};

    uint8_t tx_data_[8]{};

    uint8_t tx_length_ = 0;

    DmRxData rx_data_{};

    DmData data_{};

    mutable struct k_spinlock lock_{};

    atomic_t feedback_count_{};
};
