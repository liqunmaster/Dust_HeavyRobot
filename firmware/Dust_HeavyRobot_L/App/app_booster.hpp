#pragma once

#include <cmath>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "dm_motor.hpp"
#include "canfd_bus.hpp"
#include "motor_channel.hpp"
#include "motor_port.hpp"
#include "pid.hpp"
#include "type_math.hpp"
#include "remote_channel.hpp"

enum class BoosterState : uint8_t
{
    disabled,
    ready,
    feeding,
    reversing,
    settling,
    jam_fault,
    motor_fault,
};

struct BoosterStatus
{
    BoosterState state;
    bool feedback_fresh;
    uint8_t motor_status;
    uint32_t feedback_age_ms;
    float angle_rad;
    float target_angle_rad;
    float speed_rad_s;
    float measured_torque_nm;
    float command_torque_nm;
    uint8_t jam_retries;
    int last_error;
};

class Booster
{
    public:
    void Init();

    void GetStatus(BoosterStatus &snapshot);

    private:
    float WrapAngle(float angle);

    void SetState(BoosterState next, uint32_t now_ms);

    int SendRecovery(bool clear_error);

    int SendTorque(float torque_nm);

    void ResetMotion(float angle, uint32_t now_ms, BoosterState next);

    void UpdateMotion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising);

    void ControlStep();

    static void TimerCallback(struct k_timer *arg1);

    static void ThreadEntry(void *arg1, void *arg2, void *arg3);

    void Task();

    DmMotor motor_{};
    PlatformCanfdRxHandle feedback_handle_{};
    uint32_t feedback_sequence_ = 0;
    bool feedback_valid_ = false;
    alg::Pid angle_pid_{};
    BoosterStatus status_{};
    struct k_spinlock status_lock_{};
    bool trigger_armed_ = false;
    bool initialized_ = false;
    uint8_t jam_count_ = 0;
    uint8_t recovery_count_ = 0;
    uint8_t jam_retries_ = 0;
    uint32_t state_since_ms_ = 0;
    uint32_t last_motion_update_ms_ = 0;
    float feeding_travel_rad_ = 0.0f;
    uint32_t last_keepalive_ms_ = 0;
    uint16_t offline_ticks_ = 499;
    uint8_t fault_ticks_ = 0;
    int last_error_ = 0;
    float target_angle_rad_ = 0.0f;
    BoosterState state_ = BoosterState::disabled;
    struct k_thread booster_thread_{};
    struct k_timer booster_timer_{};
    uint32_t timer_period_ms_ = 1;
};

void app_booster_init();

void app_booster_get_status(BoosterStatus &status);
