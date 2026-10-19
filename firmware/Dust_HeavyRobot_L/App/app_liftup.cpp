#include "app_liftup.hpp"

#include <string.h>

namespace
{
    Liftup liftup;

    K_THREAD_STACK_DEFINE(lift_task, 2048);

    K_SEM_DEFINE(lift_sem, 0, 1);
}

void Liftup::TimerCallback(struct k_timer *arg1)
{
    k_sem_give(&lift_sem);
}

void Liftup::ControlStep(float dt_s)
{
    const uint32_t now_ms = k_uptime_get_32();
    for (uint8_t index = 0; index < 4; ++index) {
        const CanfdDevice bus = static_cast<CanfdDevice>(index);
        MotorFeedbackTopicData feedback{};
        PlatformCanfdSnapshot raw_feedback{};
        bool has_feedback = false;
        if (platform_canfd_latest(feedback_handles_[index], raw_feedback) == 0 &&
            now_ms - raw_feedback.timestamp_ms < 100) {
            CanfdFrame frame{};
            frame.id = motors_[index].GetFeedbackId();
            frame.id_type = CANFD_ID_STANDARD;
            frame.protocol = CANFD_PROTOCOL_CLASSIC;
            frame.length = raw_feedback.length;
            memcpy(frame.data, raw_feedback.data, raw_feedback.length);
            if (raw_feedback.sequence != feedback_sequences_[index]) {
                feedback_sequences_[index] = raw_feedback.sequence;
                feedback_valid_[index] = motors_[index].ProcessFeedback(frame) == 0;
            }
            const CubemarsData data = motors_[index].GetData();
            feedback.bus = bus;
            feedback.id = frame.id;
            feedback.kind = MotorChannelKind::cubemars;
            feedback.timestamp_ms = raw_feedback.timestamp_ms;
            feedback.speed_rad_s = data.now_omega;
            feedback.angle_rad = data.now_total_angle;
            feedback.torque_nm = data.now_torque;
            feedback.valid_count = motors_[index].GetFeedbackCount();
            has_feedback = feedback_valid_[index];
        }
        const bool feedback_fresh = has_feedback;
        LiftSetpoint request{};
        const bool command_fresh = remote_channel_latest_lift(index, request) == 0 && now_ms - request.timestamp_ms < 100;
        const float target_rad = command_fresh ? request.relative_angle_rad : 0.0f;
        float torque_nm = 0.0f;
        float measured_rad = 0.0f;
        if (feedback_fresh) {
            if (!has_origin_[index]) {
                origin_rad_[index] = feedback.angle_rad;
                has_origin_[index] = true;
            }
            measured_rad = (feedback.angle_rad - origin_rad_[index]);
            const float measured_speed = feedback.speed_rad_s;
            torque_nm = loops_[index].Update(target_rad, measured_rad, measured_speed, dt_s);
        } else {
            loops_[index].Reset();
        }

        int result = 0;
        if (feedback_fresh) {
            result = motors_[index].SetMit(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
            if (result == 0) {
                CanfdFrame frame{};
                result = motors_[index].BuildControlFrame(frame);
                if (result == 0) {
                    result = motor_topic_publish_control({bus, frame});
                }
            }
        }
        const LiftMotorStatus snapshot{feedback_fresh, has_feedback ? now_ms - feedback.timestamp_ms : UINT32_MAX, has_feedback ? feedback.valid_count : 0, target_rad, measured_rad, torque_nm, result, 0, 0, 0, 0};
        const k_spinlock_key_t key = k_spin_lock(&status_lock_);
        motor_status_[index] = snapshot;
        k_spin_unlock(&status_lock_, key);
    }
}

void Liftup::ThreadEntry(void *context, void *, void *)
{
    static_cast<Liftup *>(context)->Task();
}

void Liftup::Task()
{
    while (1) {
        k_sem_take(&lift_sem, K_FOREVER);
        ControlStep(0.002f);
    }
}

void Liftup::Init()
{
    if (initialized_) {
        return;
    }

    alg::PIDConfig position_pid{};
    position_pid.kp = 20.0f;
    position_pid.ki = 0.0f;
    position_pid.kd = 0.0f;
    position_pid.output_limit = 50.0f;
    position_pid.dt = 0.002f;

    alg::PIDConfig speed_pid{};
    speed_pid.kp = 2.0f;
    speed_pid.ki = 0.0f;
    speed_pid.kd = 0.0f;
    speed_pid.integral_limit = 65.0f * 0.5f;
    speed_pid.output_limit = 65.0f;
    speed_pid.dt = 0.002f;

    for (uint8_t index = 0; index < 4; ++index) {
        const CanfdDevice bus = static_cast<CanfdDevice>(index);
        int result = motors_[index].Init(bus, 12.5f, 50.0f, 65.0f);
        if (result != 0) {
            return;
        }
        result = canfd_port_bind(motors_[index]);
        if (result != 0) {
            return;
        }
        result = platform_canfd_subscribe(bus, motors_[index].GetFeedbackId(), CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC, feedback_handles_[index]);
        if (result != 0) {
            return;
        }
        loops_[index].Configure(position_pid, speed_pid);
    }
    for (uint8_t index = 0; index < 4; ++index) {
        const int result = canfd_port_enable(motors_[index]);
        if (result != 0) {
            return;
        }
    }
    initialized_ = true;
    k_timer_init(&lift_timer_, TimerCallback, nullptr);
    k_timer_start(&lift_timer_, K_MSEC(2), K_MSEC(2));
    k_thread_create(&lift_thread_, lift_task, K_THREAD_STACK_SIZEOF(lift_task), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void Liftup::SetTarget(uint8_t can_index, float relative_angle_rad)
{
    if (!initialized_) {
        return;
    }
    if (can_index >= 4 || !std::isfinite(relative_angle_rad) ||
        std::fabs(relative_angle_rad) > 12.5f) {
        return;
    }
    remote_channel_publish_lift(can_index, relative_angle_rad);
}

void Liftup::GetStatus(uint8_t can_index, LiftMotorStatus &status)
{
    status = {};
    if (can_index >= 4 || !initialized_) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&status_lock_);
    status = motor_status_[can_index];
    k_spin_unlock(&status_lock_, key);
    CanfdStatistics can_stats{};
    bsp_canfd_get_statistics(static_cast<CanfdDevice>(can_index), &can_stats);
    status.can_tx_completed = can_stats.tx_completed;
    status.can_tx_errors = can_stats.tx_errors;
    status.can_rx_received = can_stats.rx_received;
    status.can_rx_dropped = can_stats.rx_dropped;
}

void app_liftup_init()
{
    liftup.Init();
}

void app_liftup_set_target(uint8_t can_index, float relative_angle_rad)
{
    liftup.SetTarget(can_index, relative_angle_rad);
}

void app_liftup_get_status(uint8_t can_index, LiftMotorStatus &status)
{
    liftup.GetStatus(can_index, status);
}
