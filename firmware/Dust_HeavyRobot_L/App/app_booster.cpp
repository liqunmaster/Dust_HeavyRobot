#include "app_booster.hpp"

#include <string.h>

namespace
{
    Booster booster;

    K_THREAD_STACK_DEFINE(booster_stack, 2048);

    K_SEM_DEFINE(booster_sem, 0, 1);
}

float Booster::WrapAngle(float angle)
{
    return std::remainder(angle, 2.0f * math::pi);
}

void Booster::SetState(BoosterState next, uint32_t now_ms)
{
    state_ = next;
    state_since_ms_ = now_ms;
    if (next == BoosterState::feeding) {
        last_motion_update_ms_ = now_ms;
        feeding_travel_rad_ = 0.0f;
    }
}

int Booster::SendRecovery(bool clear_error)
{
    CanfdFrame frame{};
    const int result = clear_error ? motor_.BuildClearErrorFrame(frame) : motor_.BuildEnableFrame(frame);
    return result == 0 ? canfd_port_send_once(CANFD_DEVICE_CAN2, frame) : result;
}

int Booster::SendTorque(float torque_nm)
{
    int result = motor_.SetMit(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
    if (result == 0) {
        result = canfd_port_submit(motor_);
    }
    return result;
}

void Booster::ResetMotion(float angle, uint32_t now_ms, BoosterState next)
{
    angle_pid_.Reset();
    target_angle_rad_ = angle;
    jam_count_ = 0;
    recovery_count_ = 0;
    jam_retries_ = 0;
    SetState(next, now_ms);
}

void Booster::UpdateMotion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising)
{
    if (!armed) {
        if (state_ != BoosterState::disabled) {
            ResetMotion(angle, now_ms, BoosterState::disabled);
        }
        return;
    }
    if (state_ == BoosterState::disabled || state_ == BoosterState::motor_fault) {
        ResetMotion(angle, now_ms, BoosterState::ready);
    }

    switch (state_) {
        case BoosterState::ready:
            if (rising) {
                target_angle_rad_ = WrapAngle(angle - math::pi / 3.0f);
                jam_count_ = 0;
                jam_retries_ = 0;
                SetState(BoosterState::feeding, now_ms);
            }
            break;
        case BoosterState::feeding:
            feeding_travel_rad_ += std::fabs(speed) * static_cast<float>(now_ms - last_motion_update_ms_) * 0.001f;
            last_motion_update_ms_ = now_ms;

            if (now_ms - state_since_ms_ >= 1000 ||
                feeding_travel_rad_ >= 2.0f * (math::pi / 3.0f)) {
                angle_pid_.Reset();
                SetState(BoosterState::jam_fault, now_ms);
                break;
            }

            if (std::fabs(torque) > 4.0f && std::fabs(speed) < 0.5f) {
                if (++jam_count_ >= 5) {
                    ++jam_retries_;
                    recovery_count_ = 0;
                    target_angle_rad_ = WrapAngle(angle + math::pi / 3.0f);
                    SetState(BoosterState::reversing, now_ms);
                }
            } else {
                jam_count_ = 0;
                if (std::fabs(WrapAngle(target_angle_rad_ - angle)) < 0.05f &&
                    std::fabs(speed) < 0.5f) {
                    SetState(BoosterState::ready, now_ms);
                }
            }
            break;
        case BoosterState::reversing:
            if (std::fabs(WrapAngle(target_angle_rad_ - angle)) < 0.05f && std::fabs(speed) < 0.5f) {
                if (++recovery_count_ >= 5) {
                    SetState(BoosterState::settling, now_ms);
                }
            } else {
                recovery_count_ = 0;
            }
            if (now_ms - state_since_ms_ >= 1000) {
                SetState(BoosterState::settling, now_ms);
            }
            break;
        case BoosterState::settling:
            if (now_ms - state_since_ms_ >= 100) {
                if (jam_retries_ >= 3) {
                    angle_pid_.Reset();
                    SetState(BoosterState::jam_fault, now_ms);
                } else {
                    target_angle_rad_ = WrapAngle(angle - math::pi / 3.0f);
                    jam_count_ = 0;
                    SetState(BoosterState::feeding, now_ms);
                }
            }
            break;
        case BoosterState::jam_fault:
        case BoosterState::disabled:
        case BoosterState::motor_fault:
            break;
    }
}

void Booster::ControlStep()
{
    const uint32_t now_ms = k_uptime_get_32();
    PlatformCanfdSnapshot raw_feedback{};
    bool has_feedback = false;
    uint32_t feedback_timestamp_ms = 0;
    if (platform_canfd_latest(feedback_handle_, raw_feedback) == 0) {
        CanfdFrame frame{};
        frame.id = 0x08;
        frame.id_type = CANFD_ID_STANDARD;
        frame.protocol = CANFD_PROTOCOL_CLASSIC;
        frame.length = raw_feedback.length;
        memcpy(frame.data, raw_feedback.data, raw_feedback.length);
        if (raw_feedback.sequence != feedback_sequence_) {
            feedback_sequence_ = raw_feedback.sequence;
            feedback_valid_ = motor_.ProcessFeedback(frame) == 0;
        }
        has_feedback = feedback_valid_;
        feedback_timestamp_ms = raw_feedback.timestamp_ms;
    }
    const uint32_t age_ms = has_feedback ? now_ms - feedback_timestamp_ms : UINT32_MAX;
    const DmData data = motor_.GetData();
    RemoteTopicData remote{};
    const bool armed = remote_channel_latest_input(remote) == 0 && remote.valid && now_ms - remote.timestamp_ms < 12;
    const bool fresh = age_ms < (armed ? 12 : 30);
    const bool motor_ready = fresh && data.status == 0x01;

    const uint8_t s1 = armed ? remote.switch_right : 0;
    if (!armed) {
        trigger_armed_ = false;
    } else if (s1 == 1 || s1 == 3) {
        trigger_armed_ = true;
    }
    const bool rising = s1 == 2 && trigger_armed_;
    if (rising) {
        trigger_armed_ = false;
    }
    int result = 0;
    float command_torque = 0.0f;

    if (!armed && state_ == BoosterState::jam_fault) {
        ResetMotion(data.now_angle, now_ms, BoosterState::disabled);
    }

    if (!motor_ready) {
        if (state_ != BoosterState::motor_fault && state_ != BoosterState::jam_fault) {
            ResetMotion(data.now_angle, now_ms, BoosterState::motor_fault);
        }
        if (!fresh) {
            offline_ticks_ += timer_period_ms_;
            if (offline_ticks_ >= 500) {
                offline_ticks_ = 0;
                result = SendRecovery(false);
            }
        } else {
            offline_ticks_ = 0;
            fault_ticks_ += timer_period_ms_;
            if (fault_ticks_ >= 100) {
                fault_ticks_ = 0;
                result = SendRecovery(data.status >= 0x08);
            } else {
                result = SendTorque(0.0f);
            }
        }
    } else {
        offline_ticks_ = 0;
        fault_ticks_ = 0;
        UpdateMotion(data.now_angle, data.now_omega, data.now_torque, now_ms, armed, rising);
        if (armed && state_ != BoosterState::jam_fault && state_ != BoosterState::disabled) {
            command_torque = angle_pid_.UpdateAngle(target_angle_rad_, data.now_angle, 0.001f);
        }
        if (armed || state_ != BoosterState::disabled ||
            now_ms - last_keepalive_ms_ >= 5) {
            result = SendTorque(command_torque);
            last_keepalive_ms_ = now_ms;
        }
    }

    if (result != 0) {
        last_error_ = result;
    }
    const BoosterStatus snapshot{state_, fresh, data.status, age_ms, data.now_angle, target_angle_rad_, data.now_omega, data.now_torque, command_torque, jam_retries_, last_error_};
    const k_spinlock_key_t key = k_spin_lock(&status_lock_);
    status_ = snapshot;
    k_spin_unlock(&status_lock_, key);

    const uint32_t next_period_ms = !fresh ? 20 : !armed && state_ == BoosterState::disabled ? 5 : 1;
    if (next_period_ms != timer_period_ms_) {
        timer_period_ms_ = next_period_ms;
        k_timer_start(&booster_timer_, K_MSEC(next_period_ms), K_MSEC(next_period_ms));
    }
}

void Booster::TimerCallback(struct k_timer *arg1)
{
    k_sem_give(&booster_sem);
}

void Booster::ThreadEntry(void *context, void *, void *)
{
    static_cast<Booster *>(context)->Task();
}

void Booster::Task()
{
    while (1) {
        k_sem_take(&booster_sem, K_FOREVER);
        ControlStep();
    }
}

void Booster::Init()
{
    if (initialized_) {
        return;
    }
    const DmMitLimits limits{12.5f, 30.0f, 10.0f};
    int result = motor_.Init(CANFD_DEVICE_CAN2, 0x07, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, limits, CANFD_PROTOCOL_CLASSIC, 0x08, 0x07);
    if (result != 0) {
        return;
    }
    result = canfd_port_bind(motor_, false);
    if (result != 0) {
        return;
    }
    result = platform_canfd_subscribe(CANFD_DEVICE_CAN2, 0x08, CANFD_ID_STANDARD, CANFD_PROTOCOL_CLASSIC, feedback_handle_);
    if (result != 0) {
        return;
    }
    alg::PIDConfig pid{};
    pid.kp = 8.0f;
    pid.ki = 1.0f;
    pid.kd = 0.3f;
    pid.output_limit = 5.0f;
    pid.dt = 0.001f;
    pid.derivative_on_measurement = alg::DFirst::Enable;
    pid.derivative_filter_tau = 0.01f;
    angle_pid_.Configure(pid);
    initialized_ = true;
    k_timer_init(&booster_timer_, TimerCallback, nullptr);
    k_timer_start(&booster_timer_, K_MSEC(1), K_MSEC(1));
    k_thread_create(&booster_thread_, booster_stack, K_THREAD_STACK_SIZEOF(booster_stack), ThreadEntry, this, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void Booster::GetStatus(BoosterStatus &snapshot)
{
    snapshot = {};
    if (!initialized_) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&status_lock_);
    snapshot = status_;
    k_spin_unlock(&status_lock_, key);
}

void app_booster_init()
{
    booster.Init();
}

void app_booster_get_status(BoosterStatus &snapshot)
{
    booster.GetStatus(snapshot);
}
