#include "app_booster.hpp"
#include "input.hpp"

namespace
{
    constexpr fdcan_device bus = FDCAN_DEVICE_CAN1;
    constexpr uint8_t  motor_id    = 0x07;
    constexpr uint16_t tx_id_base  = 0x07;
    constexpr uint16_t feedback_id = 0x08;
    constexpr uint32_t period_ms   = 1;
    constexpr uint32_t active_feedback_timeout_ms = 12;
    constexpr uint32_t disabled_feedback_timeout_ms = 30;
    constexpr uint32_t disabled_keepalive_period_ms = 5;
    constexpr uint16_t offline_probe_ticks = 500;
    constexpr uint8_t fault_probe_ticks = 100;
    constexpr uint32_t recovery_timeout_ms = 1000;
    constexpr uint32_t recovery_wait_ms = 100;
    constexpr uint8_t  jam_confirm_ticks = 5;
    constexpr uint8_t  recovery_confirm_ticks = 5;
    constexpr uint8_t  max_jam_retries = 3;
    constexpr float pi = 3.14159265358979323846f;
    constexpr float shot_step_rad = -pi / 3.0f;
    constexpr float jam_torque_nm = 4.0f;
    constexpr float low_speed_rad_s = 0.5f;
    constexpr float angle_tolerance_rad = 0.05f;

    dm_motor motor;
    alg::Pid angle_pid;
    BoosterStatus status{};
    struct k_spinlock status_lock;
    bool trigger_armed = false;
    bool initialized  = false;
    uint8_t jam_count = 0;
    uint8_t recovery_count  = 0;
    uint8_t jam_retries     = 0;
    uint32_t state_since_ms = 0;
    uint32_t last_keepalive_ms = 0;
    uint16_t offline_ticks = offline_probe_ticks - 1;
    uint8_t fault_ticks = 0;
    int last_error = 0;
    float    target_angle_rad = 0.0f;
    BoosterState state = BoosterState::disabled;

    K_THREAD_STACK_DEFINE(booster_stack, 2048);
    struct k_thread booster_thread;
    K_SEM_DEFINE(booster_sem, 0, 1);
    struct k_timer booster_timer;
    uint32_t timer_period_ms = period_ms;

    float wrap_angle(float angle)
    {
        return std::remainder(angle, 2.0F * pi);
    }

    void set_state(BoosterState next, uint32_t now_ms)
    {
        state = next;
        state_since_ms = now_ms;
    }

    int send_recovery(bool clear_error)
    {
        fdcan_frame frame{};
        const int result = clear_error ? motor.build_clear_error_frame(frame) : motor.build_enable_frame(frame);
        return result == 0 ? fdcan_port_send_once(bus, frame) : result;
    }

    int send_torque(float torque_nm)
    {
        int result = motor.set_mit(0.0F, 0.0F, 0.0F, 0.0F, torque_nm);
        if (result == 0) {
            result = fdcan_port_submit(motor);
        }
        return result;
    }

    void reset_motion(float angle, uint32_t now_ms, BoosterState next)
    {
        angle_pid.reset();
        target_angle_rad = angle;
        jam_count = 0U;
        recovery_count = 0U;
        jam_retries = 0U;
        set_state(next, now_ms);
    }

    void update_motion(float angle, float speed, float torque, uint32_t now_ms, bool armed, bool rising)
    {
        if (!armed) {
            if (state != BoosterState::disabled) {
                reset_motion(angle, now_ms, BoosterState::disabled);
            }
            return;
        }
        if (state == BoosterState::disabled || state == BoosterState::motor_fault) {
            reset_motion(angle, now_ms, BoosterState::ready);
        }

        switch (state) {
            case BoosterState::ready:
                if (rising) {
                    target_angle_rad = wrap_angle(angle + shot_step_rad);
                    jam_count = 0U;
                    jam_retries = 0U;
                    set_state(BoosterState::feeding, now_ms);
                }
                break;
            case BoosterState::feeding:
                if (std::fabs(torque) > jam_torque_nm && std::fabs(speed) < low_speed_rad_s) {
                    if (++jam_count >= jam_confirm_ticks) {
                        ++jam_retries;
                        recovery_count = 0U;
                        target_angle_rad = wrap_angle(angle - shot_step_rad);
                        set_state(BoosterState::reversing, now_ms);
                    }
                } else {
                    jam_count = 0U;
                    if (std::fabs(wrap_angle(target_angle_rad - angle)) < angle_tolerance_rad &&
                        std::fabs(speed) < low_speed_rad_s) {
                        set_state(BoosterState::ready, now_ms);
                    }
                }
                break;
            case BoosterState::reversing:
                if (std::fabs(wrap_angle(target_angle_rad - angle)) < angle_tolerance_rad &&
                    std::fabs(speed) < low_speed_rad_s) {
                    if (++recovery_count >= recovery_confirm_ticks) {
                        set_state(BoosterState::settling, now_ms);
                    }
                } else {
                    recovery_count = 0U;
                }
                if (now_ms - state_since_ms >= recovery_timeout_ms) {
                    set_state(BoosterState::settling, now_ms);
                }
                break;
            case BoosterState::settling:
                if (now_ms - state_since_ms >= recovery_wait_ms) {
                    if (jam_retries >= max_jam_retries) {
                        angle_pid.reset();
                        set_state(BoosterState::jam_fault, now_ms);
                    } else {
                        target_angle_rad = wrap_angle(angle + shot_step_rad);
                        jam_count = 0U;
                        set_state(BoosterState::feeding, now_ms);
                    }
                }
                break;
            case BoosterState::jam_fault:
            case BoosterState::disabled:
            case BoosterState::motor_fault:
                break;
        }
    }

    void control_step()
    {
        const uint32_t now_ms = k_uptime_get_32();
        FdcanFeedbackTopicData feedback{};
        const bool has_feedback = fdcan_topic_latest_feedback(bus, feedback_id, FdcanMotorKind::dm, feedback) == 0;
        const uint32_t age_ms = has_feedback ? now_ms - feedback.timestamp_ms : UINT32_MAX;
        const DmData data = motor.get_data();
        input_sample remote{};
        const bool armed = input_get_sample(remote_protocol::dt7, remote) == 0;
        const bool fresh = age_ms < (armed ? active_feedback_timeout_ms : disabled_feedback_timeout_ms);
        const bool motor_ready = fresh && data.status == 0x01;
        // DT7 S1 is the right switch. Require a fresh 1/3 position before each 2.
        const uint8_t s1 = armed ? remote.dt7_data.switch_right : 0U;
        if (!armed) {
            trigger_armed = false;
        } else if (s1 == 1U || s1 == 3U) {
            trigger_armed = true;
        }
        const bool rising = s1 == 2U && trigger_armed;
        if (rising) {
            trigger_armed = false;
        }
        int result = 0;
        float command_torque = 0.0F;

        if (!armed && state == BoosterState::jam_fault) {
            reset_motion(data.now_angle, now_ms, BoosterState::disabled);
        }

        if (!motor_ready) {
            if (state != BoosterState::motor_fault && state != BoosterState::jam_fault) {
                reset_motion(data.now_angle, now_ms, BoosterState::motor_fault);
            }
            if (!fresh) {
                if (++offline_ticks >= offline_probe_ticks) {
                    offline_ticks = 0;
                    result = send_recovery(false);
                }
            } else {
                offline_ticks = 0;
                if (++fault_ticks >= fault_probe_ticks) {
                    fault_ticks = 0;
                    result = send_recovery(data.status >= 0x08U);
                } else {
                    result = send_torque(0.0F);
                }
            }
        } else {
            offline_ticks = 0;
            fault_ticks = 0;
            update_motion(data.now_angle, data.now_omega, data.now_torque, now_ms, armed, rising);
            if (armed && state != BoosterState::jam_fault && state != BoosterState::disabled) {
                command_torque = angle_pid.update_angle(target_angle_rad, data.now_angle,  static_cast<float>(period_ms) * 0.001F);
            }
            if (armed || state != BoosterState::disabled ||
                now_ms - last_keepalive_ms >= disabled_keepalive_period_ms) {
                result = send_torque(command_torque);
                last_keepalive_ms = now_ms;
            }
        }

        if (result != 0) {
            last_error = result;
        }
        const BoosterStatus snapshot{state, fresh, data.status, age_ms, data.now_angle, target_angle_rad, data.now_omega, data.now_torque, command_torque, jam_retries, last_error};
        const k_spinlock_key_t key = k_spin_lock(&status_lock);
        status = snapshot;
        k_spin_unlock(&status_lock, key);

        const uint32_t next_period_ms = !armed && state == BoosterState::disabled ?
            disabled_keepalive_period_ms : period_ms;
        if (next_period_ms != timer_period_ms) {
            timer_period_ms = next_period_ms;
            k_timer_start(&booster_timer, K_MSEC(next_period_ms), K_MSEC(next_period_ms));
        }
    }

    void timer_callback(struct k_timer *)
    {
        k_sem_give(&booster_sem);
    }

    void thread_entry(void *, void *, void *)
    {
        while (1) {
            k_sem_take(&booster_sem, K_FOREVER);
            control_step();
        }
    }
}

void app_booster_init()
{
    if (initialized) {
        return;
    }
    const DmMitLimits limits{12.5F, 30.0F, 10.0F};
    int result = motor.init(bus, motor_id, MOTOR_DM_CONTROL_METHOD_NORMAL_MIT, limits, FDCAN_PROTOCOL_CLASSIC, feedback_id, tx_id_base);
    if (result != 0) {
        return;
    }
    result = fdcan_port_bind(motor, false);
    if (result != 0) {
        return;
    }
    alg::PidConfig pid{};
    pid.kp = 8.0F;
    pid.ki = 1.0F;
    pid.kd = 0.3F;
    pid.output_limit = 5.0F;
    pid.dt = 0.001F;
    angle_pid.configure(pid);
    initialized = true;
    k_timer_init(&booster_timer, timer_callback, nullptr);
    k_timer_start(&booster_timer, K_MSEC(period_ms), K_MSEC(period_ms));
    k_thread_create(&booster_thread, booster_stack, K_THREAD_STACK_SIZEOF(booster_stack), thread_entry, nullptr, nullptr, nullptr, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

void app_booster_get_status(BoosterStatus &snapshot)
{
    snapshot = {};
    if (!initialized) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&status_lock);
    snapshot = status;
    k_spin_unlock(&status_lock, key);
}
